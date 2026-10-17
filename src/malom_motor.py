"""
Malom motor: az OpenSpiel nine_mens_morris játékszabályai és egy alfa-béta kereső gépi játékos,
a kamera és a robotkar nyelvére fordítva.

  - tábla: 24 karakter a kamera betűivel ('P' piros = ember, 'Z' zöld = robot, '.' üres),
    a pontok az OpenSpiel számozása szerint (0-23, soronként fentről, balról jobbra)
  - lépéskód (OpenSpiel): 0-23 = lerakás vagy levétel azon a ponton,
    24 + honnan*24 + hová = tologatás / ugrás
"""
import random
import time

import numpy as np
import pyspiel

# Nehézségi szintek: alfa-béta keresés (minimax) egy malom-értékelő függvénnyel.
#   max_melyseg: hány lépést (fél-lépést) néz előre legfeljebb; idokorlat: s, ennyit gondolkodhat
#   (iteratív mélyítés: amíg az idő engedi, egyre mélyebbre néz; a nehéz szint kb. 5-7 lépésig jut)
NEHEZSEGEK = {
    "konnyu":  {"nev": "Könnyű",  "max_melyseg": 1,  "idokorlat": 1.0},
    "kozepes": {"nev": "Közepes", "max_melyseg": 3,  "idokorlat": 2.0},
    "nehez":   {"nev": "Nehéz",   "max_melyseg": 20, "idokorlat": 4.0},
}

# --- a tábla geometriája az OpenSpiel számozásában ---
# a pontok helye a 7x7-es rácson (oszlop, sor) -> index a megfigyelési tenzorban (sor*7 + oszlop)
_HELY = [(0, 0), (3, 0), (6, 0), (1, 1), (3, 1), (5, 1), (2, 2), (3, 2), (4, 2),
         (0, 3), (1, 3), (2, 3), (4, 3), (5, 3), (6, 3), (2, 4), (3, 4), (4, 4),
         (1, 5), (3, 5), (5, 5), (0, 6), (3, 6), (6, 6)]
_TENZOR_INDEX = [sor * 7 + oszlop for oszlop, sor in _HELY]
MALMOK = [(0, 1, 2), (3, 4, 5), (6, 7, 8), (9, 10, 11), (12, 13, 14), (15, 16, 17), (18, 19, 20), (21, 22, 23),
          (0, 9, 21), (3, 10, 18), (6, 11, 15), (1, 4, 7), (16, 19, 22), (8, 12, 17), (5, 13, 20), (2, 14, 23)]
_VONALAK = [(0, 1), (1, 2), (3, 4), (4, 5), (6, 7), (7, 8), (9, 10), (10, 11), (12, 13), (13, 14), (15, 16),
            (16, 17), (18, 19), (19, 20), (21, 22), (22, 23), (0, 9), (9, 21), (3, 10), (10, 18), (6, 11),
            (11, 15), (1, 4), (4, 7), (16, 19), (19, 22), (8, 12), (12, 17), (5, 13), (13, 20), (2, 14), (14, 23)]
SZOMSZEDOK = [[] for _ in range(24)]
for _a, _b in _VONALAK:
    SZOMSZEDOK[_a].append(_b)
    SZOMSZEDOK[_b].append(_a)


def _lerakhato(allapot):
    for sor in str(allapot).splitlines():
        if sor.startswith("Men to deploy:"):
            return [int(x) for x in sor.split(":")[1].split()]
    return [0, 0]


def ertekeles(allapot, jatekos):
    """
    A helyzet értéke a "jatekos" szemszögéből (nagyobb = jobb neki). Szempontok:
    bábuk száma (táblán + még lerakható), kész malmok, nyitott kettesek (2 saját + 1 üres egy malomvonalon),
    mozgékonyság a tologatási szakaszban (a mozgásképtelen fél veszít).
    """
    if allapot.is_terminal():
        return 100000 * allapot.returns()[jatekos]
    tenzor = np.asarray(allapot.observation_tensor()).reshape(5, 49)
    feher, fekete = tenzor[0][_TENZOR_INDEX], tenzor[1][_TENZOR_INDEX]
    en, ellen = (feher, fekete) if jatekos == 0 else (fekete, feher)
    lerakhato = _lerakhato(allapot)
    en_le, ellen_le = lerakhato[jatekos], lerakhato[1 - jatekos]

    pont = 100.0 * ((en.sum() + en_le) - (ellen.sum() + ellen_le))
    for m in MALMOK:
        e, f = en[list(m)].sum(), ellen[list(m)].sum()
        if e == 3:
            pont += 25
        elif f == 3:
            pont -= 25
        elif e == 2 and f == 0:
            pont += 12
        elif f == 2 and e == 0:
            pont -= 12
    ures = 1 - en - ellen
    if en_le == 0 and en.sum() > 3:
        mozgas = sum(1 for p in range(24) if en[p] for q in SZOMSZEDOK[p] if ures[q])
        pont += 3 * mozgas - (300 if mozgas == 0 else 0)
    if ellen_le == 0 and ellen.sum() > 3:
        mozgas = sum(1 for p in range(24) if ellen[p] for q in SZOMSZEDOK[p] if ures[q])
        pont -= 3 * mozgas - (300 if mozgas == 0 else 0)
    return pont


class _Idotullepes(Exception):
    pass


class AlfaBetaJatekos:
    """Minimax alfa-béta vágással és iteratív mélyítéssel (időkorláton belül a legmélyebb kész keresés)."""

    def __init__(self, max_melyseg, idokorlat):
        self.max_melyseg, self.idokorlat = max_melyseg, idokorlat
        self.utolso_melyseg = 0

    def step(self, allapot):
        self.hatarido = time.time() + self.idokorlat
        jatekos = allapot.current_player()
        lepesek = list(allapot.legal_actions())
        random.shuffle(lepesek)                      # azonos értékű lépések közül véletlenszerűen
        legjobb, self.utolso_melyseg = lepesek[0], 0
        for melyseg in range(1, self.max_melyseg + 1):
            try:
                ertek, lepes = self._gyoker(allapot, melyseg, jatekos, lepesek)
            except _Idotullepes:
                break
            legjobb, self.utolso_melyseg = lepes, melyseg
            lepesek.sort(key=lambda a: a != lepes)   # a legjobb lépést nézi először a következő körben
            if abs(ertek) > 50000:                   # biztos nyerés / vereség: nincs mit tovább keresni
                break
        return legjobb

    def _gyoker(self, allapot, melyseg, jatekos, lepesek):
        alfa, legjobb = -1e18, lepesek[0]
        for a in lepesek:
            kovetkezo = allapot.clone()
            kovetkezo.apply_action(a)
            v = self._alfabeta(kovetkezo, self._uj_melyseg(allapot, kovetkezo, melyseg), alfa, 1e18, jatekos)
            if v > alfa:
                alfa, legjobb = v, a
        return alfa, legjobb

    @staticmethod
    def _uj_melyseg(elozo, kovetkezo, melyseg):
        # a malom utáni levétel ugyanannak a játékosnak a következő lépése: nem számít külön mélységnek
        if not kovetkezo.is_terminal() and kovetkezo.current_player() == elozo.current_player():
            return melyseg
        return melyseg - 1

    def _alfabeta(self, allapot, melyseg, alfa, beta, jatekos):
        if time.time() > self.hatarido:
            raise _Idotullepes
        if melyseg <= 0 or allapot.is_terminal():
            return ertekeles(allapot, jatekos)
        maximalo = allapot.current_player() == jatekos
        for a in allapot.legal_actions():
            kovetkezo = allapot.clone()
            kovetkezo.apply_action(a)
            v = self._alfabeta(kovetkezo, self._uj_melyseg(allapot, kovetkezo, melyseg), alfa, beta, jatekos)
            if maximalo:
                alfa = max(alfa, v)
            else:
                beta = min(beta, v)
            if alfa >= beta:
                break
        return alfa if maximalo else beta


class MalomMotor:
    def __init__(self, robot_kezd, robot_szin="Z", ember_szin="P", nehezseg="kozepes"):
        self.jatek = pyspiel.load_game("nine_mens_morris")
        self.allapot = self.jatek.new_initial_state()
        self.robot_jatekos = 0 if robot_kezd else 1         # 0 = fehér (W), ő kezd
        self.robot_szin, self.ember_szin = robot_szin, ember_szin
        beallitas = NEHEZSEGEK[nehezseg]
        self.nehezseg_nev = beallitas["nev"]
        self.bot = AlfaBetaJatekos(beallitas["max_melyseg"], beallitas["idokorlat"])

    # ---------- állapot ----------
    def _betu(self, jel):
        """OpenSpiel jel ('W' / 'B' / '.') -> kamera betű."""
        if jel == ".":
            return "."
        jatekos = 0 if jel == "W" else 1
        return self.robot_szin if jatekos == self.robot_jatekos else self.ember_szin

    def tabla(self, allapot=None):
        """A tábla 24 karakteren, kamera betűkkel."""
        szoveg = str(allapot or self.allapot).split("\n\n")[0]
        return "".join(self._betu(c) for c in szoveg if c in "WB.")[:24]

    def _szamok(self, allapot=None):
        """(lerakható bábuk, táblán lévő bábuk) játékosonként: ([w, b], [w, b])."""
        lerakhato = szam = [9, 9]
        for sor in str(allapot or self.allapot).splitlines():
            if sor.startswith("Men to deploy:"):
                lerakhato = [int(x) for x in sor.split(":")[1].split()]
            elif sor.startswith("Num men:"):
                szam = [int(x) for x in sor.split(":")[1].split()]
        return lerakhato, szam

    def vege(self):
        return self.allapot.is_terminal()

    def eredmeny(self):
        """'robot', 'ember' vagy 'döntetlen' (csak a játék végén)."""
        e = self.allapot.returns()[self.robot_jatekos]
        return "robot" if e > 0 else "ember" if e < 0 else "döntetlen"

    def robot_jon(self):
        return not self.vege() and self.allapot.current_player() == self.robot_jatekos

    def levetel_jon(self, allapot=None):
        """Malom után: a soron lévő játékosnak most le kell vennie egy ellenséges bábut."""
        return "Capture time" in str(allapot or self.allapot)

    def ember_feladata(self):
        """Szöveges útmutató az ember következő lépéséhez."""
        if self.levetel_jon():
            return "malmod lett: vegyél le egy zöld bábut a tábláról"
        lerakhato, szam = self._szamok()
        en = 1 - self.robot_jatekos
        if lerakhato[en] > 0:
            return f"tegyél le egy piros bábut egy üres pontra (még {lerakhato[en]} db)"
        if szam[en] == 3:
            return "3 bábud maradt: bármelyikkel bármelyik üres pontra ugorhatsz"
        return "tolj egy piros bábut egy szomszédos üres pontra"

    # ---------- lépések ----------
    @staticmethod
    def lepes_bontasa(lepes):
        """Lépéskód -> (honnan, hová): lerakásnál/levételnél honnan = None, hová = a pont."""
        if lepes < 24:
            return None, lepes
        return (lepes - 24) // 24, (lepes - 24) % 24

    def lepes_szovege(self, lepes):
        honnan, hova = self.lepes_bontasa(lepes)
        if honnan is not None:
            return f"{honnan} -> {hova}"
        return f"levétel: {hova}" if self.levetel_jon() else f"lerakás: {hova}"

    def lep(self, lepes):
        self.allapot.apply_action(lepes)

    def robot_lepese(self):
        """A gépi játékos lépése (a robot soron van)."""
        return self.bot.step(self.allapot)

    def ember_lepesei(self, uj_tabla):
        """
        Megkeresi, milyen szabályos emberi lépés(ek) visznek a mostani tábláról az újra (amit a kamera lát).
        Egy lépés (lerakás / tologatás / levétel), vagy kettő: lépés + az ebből lett malom utáni levétel.
        Visszatér: a lépéskódok listája, vagy None, ha nincs ilyen.
        """
        if self.robot_jon() or self.vege():
            return None
        for a in self.allapot.legal_actions():
            s1 = self.allapot.clone()
            s1.apply_action(a)
            if self.tabla(s1) == uj_tabla:
                return [a]
            # malom lett, és a levétel is megtörtént egy stabil helyzeten belül
            if not s1.is_terminal() and s1.current_player() != self.robot_jatekos and self.levetel_jon(s1):
                for b in s1.legal_actions():
                    s2 = s1.clone()
                    s2.apply_action(b)
                    if self.tabla(s2) == uj_tabla:
                        return [a, b]
        return None