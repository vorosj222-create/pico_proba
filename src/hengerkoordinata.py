import math
import threading
import time

from inverz_kinematika import InverzKinematika


class HengerKinematika:
    """
    Hengerkoordináták (r, phi, z) <-> motorlépések.
      r   : a fogáspont vízszintes távolsága a J1 tengelytől [mm]
      phi : elfordulás a J1 nullapontjához képest [fok], pozitív = óramutatóval ellentétes
      z   : a fogáspont magassága a váll (J2) tengelyéhez képest [mm]
    A számolást a meglévő InverzKinematika osztály végzi (phi = 0 síkban),
    a J1 pedig közvetlenül a phi szögből jön.
    """

    # Csuklókorlátok fokban (a nullapont = homing utáni alaphelyzet)
    J1_MIN, J1_MAX = -180.0, 180.0
    J2_MIN, J2_MAX = -90.0, 31.0
    J3_MIN, J3_MAX = -90.0, 49.0   # -90: eddig tudja a J4 szervó függőlegesen tartani a fogót

    # A firmware-ben beállított motorhatárok (main.cpp: setSpeedInHz / setAcceleration)
    MAX_LEPES_SEB = 12000.0     # lépés/s
    MAX_LEPES_GYORS = 15000.0   # lépés/s²

    def __init__(self, ik=None):
        self.ik = ik if ik is not None else InverzKinematika()
        # Ugyanaz a számolás, de egészre kerekítés nélkül: a pályatervezőnek sima (zajmentes)
        # csuklópálya kell, különben a kerekítési zajt görbületnek látja és feleslegesen lassít.
        self._ik_float = InverzKinematika()
        self._ik_float.L1, self._ik_float.L2 = self.ik.L1, self.ik.L2
        self._ik_float.Z_OFFSET = self.ik.Z_OFFSET
        self._ik_float.fok_to_lepes = lambda fok: fok * self._ik_float.STEPS_PER_DEGREE

    def henger_to_lepes(self, r, phi, z, egesz=True):
        """(r, phi, z) -> (j1, j2, j3) lépés, vagy None, ha nem elérhető / korláton kívül.
        egesz=False: tört lépésszámok (a pályatervezőnek)."""
        if r < 0:
            return None
        ik = self.ik if egesz else self._ik_float
        eredmeny = ik.koordinata_szamitas(r, 0.0, z)
        if eredmeny is None:
            return None
        _, j2, j3 = eredmeny
        j1 = ik.fok_to_lepes(phi)
        if not self.korlaton_belul(j1, j2, j3):
            return None
        return j1, j2, j3

    def lepes_to_henger(self, j1, j2, j3):
        """(j1, j2, j3) lépés -> (r, phi, z)"""
        r, _, z = self.ik.direkt_kinematika(0, j2, j3)
        phi = round(self.ik.lepes_to_fok(j1), 1)
        return r, phi, z

    def korlaton_belul(self, j1, j2, j3):
        f1 = self.ik.lepes_to_fok(j1)
        f2 = self.ik.lepes_to_fok(j2)
        f3 = self.ik.lepes_to_fok(j3)
        return (self.J1_MIN <= f1 <= self.J1_MAX and
                self.J2_MIN <= f2 <= self.J2_MAX and
                self.J3_MIN <= f3 <= self.J3_MAX)


class UtvonalTervezo:
    """
    Egyenes szakaszokból (hengerkoordinátában) álló pálya, lekerekített töréspontokkal.
    Az időzítés úgy készül, hogy egyik motor se lépje túl a maximális sebességet és gyorsulást,
    és a pálya közben sehol ne álljon meg (csak az elején és a végén).
    """

    PHI_SKALA_MM = 150.0        # 1 radián phi ennyi mm-nek számít a pálya "hosszában" (tipikus munkasugár)
    LEKEREKITES_MAX_MM = 15.0   # a töréspont előtt/után legfeljebb ennyivel kezd fordulni
    MINTA_MM = 0.5              # a geometriai pálya felbontása
    MAX_FOGO_SEB = 300.0        # mm/s, a fogó maximális pályasebessége
    BIZTONSAG = 0.9             # a motorhatárok ennyiszeresét használjuk ki
    DT = 0.03                   # s, ennyi időnként megy ki egy QMOVE

    def __init__(self, henger):
        self.h = henger

    # ---------- geometria ----------
    def _q(self, p):
        r, phi, z = p
        return (r, math.radians(phi) * self.PHI_SKALA_MM, z)

    def _p(self, q):
        r, s, z = q
        return (r, math.degrees(s / self.PHI_SKALA_MM), z)

    @staticmethod
    def _kul(a, b):
        return tuple(x - y for x, y in zip(a, b))

    @staticmethod
    def _hossz(v):
        return math.sqrt(sum(x * x for x in v))

    def _geometriai_pontok(self, pontok):
        """Töréspontos pálya sűrű mintavételezése, a töréspontokon másodfokú Bézier-ívvel."""
        q = [self._q(p) for p in pontok]
        # egymást követő azonos pontok kiszűrése
        tiszta = [q[0]]
        for p in q[1:]:
            if self._hossz(self._kul(p, tiszta[-1])) > 1e-6:
                tiszta.append(p)
        q = tiszta
        if len(q) < 2:
            return [self._p(q[0])]

        # szakaszonkénti kezdő- és végpontok a lekerekítések után
        darabok = []  # ("egyenes", A, B) vagy ("iv", A, K, B)
        eleje = q[0]
        for i in range(1, len(q) - 1):
            be = self._kul(q[i], q[i - 1]); ki = self._kul(q[i + 1], q[i])
            lb, lk = self._hossz(be), self._hossz(ki)
            d = min(self.LEKEREKITES_MAX_MM, 0.5 * lb, 0.5 * lk)
            a = tuple(q[i][j] - be[j] / lb * d for j in range(3))
            b = tuple(q[i][j] + ki[j] / lk * d for j in range(3))
            darabok.append(("egyenes", eleje, a))
            darabok.append(("iv", a, q[i], b))
            eleje = b
        darabok.append(("egyenes", eleje, q[-1]))

        minta = [darabok[0][1]]
        for d in darabok:
            if d[0] == "egyenes":
                _, a, b = d
                n = max(1, int(math.ceil(self._hossz(self._kul(b, a)) / self.MINTA_MM)))
                for k in range(1, n + 1):
                    t = k / n
                    minta.append(tuple(a[j] + (b[j] - a[j]) * t for j in range(3)))
            else:
                _, a, c, b = d
                hossz = self._hossz(self._kul(c, a)) + self._hossz(self._kul(b, c))
                n = max(2, int(math.ceil(hossz / self.MINTA_MM)))
                for k in range(1, n + 1):
                    t = k / n
                    minta.append(tuple((1 - t) ** 2 * a[j] + 2 * (1 - t) * t * c[j] + t ** 2 * b[j]
                                       for j in range(3)))
        return [self._p(m) for m in minta]

    # ---------- tervezés ----------
    def tervez(self, pontok, max_fogo_seb=None):
        """
        pontok: [(r, phi, z), ...] legalább 2 pont (az első a jelenlegi pozíció).
        Visszatér: (mintak, hiba) ahol mintak = [(j1, j2, j3), ...] DT időközönként.
        Ha valamelyik pálya pont nem elérhető, mintak = None és hiba a szöveges ok.
        """
        v_fogo = max_fogo_seb if max_fogo_seb else self.MAX_FOGO_SEB
        geo = self._geometriai_pontok(pontok)

        lepesek = []
        for p in geo:
            s = self.h.henger_to_lepes(*p, egesz=False)
            if s is None:
                return None, f"A pálya egy pontja nem elérhető: r={p[0]:.1f} phi={p[1]:.1f} z={p[2]:.1f}"
            lepesek.append(tuple(float(x) for x in s))
        n = len(lepesek)
        if n < 2:
            return [tuple(int(x) for x in lepesek[0])], None

        # pályahossz a q-térben
        qg = [self._q(p) for p in geo]
        ds = [max(self._hossz(self._kul(qg[k + 1], qg[k])), 1e-9) for k in range(n - 1)]

        vmax = self.BIZTONSAG * self.h.MAX_LEPES_SEB
        amax_fel = 0.5 * self.BIZTONSAG * self.h.MAX_LEPES_GYORS  # fele a görbületre, fele a gyorsításra

        # dq/ds és d²q/ds² minden mintapontban
        d1 = []; d2 = []
        for k in range(n):
            a = max(k - 1, 0); b = min(k + 1, n - 1)
            hossz = sum(ds[a:b])
            d1.append([(lepesek[b][i] - lepesek[a][i]) / hossz for i in range(3)])
            if 0 < k < n - 1:
                h1, h2 = ds[k - 1], ds[k]
                d2.append([2 * ((lepesek[k + 1][i] - lepesek[k][i]) / h2 -
                                (lepesek[k][i] - lepesek[k - 1][i]) / h1) / (h1 + h2) for i in range(3)])
            else:
                d2.append([0.0, 0.0, 0.0])

        # sebességkorlát pontonként (pálya-sebesség, "q-mm"/s)
        vlim = []
        for k in range(n):
            v = v_fogo
            for i in range(3):
                if abs(d1[k][i]) > 1e-12:
                    v = min(v, vmax / abs(d1[k][i]))
                if abs(d2[k][i]) > 1e-12:
                    v = min(v, math.sqrt(amax_fel / abs(d2[k][i])))
            vlim.append(v)
        vlim[0] = 0.0; vlim[-1] = 0.0

        # megengedett pályagyorsulás pontonként
        alim = []
        for k in range(n):
            m = max(abs(x) for x in d1[k])
            alim.append(amax_fel / m if m > 1e-12 else 1e9)

        # előre- és hátrafelé menet: v² <= v_elozo² + 2·a·ds
        v = vlim[:]
        for k in range(1, n):
            v[k] = min(v[k], math.sqrt(v[k - 1] ** 2 + 2 * alim[k - 1] * ds[k - 1]))
        for k in range(n - 2, -1, -1):
            v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2 * alim[k + 1] * ds[k]))

        # időbélyegek
        t = [0.0]
        for k in range(n - 1):
            vk = v[k] + v[k + 1]
            if vk <= 1e-9:
                vk = 2 * max(v[k], v[k + 1], 1e-3)
            t.append(t[-1] + 2 * ds[k] / vk)

        # újramintavételezés DT időközönként
        mintak = []
        idx = 0
        ido = 0.0
        while ido < t[-1]:
            while idx < n - 2 and t[idx + 1] < ido:
                idx += 1
            dt = t[idx + 1] - t[idx]
            u = (ido - t[idx]) / dt if dt > 0 else 0.0
            mintak.append(tuple(int(round(lepesek[idx][i] + (lepesek[idx + 1][i] - lepesek[idx][i]) * u))
                                for i in range(3)))
            ido += self.DT
        mintak.append(tuple(int(round(x)) for x in lepesek[-1]))
        return mintak, None


class PalyaEteto:
    """
    A megtervezett mintákat háttérszálon, DT időközönként küldi ki QMOVE paranccsal.
    Minden parancs tartalmazza a három léptetőmotor sebességét is, így a motorok együtt haladnak.
    A célpont a pályán előre fut (legfeljebb ELORETEKINTES_S-ig, de irányváltásnál megáll),
    hogy a motor ne kezdjen fékezni a minták között.
    """

    ELORETEKINTES_S = 0.5   # >= max_seb / (2 · gyorsulás) = 12000 / 30000 = 0.4 s

    def __init__(self, soros, dt):
        self.soros = soros
        self.dt = dt
        self._stop = threading.Event()
        self._szal = None
        self._fut = False
        self.lefoglalva = False    # az automatizáció a teljes folyamat idejére lefoglalja (fázisok között is)
        self._lock = threading.Lock()  # a küldés és a leállítás ne keveredhessen össze a soros porton
        self.utolso_minta = None   # az utoljára ténylegesen elküldött pályapont (j1, j2, j3)

    def foglalt(self):
        return self._fut or self.lefoglalva or (self._szal is not None and self._szal.is_alive())

    def uj_mozgas(self):
        """Egy új, felhasználó által indított mozgás (vagy automatizáció) előtt törli a leállítást."""
        self._stop.clear()

    def megallitva(self):
        return self._stop.is_set()

    def kuld(self, parancs):
        """Parancs küldése, de csak ha nincs leállítás. A zár miatt QSTOP után már semmi nem mehet ki."""
        with self._lock:
            if self._stop.is_set():
                return False
            self.soros.parancs_kuldes(parancs)
            return True

    def leallit(self):
        """Vészleállítás: tiltja a további küldést, és kiküldi a QSTOP-ot (a zár alatt, így utána QMOVE már nem mehet)."""
        with self._lock:
            self._stop.set()
            self.soros.parancs_kuldes("QSTOP")

    def futtat(self, mintak, j4, j5):
        """Blokkoló: végigküldi a pályát, és megvárja, míg a motorok odaérnek. True = végigment."""
        if self._stop.is_set():
            return False
        self._fut = True
        try:
            return self._futtat(mintak, j4, j5)
        finally:
            self._fut = False

    def _futtat(self, mintak, j4, j5):
        L = max(1, int(math.ceil(self.ELORETEKINTES_S / self.dt)))
        kezdet = time.perf_counter()
        n = len(mintak)
        for k in range(n):
            if self._stop.is_set():
                return False
            cel = self._elore_cel(mintak, k, L)
            kov = mintak[min(k + 1, n - 1)]
            seb = [max(10, int(round(abs(kov[i] - mintak[k][i]) / self.dt))) for i in range(3)]
            if not self.kuld(f"QMOVE {cel[0]} {cel[1]} {cel[2]} {j4} {j5} {seb[0]} {seb[1]} {seb[2]}"):
                return False
            self.utolso_minta = mintak[k]
            varakozas = kezdet + (k + 1) * self.dt - time.perf_counter()
            if varakozas > 0 and self._stop.wait(varakozas):
                return False
        # utolsó parancs: pontos végpont
        vege = mintak[-1]
        if not self.kuld(f"QMOVE {vege[0]} {vege[1]} {vege[2]} {j4} {j5} 2000 2000 2000"):
            return False
        self.utolso_minta = vege
        # rövid ráhagyás, hogy a motorok biztosan beérjenek
        if self._stop.wait(0.3):
            return False
        return True

    def inditas_hatterben(self, mintak, j4, j5, kesz_callback=None):
        def fut():
            ok = self.futtat(mintak, j4, j5)
            if kesz_callback:
                kesz_callback(ok)
        self._szal = threading.Thread(target=fut, daemon=True)
        self._szal.start()

    @staticmethod
    def _elore_cel(mintak, k, L):
        """A k. mintától legfeljebb L mintát előre néz, de minden motornál megáll az irányváltásnál."""
        n = len(mintak)
        cel = list(mintak[min(k + 1, n - 1)])
        for i in range(3):
            irany = 0
            for m in range(k + 1, min(k + L, n - 1) + 1):
                lepes = mintak[m][i] - mintak[m - 1][i]
                if lepes != 0:
                    uj = 1 if lepes > 0 else -1
                    if irany == 0:
                        irany = uj
                    elif uj != irany:
                        break
                cel[i] = mintak[m][i]
        return cel