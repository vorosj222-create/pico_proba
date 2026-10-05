import math

# =========================================================================
# BEMÉRT PONTOK (a kart odavezetve leolvasva), a malomjáték (OpenSpiel) számozása szerint
#   pont száma: (r mm, szög fok)
#
#  0 ---------- 1 ---------- 2
#  |    3 ----- 4 ----- 5    |
#  |    |   6 - 7 - 8   |    |
#  9 -- 10 - 11      12 - 13 - 14
#  |    |  15 - 16 - 17 |    |
#  |    18 ---- 19 ---- 20   |
# 21 ---------- 22 --------- 23
# =========================================================================
BEMERT_PONTOK = {
    # külső négyzet sarkai
    0: (105.0, -43.0),
    2: (228.0, -19.0),
    21: (99.0, 40.0),
    23: (226.0, 16.5),
    # legbelső négyzet sarkai
    6: (125.0, -12.0),
    8: (172.0, -9.0),
    15: (124.0, 9.0),
    17: (171.0, 6.0),
}
MALOM_Z = -58.0          # erre a magasságra ereszkedik le a kar a pont fölött (a tábla síkja kb. -68)
FELSO_Z = -35.0          # a pontok között ennél lejjebb sosem mehet a kar (bábuk mozgatásakor)
KANYAR_MM = 10.0         # a sarkok lekerekítése (mm); a kar ennyivel FELSO_Z fölött halad (-25),
                         # így a lekerekített kanyar sem viszi FELSO_Z alá, és nem áll meg a sarkokban
KALIBRACIOS_POZICIO = (118.0, 90.0, 51.0)   # kamera kalibrációkor ide áll a kar (r, phi, z), hogy ne takarja a táblát
CLAW_NYITVA = 96         # J5 fogó: nyitott állás bábu áthelyezésnél (fogás előtt / elengedés)
CLAW_ZARVA = 150         # J5 fogó: zárt állás (bábu megfogása)
BABU_Z = -55.0           # bábu áthelyezésnél erre a magasságra ereszkedik le (fogás és lerakás)
CLAW_VARAKOZAS = 0.4     # s, ennyit vár a fogó nyitása/zárása után

# Ideális táblakoordináták (7x7-es rács, középpont = 0): külső ±3, középső ±2, belső ±1
TABLA_KOORD = {
    0: (-3, -3), 1: (0, -3), 2: (3, -3),
    3: (-2, -2), 4: (0, -2), 5: (2, -2),
    6: (-1, -1), 7: (0, -1), 8: (1, -1),
    9: (-3, 0), 10: (-2, 0), 11: (-1, 0), 12: (1, 0), 13: (2, 0), 14: (3, 0),
    15: (-1, 1), 16: (0, 1), 17: (1, 1),
    18: (-2, 2), 19: (0, 2), 20: (2, 2),
    21: (-3, 3), 22: (0, 3), 23: (3, 3),
}

# --- Tartalék helyek: a még le nem rakott és a leütött bábuk virtuális helye a tábla mellett ---
# Felül (a 0-1-2 sor fölött) 9 hely: 24..32, alul (a 21-22-23 sor alatt) 9 hely: 33..41.
# Két sorban, cikcakban: a páros sorszámúak a táblához közelebbi sorban (5 db), a páratlanok a
# távolabbi sorban (4 db), vízszintesen egyenletesen, középre igazítva.
# Táblaegységben megadva (1 egység = a belső négyzet fél oldala, ~23 mm).
TARTALEK_SOROK = (4.4, 5.2)        # a két sor távolsága a tábla közepétől (a külső sor: 3.0)
TARTALEK_FEL_SZELESSEG = 2.5       # a sor fél szélessége (a külső négyzeté 3.0; a jobb szélső 3.0-nál már nem érhető el)
TARTALEK_DB = 9
FELSO_TARTALEK = list(range(24, 24 + TARTALEK_DB))     # 24..32
ALSO_TARTALEK = list(range(33, 33 + TARTALEK_DB))      # 33..41


def tartalek_koord():
    """A 18 tartalék hely táblakoordinátája: {sorszám: (u, v)}."""
    koord = {}
    for oldal, sorszamok in ((-1, FELSO_TARTALEK), (1, ALSO_TARTALEK)):
        for i, n in enumerate(sorszamok):
            u = -TARTALEK_FEL_SZELESSEG + i * (2 * TARTALEK_FEL_SZELESSEG / (TARTALEK_DB - 1))
            v = oldal * TARTALEK_SOROK[i % 2]
            koord[n] = (u, v)
    return koord


# A nem bemért pontok két szomszédjuk felezőpontjai (derékszögű x-y síkban számolva):
#  - külső és belső oldalfelezők: a két sarok között félúton
#  - középső négyzet pontjai: a külső és a belső megfelelő pontja között félúton
FELEZOK = [
    (1, 0, 2), (9, 0, 21), (14, 2, 23), (22, 21, 23),      # külső oldalfelezők
    (7, 6, 8), (11, 6, 15), (12, 8, 17), (16, 15, 17),     # belső oldalfelezők
    (3, 0, 6), (4, 1, 7), (5, 2, 8), (10, 9, 11),          # középső négyzet
    (13, 12, 14), (18, 21, 15), (19, 22, 16), (20, 23, 17),
]


def _xy(r, fok):
    return r * math.cos(math.radians(fok)), r * math.sin(math.radians(fok))


def _henger(x, y):
    return math.hypot(x, y), math.degrees(math.atan2(y, x))


def malom_pontok_szamitasa(bemert=BEMERT_PONTOK):
    """
    A 8 bemért pontból kiszámolja mind a 24 táblapont és a 18 tartalék hely helyét.
    Visszatér: {pont: (r, phi, z)}, pont = 0..23 (tábla) és 24..41 (tartalék).
    A bemért pontok pontosan a megadott helyükre kerülnek, a többi táblapontot felezéssel számolja
    (x-y síkban, mert a táblán az egyenesek egyenesek, r-phi-ben nem). A táblán kívüli tartalék
    helyekhez nincs mihez felezni, ezeket a 8 pontra illesztett affin leképezéssel számolja.
    """
    xy = {p: _xy(*rf) for p, rf in bemert.items()}
    for p, a, b in FELEZOK:   # a sorrend olyan, hogy a felhasznált pontok már ki vannak számolva
        xy[p] = ((xy[a][0] + xy[b][0]) / 2, (xy[a][1] + xy[b][1]) / 2)
    A = affin_illesztes(bemert)
    for p, (u, v) in tartalek_koord().items():
        xy[p] = tuple(u * A[0][k] + v * A[1][k] + A[2][k] for k in range(2))
    return {p: (*_henger(*xy[p]), MALOM_Z) for p in sorted(xy)}


def affin_illesztes(bemert=BEMERT_PONTOK):
    """Legkisebb négyzetes affin leképezés táblakoordinátáról robot x-y-ra: [u v 1] * A = [x y]."""
    sorok = [((*TABLA_KOORD[p], 1.0), _xy(*rf)) for p, rf in bemert.items()]
    ata = [[sum(s[0][i] * s[0][j] for s in sorok) for j in range(3)] for i in range(3)]
    atb = [[sum(s[0][i] * s[1][k] for s in sorok) for k in range(2)] for i in range(3)]
    return _megold(ata, atb)


def ellenorzes(bemert=BEMERT_PONTOK):
    """
    Megnézi, mennyire illeszkednek a bemért pontok egy szabályos malomtáblára (legkisebb négyzetes
    affin illesztés). Visszatér: {pont: eltérés mm-ben}. Egy nagy eltérés elírt vagy felcserélt pontot jelez.
    """
    sorok = [((*TABLA_KOORD[p], 1.0), _xy(*rf)) for p, rf in bemert.items()]
    megoldas = affin_illesztes(bemert)
    elteres = {}
    for p, (u, x) in zip(bemert, sorok):
        becsult = [sum(u[i] * megoldas[i][k] for i in range(3)) for k in range(2)]
        elteres[p] = math.hypot(becsult[0] - x[0], becsult[1] - x[1])
    return elteres


def _megold(a, b):
    """Kis lineáris egyenletrendszer (Gauss-elimináció), több jobb oldallal."""
    n = len(a)
    m = [a[i][:] + b[i][:] for i in range(n)]
    for i in range(n):
        piv = max(range(i, n), key=lambda r: abs(m[r][i]))
        m[i], m[piv] = m[piv], m[i]
        for r in range(n):
            if r != i:
                f = m[r][i] / m[i][i]
                m[r] = [x - f * y for x, y in zip(m[r], m[i])]
    return [[m[i][n + k] / m[i][i] for k in range(len(b[0]))] for i in range(n)]


if __name__ == "__main__":
    for p, (r, phi, z) in malom_pontok_szamitasa().items():
        print(f"{p:2d}: r={r:6.1f}  phi={phi:6.1f}  z={z}")
    print("\nIllesztési eltérés (mm):", {p: round(e, 1) for p, e in ellenorzes().items()})