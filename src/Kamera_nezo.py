import cv2
import numpy as np
import math
import sys
import threading
import queue

import malom_tabla   # a tábla- és tartalékhelyek táblakoordinátái (ugyanaz, amit a robotkar használ)

# =========================================================================
# BEÁLLÍTÁSOK
# =========================================================================
KAMERA_INDEX = 0        # ha a gépnek van beépített kamerája, az USB-s valószínűleg az 1-es
KEP_SZELESSEG = 640
KEP_MAGASSAG = 480
ABLAK_NEV = "Webkamera kep"   # ékezet nélkül: az OpenCV ablakcíme nem kezeli az ékezeteket
# A kép elforgatása közvetlenül beolvasás után (a felismerés is az elforgatott képen fut).
# 90 fokkal (óramutatóval ellentétesen): a 640 pixeles oldal lesz függőleges. Ha fejjel lefelé áll a kép,
# cseréld cv2.ROTATE_90_CLOCKWISE-ra; forgatás nélkül: None
KEP_FORGATAS = cv2.ROTATE_90_COUNTERCLOCKWISE
MASZK_ABLAK_NEV = "Szin terkepek (fent: piros, lent: zold)"
PONT_ABLAK_NEV = "Malom pontok (bal: sotet maszk, jobb: vonalak nelkul)"

# --- Malomtábla: a 24 fekete pont felismerése ---
# A pontokat vonalak kötik össze, ezért a sötét maszkot először "kinyitjuk" (morfológiai nyitás)
# egy, a keresett körnél jóval kisebb, de a vonalaknál szélesebb korongal: ettől a vonalak eltűnnek,
# a pontok megmaradnak. Utána a megmaradt foltok közül azok a pontok, amelyek
#   - kör alakúak (körszerűség és kitöltés alapján), és
#   - a sugaruk a csúszkán beállított érték ±25%-án belül van.
MALOM_PONTOK_SZAMA = 24
ADAPTIV_BLOKK = 61       # px, páratlan; a helyi sötétség-küszöb ablaka (egyenetlen megvilágítás ellen)
ADAPTIV_C = 10           # ennyivel sötétebbnek kell lennie a környezeténél, hogy "fekete" legyen
ALAP_SUGAR_PX = 8        # a csúszka induló értéke: a keresett kör sugara pixelben (bemérve)
MAX_SUGAR_PX = 40        # a csúszka felső határa
MERET_TURES = 0.25       # ±25% a beállított sugárhoz képest
NYITAS_ARANY = 0.7       # a vonalakat eltüntető korong sugara a keresett sugár ennyiszerese
MIN_KORSZERUSEG = 0.75   # 4·π·terület / kerület²  (tökéletes kör = 1.0, négyzet ≈ 0.785)
MIN_KITOLTES = 0.70      # folt területe / köré írt kör területe (kör ≈ 0.9-1.0, négyzet ≈ 0.64)
CSUSZKA_NEV = "Kor sugara px"

SZINKERESES = False      # a piros/zöld keresés ('s' billentyűvel kapcsolható)

# --- Kamera kalibráció (pixel <-> robot) ---
KAL_KEPKOCKA = 10        # ennyi olyan képkocka átlagából számol, amelyen mind a 24 pont megvan
KAL_MAX_KEPKOCKA = 90    # legfeljebb ennyi képkockát próbál (kb. 3 s), utána hibát jelez

# --- A keresett színek ---
# Mérték pixelenként: a saját csatorna mínusz a másik kettő közül a nagyobb
#   pirosság = R - max(G, B),   zöldség = G - max(R, B)   (0 = egyáltalán nem olyan színű)
# Minden színnél: (név a képen, BGR csatorna indexe, küszöb, keret színe BGR-ben)
SZINEK = [
    ("PIROS", 2, 20, (0, 0, 255)),   # küszöb: ha a legpirosabb pont is ennél gyengébb -> nincs piros
    ("ZOLD",  1, 10, (0, 255, 0)),   # a zöld korong ~20 körüli értéket ad
]
RELATIV_KUSZOB = 0.5     # a folthoz tartozik minden pixel, ami legalább a csúcs 50%-a
MIN_TERULET = 100        # pixel², ennél kisebb foltot nem jelzünk
# =========================================================================


def kamera_inditas(index):
    # Ugyanúgy nyitjuk, mint a korábbi, jól működő ImageProcessor: DirectShow backenddel (Windows)
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, KEP_SZELESSEG)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, KEP_MAGASSAG)
    return cap


def szin_terkep(frame, csatorna):
    """Pixelenként: a megadott csatorna mínusz a másik kettő maximuma, 0-255 között."""
    csat = cv2.split(frame.astype(np.int16))
    tobbi = [csat[i] for i in range(3) if i != csatorna]
    terkep = np.clip(csat[csatorna] - np.maximum(tobbi[0], tobbi[1]), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(terkep, (9, 9), 0)   # zajszűrés, hogy ne egy-egy zajos pixel nyerjen


def legerosebb_folt(terkep, min_ertek):
    """
    Megkeresi a térkép legerősebb pontját, és az azt körülvevő összefüggő foltot.
    Visszatér: ((cx, cy, fel_oldal), csucs_ertek, maszk) vagy (None, csucs_ertek, maszk)
    """
    _, csucs, _, csucs_hely = cv2.minMaxLoc(terkep)
    if csucs < min_ertek:
        return None, csucs, np.zeros_like(terkep)

    maszk = (terkep >= csucs * RELATIV_KUSZOB).astype(np.uint8) * 255
    maszk = cv2.morphologyEx(maszk, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    _, cimkek, stat, kozeppontok = cv2.connectedComponentsWithStats(maszk)
    cimke = cimkek[csucs_hely[1], csucs_hely[0]]   # az a folt, amelyikben a legerősebb pont van
    if cimke == 0 or stat[cimke, cv2.CC_STAT_AREA] < MIN_TERULET:
        return None, csucs, maszk

    cx, cy = kozeppontok[cimke]
    fel = max(stat[cimke, cv2.CC_STAT_WIDTH], stat[cimke, cv2.CC_STAT_HEIGHT]) / 2.0
    return (cx, cy, fel), csucs, maszk


def folt_rajzolasa(frame, folt, csucs, nev, keret_szin, sor):
    """Bekeretezi a foltot, és a bal felső sarokba (a megadott sorba) kiírja az értéket."""
    szoveg_y = 22 + sor * 24
    if folt is None:
        cv2.putText(frame, f"{nev}: nincs (max: {int(csucs)})", (10, szoveg_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, keret_szin, 2)
        return
    cx, cy, fel = folt
    x, y, f = int(round(cx)), int(round(cy)), int(round(fel)) + 4
    cv2.rectangle(frame, (x - f, y - f), (x + f, y + f), keret_szin, 2)
    cv2.circle(frame, (x, y), 3, (255, 255, 255), -1)
    cv2.putText(frame, f"LEG-{nev}", (x - f, y - f - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, keret_szin, 2)
    cv2.putText(frame, f"X:{x} Y:{y}", (x - f, y + f + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, keret_szin, 1)
    cv2.putText(frame, f"{nev}: {int(csucs)}", (10, szoveg_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, keret_szin, 2)


def sotet_maszk(frame):
    """Fehér ott, ahol a kép a környezeténél sötétebb (fekete pontok és vonalak)."""
    szurke = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    maszk = cv2.adaptiveThreshold(szurke, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY_INV, ADAPTIV_BLOKK, ADAPTIV_C)
    return cv2.morphologyEx(maszk, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # apró zaj ki


def malom_pontok_keresese(frame, sugar):
    """
    sugar: a keresett kör sugara pixelben (csúszka).
    Visszatér: (pontok, maszk, nyitott_maszk, mert_sugar), ahol pontok = [(cx, cy, r), ...],
    mert_sugar a kör alakú foltok medián sugara (méret-szűrés nélkül) - segít beállítani a csúszkát.
    """
    maszk = sotet_maszk(frame)

    # 1) Vonalak eltüntetése: a keresett körnél kisebb, a vonalnál nagyobb korongal nyitunk
    k = max(3, 2 * int(round(NYITAS_ARANY * sugar)) + 1)
    nyitott = cv2.morphologyEx(maszk, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

    # 2) Kör alakú foltok, a megfelelő mérettel
    kontúrok, _ = cv2.findContours(nyitott, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    min_r, max_r = sugar * (1 - MERET_TURES), sugar * (1 + MERET_TURES)
    korok, pontok = [], []
    for cnt in kontúrok:
        terulet = cv2.contourArea(cnt)
        kerulet = cv2.arcLength(cnt, True)
        if terulet <= 0 or kerulet <= 0:
            continue
        (ex, ey), r = cv2.minEnclosingCircle(cnt)
        if (4 * math.pi * terulet) / (kerulet * kerulet) < MIN_KORSZERUSEG:
            continue
        if r <= 0 or terulet / (math.pi * r * r) < MIN_KITOLTES:
            continue
        # a folt "egyenértékű" sugara (azonos területű kör): kevésbé érzékeny a széli egyenetlenségre
        r_ekv = math.sqrt(terulet / math.pi)
        korok.append(r_ekv)
        if not (min_r <= r_ekv <= max_r):
            continue
        M = cv2.moments(cnt)
        pontok.append((M["m10"] / M["m00"], M["m01"] / M["m00"], r_ekv))

    # 3) Legfeljebb 24: a látómező közepéhez legközelebbiek (a tábla a kép közepén van,
    #    a szélén lévő zavaró foltok így kiesnek)
    kozep_x, kozep_y = frame.shape[1] / 2.0, frame.shape[0] / 2.0
    pontok.sort(key=lambda p: (p[0] - kozep_x) ** 2 + (p[1] - kozep_y) ** 2)
    pontok = pontok[:MALOM_PONTOK_SZAMA]
    mert_sugar = float(np.median(korok)) if korok else 0.0
    return pontok, maszk, nyitott, mert_sugar


# A malomtábla soraiban lévő pontok száma fentről lefelé (OpenSpiel sorszámozás: soronként, balról jobbra)
#  0  1  2 / 3  4  5 / 6  7  8 / 9 10 11 12 13 14 / 15 16 17 / 18 19 20 / 21 22 23
MALOM_SOROK = [3, 3, 3, 6, 3, 3, 3]


def malom_pontok_sorszamozasa(pontok):
    """
    A 24 pontot a játék (OpenSpiel) sorrendjébe rendezi: fentről lefelé soronként, soron belül
    balról jobbra. A listában az i. elem az i. sorszámú pont. Ha nem pontosan 24 pont van, None.
    """
    if len(pontok) != MALOM_PONTOK_SZAMA:
        return None
    y_szerint = sorted(pontok, key=lambda p: p[1])
    rendezett, kezd = [], 0
    for db in MALOM_SOROK:
        rendezett += sorted(y_szerint[kezd:kezd + db], key=lambda p: p[0])
        kezd += db
    return rendezett


def malom_pontok_rajzolasa(frame, pontok):
    for cx, cy, sugar in pontok:
        x, y, r = int(round(cx)), int(round(cy)), int(round(sugar)) + 3
        cv2.circle(frame, (x, y), r, (0, 200, 255), 2)
        cv2.circle(frame, (x, y), 2, (0, 0, 255), -1)
    sorszamozott = malom_pontok_sorszamozasa(pontok)
    if sorszamozott is not None:
        for i, (cx, cy, sugar) in enumerate(sorszamozott):
            x, y = int(round(cx)), int(round(cy))
            hely = (x + int(sugar) + 4, y - int(sugar) - 2)   # a pont jobb felső sarkához
            cv2.putText(frame, str(i), hely, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)       # körvonal
            cv2.putText(frame, str(i), hely, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    szin = (0, 255, 0) if len(pontok) == MALOM_PONTOK_SZAMA else (0, 0, 255)
    cv2.putText(frame, f"Malom pontok: {len(pontok)}/{MALOM_PONTOK_SZAMA}",
                (10, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7, szin, 2)


def tartalek_helyek_rajzolasa(frame, sorszamozott):
    """
    A 24 felismert pontból homográfiát számol (táblakoordináta -> kép), és ezzel berajzolja a
    tábla melletti 18 virtuális tartalék helyet (24-41) - ugyanott, ahová a robotkar teszi a bábukat.
    """
    if sorszamozott is None:
        return
    tabla = np.float32([malom_tabla.TABLA_KOORD[i] for i in range(MALOM_PONTOK_SZAMA)])
    kep = np.float32([(p[0], p[1]) for p in sorszamozott])
    H, _ = cv2.findHomography(tabla, kep)
    if H is None:
        return
    tartalek = malom_tabla.tartalek_koord()
    sorszamok = sorted(tartalek)
    vetitett = cv2.perspectiveTransform(np.float32([[tartalek[n] for n in sorszamok]]), H)[0]
    sugar = int(np.median([p[2] for p in sorszamozott])) + 3
    for n, (x, y) in zip(sorszamok, vetitett):
        x, y = int(round(x)), int(round(y))
        cv2.circle(frame, (x, y), sugar, (255, 160, 0), 2)
        cv2.putText(frame, str(n), (x - 9, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
        cv2.putText(frame, str(n), (x - 9, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)


def meret_kiirasa(frame, sugar, mert_sugar):
    """A csúszka értéke, az elfogadott tartomány és a képen mért körök medián sugara."""
    cv2.putText(frame, f"Keresett sugar: {sugar} px ({sugar * (1 - MERET_TURES):.1f}-{sugar * (1 + MERET_TURES):.1f})",
                (10, frame.shape[0] - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"Mert kor sugar (median): {mert_sugar:.1f} px",
                (10, frame.shape[0] - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


# =========================================================================
# KALIBRÁCIÓ: a 24 malompont pixelhelye <-> robotbeli helye (x, y mm a tábla síkjában)
# =========================================================================
def robot_xy_pontok():
    """A 42 pont (0-23 tábla, 24-41 tartalék) robotbeli derékszögű helye: {pont: (x, y)} mm."""
    eredmeny = {}
    for p, (r, phi, _z) in malom_tabla.malom_pontok_szamitasa().items():
        eredmeny[p] = (r * math.cos(math.radians(phi)), r * math.sin(math.radians(phi)))
    return eredmeny


def kalibracio_futtatasa(cap, sugar):
    """
    Több képkockán megkeresi a 24 pontot, átlagolja a pixelhelyüket, és kiszámolja a homográfiát
    (robot x-y -> pixel): ez együtt kezeli az eltolást, forgatást, nagyítást és a perspektívát.
    Visszatér: (H, H_inv, info_szoveg) vagy (None, None, hiba_szoveg).
    """
    gyujtes, utolso_db = [], 0
    for _ in range(KAL_MAX_KEPKOCKA):
        ret, frame = cap.read()
        if not ret:
            break
        if KEP_FORGATAS is not None:
            frame = cv2.rotate(frame, KEP_FORGATAS)
        pontok, _, _, _ = malom_pontok_keresese(frame, sugar)
        utolso_db = len(pontok)
        sorszamozott = malom_pontok_sorszamozasa(pontok)
        if sorszamozott is not None:
            gyujtes.append([(p[0], p[1]) for p in sorszamozott])
        # közben látszódjon, mit lát
        kep = frame.copy()
        malom_pontok_rajzolasa(kep, pontok)
        cv2.putText(kep, f"KALIBRALAS... {len(gyujtes)}/{KAL_KEPKOCKA}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow(ABLAK_NEV, kep)
        cv2.waitKey(1)
        if len(gyujtes) >= KAL_KEPKOCKA:
            break

    if len(gyujtes) < KAL_KEPKOCKA:
        return None, None, (f"nem sikerult mind a 24 pontot megtalalni (utoljara {utolso_db} pont). "
                            f"Ures legyen a tabla, es allitsd be a kor sugarat ('p' billentyu: elonezet)!")

    pixel = np.mean(np.array(gyujtes, dtype=np.float64), axis=0)          # 24 x 2, átlagolt pixelhelyek
    robot = robot_xy_pontok()
    robot_24 = np.array([robot[i] for i in range(MALOM_PONTOK_SZAMA)], dtype=np.float64)
    H, _ = cv2.findHomography(robot_24, pixel, 0)                         # legkisebb négyzetes illesztés
    if H is None:
        return None, None, "a homografia nem szamolhato"
    H_inv = np.linalg.inv(H)

    # illeszkedés: pixelben (robot -> kép) és mm-ben (kép -> robot)
    vetitett = cv2.perspectiveTransform(robot_24.reshape(1, -1, 2), H)[0]
    hiba_px = np.linalg.norm(vetitett - pixel, axis=1)
    vissza = cv2.perspectiveTransform(pixel.reshape(1, -1, 2), H_inv)[0]
    hiba_mm = np.linalg.norm(vissza - robot_24, axis=1)
    info = (f"atlag {hiba_px.mean():.1f} px / {hiba_mm.mean():.1f} mm, "
            f"max {hiba_px.max():.1f} px / {hiba_mm.max():.1f} mm (a {int(np.argmax(hiba_mm))}. pontnal)")
    return H, H_inv, info


def pixel_robotba(H_inv, x, y):
    """Pixel -> robot (x, y) mm, (r, phi) a tábla síkjában."""
    rx, ry = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float64), H_inv)[0][0]
    return rx, ry, math.hypot(rx, ry), math.degrees(math.atan2(ry, rx))


def robot_pixelbe(H, rx, ry):
    px, py = cv2.perspectiveTransform(np.array([[[rx, ry]]], dtype=np.float64), H)[0][0]
    return int(round(px)), int(round(py))


def stdin_olvaso(parancsok):
    """A robotkar programja a szabványos bemeneten küld parancsot (pl. 'KALIBRALAS')."""
    try:
        for sor in sys.stdin:
            parancsok.put(sor.strip())
    except (OSError, ValueError):
        pass


def ablak_bezaras(nev):
    """Bezár egy segédablakot, ha nyitva van (nem nyitott ablaknál sem dob hibát)."""
    try:
        if cv2.getWindowProperty(nev, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(nev)
    except cv2.error:
        pass


def main():
    cap = kamera_inditas(KAMERA_INDEX)
    if cap is None:
        print(f"[HIBA] A(z) {KAMERA_INDEX}. kamera nem nyitható meg. "
              f"Próbáld a KAMERA_INDEX-et 1-re vagy 2-re állítani!", flush=True)
        return

    szel = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    mag = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[OK] Kamera elindult ({szel}x{mag}). Kilépés: 'q' / ESC, vagy az ablak bezárása.", flush=True)
    print("     'k': kalibrálás (kar nélkül), 'p': pontkeresés előnézet, 'm': hangoló ablak, "
          "'s': piros/zöld keresés", flush=True)

    cv2.namedWindow(ABLAK_NEV, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar(CSUSZKA_NEV, ABLAK_NEV, ALAP_SUGAR_PX, MAX_SUGAR_PX, lambda _ertek: None)

    # kattintás figyelése (kalibráció után: pixel -> robot koordináta ellenőrzés)
    kattintas = {"pont": None}
    def eger(esemeny, x, y, _flags, _param):
        if esemeny == cv2.EVENT_LBUTTONDOWN:
            kattintas["pont"] = (x, y)
    cv2.setMouseCallback(ABLAK_NEV, eger)

    parancsok = queue.Queue()
    threading.Thread(target=stdin_olvaso, args=(parancsok,), daemon=True).start()

    maszk_latszik = False
    szinkereses = SZINKERESES
    elonezet = False                      # pontkeresés előnézete kalibráció előtt (a csúszka beállításához)
    H = H_inv = None                      # kalibráció: robot x-y <-> pixel
    kal_info = ""
    robot_pontok = {}
    kattintas_eredmeny = None             # (kattintott pixel, legközelebbi pont pixelhelye, szöveg)

    try:
        while True:
            # --- kalibrálási kérés (robotkar programjától vagy a 'k' billentyűtől) ---
            kalibralj = False
            while not parancsok.empty():
                if parancsok.get() == "KALIBRALAS":
                    kalibralj = True
            if kalibralj:
                sugar = max(2, cv2.getTrackbarPos(CSUSZKA_NEV, ABLAK_NEV))
                uj_H, uj_H_inv, szoveg = kalibracio_futtatasa(cap, sugar)
                if uj_H is None:
                    print(f"KALIBRACIO_HIBA {szoveg}", flush=True)
                else:
                    H, H_inv, kal_info = uj_H, uj_H_inv, szoveg
                    robot_pontok = robot_xy_pontok()
                    kattintas_eredmeny = None
                    elonezet = maszk_latszik = False
                    ablak_bezaras(PONT_ABLAK_NEV)
                    print(f"KALIBRACIO_OK {szoveg}", flush=True)

            ret, frame = cap.read()
            if not ret:
                print("[HIBA] Nem sikerült képet olvasni a kamerából!", flush=True)
                break
            if KEP_FORGATAS is not None:
                frame = cv2.rotate(frame, KEP_FORGATAS)
            debug_sorok = []
            eredeti = frame.copy()   # a keresés a rajzolás előtti képen fusson (a keretek ne zavarjanak)

            if H is None:
                # --- még nincs kalibráció: csak a kép (kérésre a pontkeresés előnézete) ---
                if elonezet:
                    sugar = max(2, cv2.getTrackbarPos(CSUSZKA_NEV, ABLAK_NEV))
                    pontok, sotet, magok, mert_sugar = malom_pontok_keresese(eredeti, sugar)
                    malom_pontok_rajzolasa(frame, pontok)
                    tartalek_helyek_rajzolasa(frame, malom_pontok_sorszamozasa(pontok))
                    meret_kiirasa(frame, sugar, mert_sugar)
                    if maszk_latszik:
                        cv2.imshow(PONT_ABLAK_NEV, cv2.resize(np.hstack([sotet, magok]), None, fx=0.5, fy=0.5))
                cv2.putText(frame, "NINCS KALIBRALVA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # --- kalibrálva: nincs pontkeresés; kattintásra pixel -> robot koordináta ---
                if kattintas["pont"] is not None:
                    kx, ky = kattintas["pont"]
                    kattintas["pont"] = None
                    rx, ry, r, phi = pixel_robotba(H_inv, kx, ky)
                    legk = min(robot_pontok, key=lambda p: (robot_pontok[p][0] - rx) ** 2 + (robot_pontok[p][1] - ry) ** 2)
                    tav = math.hypot(robot_pontok[legk][0] - rx, robot_pontok[legk][1] - ry)
                    szoveg = f"r={r:.1f} phi={phi:.1f} | legkozelebbi pont: {legk} ({tav:.1f} mm)"
                    kattintas_eredmeny = ((kx, ky), robot_pixelbe(H, *robot_pontok[legk]), szoveg)
                    print(f"[KATTINTAS] pixel=({kx},{ky}) -> x={rx:.1f} y={ry:.1f} {szoveg}", flush=True)
                if kattintas_eredmeny is not None:
                    (kx, ky), (lx, ly), szoveg = kattintas_eredmeny
                    cv2.drawMarker(frame, (kx, ky), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
                    cv2.circle(frame, (lx, ly), 10, (0, 255, 0), 2)    # ahol a transzformáció szerint a pont van
                    cv2.putText(frame, szoveg, (10, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 4)
                    cv2.putText(frame, szoveg, (10, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                cv2.putText(frame, "KALIBRALVA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, kal_info.split(",")[0], (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            for sor, (nev, csatorna, kuszob, keret_szin) in enumerate(SZINEK if szinkereses else []):
                terkep = szin_terkep(eredeti, csatorna)
                folt, csucs, maszk = legerosebb_folt(terkep, kuszob)
                folt_rajzolasa(frame, folt, csucs, nev, keret_szin, sor + 3)
                if maszk_latszik:
                    erositett = cv2.normalize(terkep, None, 0, 255, cv2.NORM_MINMAX)
                    debug_sorok.append(np.hstack([erositett, maszk]))

            cv2.imshow(ABLAK_NEV, frame)
            if maszk_latszik and debug_sorok:
                cv2.imshow(MASZK_ABLAK_NEV, cv2.resize(np.vstack(debug_sorok), None, fx=0.5, fy=0.5))

            billentyu = cv2.waitKey(1) & 0xFF
            if billentyu in (ord('q'), 27):   # q vagy ESC
                break
            if billentyu == ord('k'):
                parancsok.put("KALIBRALAS")
            if billentyu == ord('p'):
                elonezet = not elonezet
                if not elonezet:
                    ablak_bezaras(PONT_ABLAK_NEV)
            if billentyu == ord('m'):
                maszk_latszik = not maszk_latszik
                if not maszk_latszik:
                    ablak_bezaras(PONT_ABLAK_NEV)
                    ablak_bezaras(MASZK_ABLAK_NEV)
            if billentyu == ord('s'):
                szinkereses = not szinkereses
                if not szinkereses:
                    ablak_bezaras(MASZK_ABLAK_NEV)
            # az ablak X gombjával való bezárás figyelése
            if cv2.getWindowProperty(ABLAK_NEV, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[OK] Kamera leállítva.", flush=True)


if __name__ == "__main__":
    main()