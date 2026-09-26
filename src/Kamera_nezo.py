import cv2
import numpy as np

# =========================================================================
# BEÁLLÍTÁSOK
# =========================================================================
KAMERA_INDEX = 0        # ha a gépnek van beépített kamerája, az USB-s valószínűleg az 1-es
KEP_SZELESSEG = 640
KEP_MAGASSAG = 480
ABLAK_NEV = "Webkamera kép"
MASZK_ABLAK_NEV = "Zoldseg terkep (hangolashoz)"

# --- "Legzöldebb" folt keresése ---
# Zöldség-mérték pixelenként: G - max(R, B)  (0 = nem zöld, minél nagyobb, annál zöldebb)
MIN_ZOLDSEG = 10         # ha a legzöldebb pont is ennél kevésbé zöld, nincs mit bekeretezni (a korong ~20)
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


def zoldseg_terkep(frame):
    """Pixelenkénti zöldség: G - max(R, B), 0-255 között. A kép szürke-fehér-fekete részei ~0."""
    b, g, r = cv2.split(frame.astype(np.int16))
    zold = np.clip(g - np.maximum(r, b), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(zold, (9, 9), 0)   # zajszűrés, hogy ne egy-egy zajos pixel nyerjen


def legzoldebb_folt(terkep):
    """
    Megkeresi a legzöldebb pontot, és az azt körülvevő, összefüggő zöld foltot.
    Visszatér: ((cx, cy, fel_oldal), csucs_ertek, maszk) vagy (None, csucs_ertek, maszk)
    """
    _, csucs, _, csucs_hely = cv2.minMaxLoc(terkep)
    if csucs < MIN_ZOLDSEG:
        return None, csucs, np.zeros_like(terkep)

    maszk = (terkep >= csucs * RELATIV_KUSZOB).astype(np.uint8) * 255
    maszk = cv2.morphologyEx(maszk, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    _, cimkek, stat, kozeppontok = cv2.connectedComponentsWithStats(maszk)
    cimke = cimkek[csucs_hely[1], csucs_hely[0]]   # az a folt, amelyikben a legzöldebb pont van
    if cimke == 0 or stat[cimke, cv2.CC_STAT_AREA] < MIN_TERULET:
        return None, csucs, maszk

    cx, cy = kozeppontok[cimke]
    fel = max(stat[cimke, cv2.CC_STAT_WIDTH], stat[cimke, cv2.CC_STAT_HEIGHT]) / 2.0
    return (cx, cy, fel), csucs, maszk


def folt_rajzolasa(frame, folt, csucs):
    if folt is None:
        cv2.putText(frame, f"Nincs zold (max: {int(csucs)})", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return
    cx, cy, fel = folt
    x, y, f = int(round(cx)), int(round(cy)), int(round(fel)) + 4
    cv2.rectangle(frame, (x - f, y - f), (x + f, y + f), (0, 255, 0), 2)
    cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
    cv2.putText(frame, "LEGZOLDEBB", (x - f, y - f - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f"X:{x} Y:{y}", (x - f, y + f + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    cv2.putText(frame, f"Zoldseg: {int(csucs)}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    cap = kamera_inditas(KAMERA_INDEX)
    if cap is None:
        print(f"[HIBA] A(z) {KAMERA_INDEX}. kamera nem nyitható meg. "
              f"Próbáld a KAMERA_INDEX-et 1-re vagy 2-re állítani!")
        return

    szel = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    mag = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[OK] Kamera elindult ({szel}x{mag}). Kilépés: 'q' / ESC, vagy az ablak bezárása.")
    print("     'm': a zöldség-térkép ablak be-/kikapcsolása (hangoláshoz)")

    cv2.namedWindow(ABLAK_NEV, cv2.WINDOW_AUTOSIZE)
    maszk_latszik = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[HIBA] Nem sikerült képet olvasni a kamerából!")
                break

            terkep = zoldseg_terkep(frame)
            folt, csucs, maszk = legzoldebb_folt(terkep)
            folt_rajzolasa(frame, folt, csucs)

            cv2.imshow(ABLAK_NEV, frame)
            if maszk_latszik:
                # bal oldalt a zöldség-térkép (felerősítve), jobb oldalt a kiválasztott folt
                erositett = cv2.normalize(terkep, None, 0, 255, cv2.NORM_MINMAX)
                cv2.imshow(MASZK_ABLAK_NEV, np.hstack([erositett, maszk]))

            billentyu = cv2.waitKey(1) & 0xFF
            if billentyu in (ord('q'), 27):   # q vagy ESC
                break
            if billentyu == ord('m'):
                maszk_latszik = not maszk_latszik
                if not maszk_latszik:
                    cv2.destroyWindow(MASZK_ABLAK_NEV)
            # az ablak X gombjával való bezárás figyelése
            if cv2.getWindowProperty(ABLAK_NEV, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[OK] Kamera leállítva.")


if __name__ == "__main__":
    main()