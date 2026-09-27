"""
Malom próba: ember a gép ellen, a terminálban.
Egyben minta arra, hogyan hívható az OpenSpiel a saját programunkból.

A pontok számozása (OpenSpiel):
 0 ---------- 1 ---------- 2
 |    3 ----- 4 ----- 5    |
 |    |   6 - 7 - 8   |    |
 9 -- 10 - 11      12 - 13 - 14
 |    |  15 - 16 - 17 |    |
 |    18 ---- 19 ---- 20   |
21 ---------- 22 --------- 23
"""
import numpy as np
import pyspiel
from open_spiel.python.algorithms import mcts

GEP_EROSSEGE = 1000      # MCTS szimulációk száma lépésenként (kisebb = gyengébb, gyorsabb)
GEP_KEZD = False         # True: a gép a fehér (W) és ő kezd


# --- Nagyobb, számozott tábla kirajzolása ---
# A pontok helye egy 7x7-es rácson (oszlop, sor), az OpenSpiel sorszámozása szerint
PONT_HELYEK = [(0, 0), (3, 0), (6, 0), (1, 1), (3, 1), (5, 1), (2, 2), (3, 2), (4, 2),
               (0, 3), (1, 3), (2, 3), (4, 3), (5, 3), (6, 3), (2, 4), (3, 4), (4, 4),
               (1, 5), (3, 5), (5, 5), (0, 6), (3, 6), (6, 6)]
VONALAK = [(0, 1), (1, 2), (3, 4), (4, 5), (6, 7), (7, 8), (9, 10), (10, 11), (12, 13), (13, 14),
           (15, 16), (16, 17), (18, 19), (19, 20), (21, 22), (22, 23),
           (0, 9), (9, 21), (3, 10), (10, 18), (6, 11), (11, 15), (1, 4), (4, 7),
           (16, 19), (19, 22), (8, 12), (12, 17), (5, 13), (13, 20), (2, 14), (14, 23)]
CELLA_SZ, CELLA_M = 8, 3   # egy rácslépés szélessége és magassága karakterben


def tabla_allapot(allapot):
    """A 24 pont tartalma ('W', 'B' vagy '.') az OpenSpiel állapot szöveges alakjából."""
    tabla_sorok = str(allapot).split("\n\n")[0]
    pontok = [c for c in tabla_sorok if c in "WB."]   # a pontok soronként, balról jobbra = 0..23
    return pontok[:24]


def tabla_rajzolasa(allapot):
    pontok = tabla_allapot(allapot)
    szel, mag = 6 * CELLA_SZ + 5, 6 * CELLA_M + 1
    vaszon = [[" "] * szel for _ in range(mag)]
    hely = lambda i: (PONT_HELYEK[i][0] * CELLA_SZ + 2, PONT_HELYEK[i][1] * CELLA_M)
    for a, b in VONALAK:
        (xa, ya), (xb, yb) = hely(a), hely(b)
        if ya == yb:
            for x in range(min(xa, xb), max(xa, xb) + 1):
                vaszon[ya][x] = "-"
        else:
            for y in range(min(ya, yb), max(ya, yb) + 1):
                vaszon[y][xa] = "|"
    for i, tartalom in enumerate(pontok):
        x, y = hely(i)
        felirat = f"{i:^3}" if tartalom == "." else f"[{tartalom}]"
        for k, c in enumerate(felirat):
            vaszon[y][x - 1 + k] = c
    print("\n".join("".join(sor).rstrip() for sor in vaszon))

    # állapotsorok magyarul
    szoveg = str(allapot)
    for sor in szoveg.splitlines():
        if sor.startswith("Men to deploy:"):
            w, b = sor.split(":")[1].split()
            print(f"Még lerakható: W {w}, B {b}")
        elif sor.startswith("Num men:"):
            w, b = sor.split(":")[1].split()
            print(f"Bábuk száma:   W {w}, B {b}")
    if "Capture time" in szoveg:
        print("Malom! Most egy ellenfél-bábut kell levenni.")
    print()


def gep_letrehozasa(jatek):
    rng = np.random.RandomState()
    kiertekelo = mcts.RandomRolloutEvaluator(n_rollouts=1, random_state=rng)
    return mcts.MCTSBot(jatek, uct_c=2, max_simulations=GEP_EROSSEGE,
                        evaluator=kiertekelo, random_state=rng)


def lepes_szovegge(allapot, lepes):
    """Az OpenSpiel lépéskódja olvasható formában, pl. 'Point 5' vagy 'Move 5 -> 4'."""
    return allapot.action_to_string(allapot.current_player(), lepes)


def ember_lepese(allapot):
    """Bekér egy szabályos lépést. Megadható a pont száma, vagy lépésnél 'honnan hova' (pl. '5 4')."""
    szabalyos = allapot.legal_actions()
    while True:
        bemenet = input("Lépésed (üres Enter = lehetséges lépések): ").strip()
        if not bemenet:
            print(", ".join(lepes_szovegge(allapot, l) for l in szabalyos))
            continue
        try:
            reszek = [int(x) for x in bemenet.replace("->", " ").split()]
        except ValueError:
            print("Számot adj meg (pl. '5' vagy '5 4')!")
            continue
        # egy szám: lerakás / levétel; két szám: tologatás honnan -> hova (kód: 24 + honnan*24 + hova)
        lepes = reszek[0] if len(reszek) == 1 else 24 + reszek[0] * 24 + reszek[1]
        if lepes in szabalyos:
            return lepes
        print("Ez nem szabályos lépés.")


def main():
    jatek = pyspiel.load_game("nine_mens_morris")
    gep = gep_letrehozasa(jatek)
    gep_jatekos = 0 if GEP_KEZD else 1          # 0 = fehér (W), 1 = fekete (B)
    allapot = jatek.new_initial_state()

    print(f"Te vagy a {'fekete (B)' if GEP_KEZD else 'fehér (W)'}.")
    while not allapot.is_terminal():
        tabla_rajzolasa(allapot)
        if allapot.current_player() == gep_jatekos:
            lepes = gep.step(allapot)
            print(f">>> Gép lépése: {lepes_szovegge(allapot, lepes)}\n")
        else:
            lepes = ember_lepese(allapot)
        allapot.apply_action(lepes)

    tabla_rajzolasa(allapot)
    eredmeny = allapot.returns()[gep_jatekos]
    print("A gép nyert." if eredmeny > 0 else "Te nyertél!" if eredmeny < 0 else "Döntetlen.")


if __name__ == "__main__":
    main()