import math

class InverzKinematika:
    def __init__(self):
        # Karok fizikai hossza milliméterben
        self.L1 = 118.0
        self.L2 = 118.0  # MÓDOSÍTVA: 131.5-ről 118.0 mm-re
        
        # Fogási pont függőleges eltolása a J4 tengely alatt (fixen lefelé)
        self.Z_OFFSET = 67.0  # ÚJ: 67.0 mm függőleges offszet
        
        # Átváltási arány: 48000 lépés = 360 fok -> 1 fok = 133.333 lépés
        self.STEPS_PER_DEGREE = 48000 / 360.0

    def fok_to_lepes(self, fok):
        return int(fok * self.STEPS_PER_DEGREE)

    def lepes_to_fok(self, lepes):
        return float(lepes / self.STEPS_PER_DEGREE)

    def koordinata_szamitas(self, x, y, z):
        """
        [INVERZ KINEMATIKA] XYZ mm (Fogáspont) -> Motor lépések
        A megadott Z koordinátát korrigáljuk a J4 csuklótengely magasságára!
        """
        # A megadott Z a fogáspont. A J4 csuklóízület ennél fixen 67 mm-rel magasabban van.
        z_csuklo = z + self.Z_OFFSET

        if x == 0 and y == 0:
            theta_1_deg = 0.0
        else:
            theta_1_deg = math.degrees(math.atan2(y, x))

        r = math.sqrt(x**2 + y**2)
        d = math.sqrt(r**2 + z_csuklo**2)  # A csuklópozíció távolsága a bázistól

        # ÚJ Munkatér ellenőrzése az új L2 és z_csuklo alapján
        if d > (self.L1 + self.L2) or d < abs(self.L1 - self.L2) or d == 0:
            return None

        try:
            beta = math.acos((self.L1**2 + self.L2**2 - d**2) / (2.0 * self.L1 * self.L2))
            alpha_2 = math.acos((self.L1**2 + d**2 - self.L2**2) / (2.0 * self.L1 * d))
            alpha_1 = math.atan2(z_csuklo, r)

            beta_deg = math.degrees(beta)
            alpha_1_deg = math.degrees(alpha_1)
            alpha_2_deg = math.degrees(alpha_2)

            gamma_1 = alpha_1_deg + alpha_2_deg
            gamma_2 = gamma_1 + beta_deg - 180.0

            delta_j1 = theta_1_deg
            delta_j2 = gamma_1 - 90.0
            delta_j3 = gamma_2 - 0.0

            j1_steps = self.fok_to_lepes(delta_j1)
            j2_steps = self.fok_to_lepes(delta_j2)
            j3_steps = self.fok_to_lepes(delta_j3)

            return j1_steps, j2_steps, j3_steps

        except ValueError:
            return None
    def direkt_kinematika(self, j1_steps, j2_steps, j3_steps):
        """
        [DIREKT KINEMATIKA] Motor lépések -> XYZ mm (Fogáspont)
        A J4 csuklóízületből számolt Z koordinátából levonjuk a fogáspont offszetét!
        """
        delta_j1 = self.lepes_to_fok(j1_steps)
        delta_j2 = self.lepes_to_fok(j2_steps)
        delta_j3 = self.lepes_to_fok(j3_steps)

        theta_1 = math.radians(delta_j1)
        gamma_1 = math.radians(90.0 + delta_j2)
        gamma_2 = math.radians(0.0 + delta_j3)

        r_elbow = self.L1 * math.cos(gamma_1)
        z_elbow = self.L1 * math.sin(gamma_1)

        r_wrist = self.L2 * math.cos(gamma_2)
        z_wrist = self.L2 * math.sin(gamma_2)

        r_total = r_elbow + r_wrist
        z_csuklo = z_elbow + z_wrist

        # A fizikai fogáspont Z koordinátája mindig 67 mm-rel a csukló alatt van
        z_fogaspont = z_csuklo - self.Z_OFFSET

        x = r_total * math.cos(theta_1)
        y = r_total * math.sin(theta_1)
        z = z_fogaspont

        return round(x, 1), round(y, 1), round(z, 1)

    def kor_palya_generalas(self, cx=143.0, cy=45.0, cz=-60.0, atmero=60.0, felbontas=36, ismetles=10):
        sugar = atmero / 2.0
        lepes_pontok = []
        for kor_idx in range(ismetles):
            for i in range(felbontas):
                szog = (i / felbontas) * 2.0 * math.pi
                x = cx + sugar * math.cos(szog)
                y = cy + sugar * math.sin(szog)
                z = cz
                eredmeny = self.koordinata_szamitas(x, y, z)
                if eredmeny is not None:
                    lepes_pontok.append(eredmeny)
        return lepes_pontok
