import tkinter as tk
from tkinter import messagebox
import time
from soros_kezelo import SorosKezelo
from inverz_kinematika import InverzKinematika
from felhasznaloi_felulet import FelhasznaloiFelulet

class RobotkarAlkalmazas:
    def __init__(self):
        self.root = tk.Tk()
        
        # Belső regiszterek inicializálása a ti pozitív X-tengelyes koordinátátok szerint
        self.aktualis_x = 131.5
        self.aktualis_y = 0.0
        self.aktualis_z = 118.0
        self.homing_folyamatban = False
        
        # Almodulok példányosítása
        self.soros = SorosKezelo(log_callback=self.log_erkezett)
        self.kinematika = InverzKinematika()
        
        # GUI létrehozása és a gomb-események összekötése
        self.gui = FelhasznaloiFelulet(
            root=self.root,
            on_connect=self.esemeny_kapcsolodas,
            on_home=self.esemeny_homing,
            on_send=self.esemeny_koordinata_kuldes,
            on_jog=self.esemeny_leptetes,
            on_s_curve=self.esemeny_s_gorbe_inditas,
            on_send_xyz=self.esemeny_xyz_kuldes,
            on_jog_xyz=self.esemeny_xyz_leptetes,
            on_qstop=self.esemeny_qstop
        )
        
        self._feluleti_ertekek_alaphelyzetbe()
        self.gui.portok_frissitese(self.soros.aktiv_portok_lekerese())

    def log_erkezett(self, szoveg):
        self.gui.log_kiiras(szoveg)
        if "STATUSZ" in szoveg or "KESZ" in szoveg or "ALAPHELYZETBEN" in szoveg:
            self.homing_folyamatban = False

    def _feluleti_ertekek_alaphelyzetbe(self):
        self.gui.j1_entry.delete(0, tk.END)
        self.gui.j1_entry.insert(0, "0")
        self.gui.j2_entry.delete(0, tk.END)
        self.gui.j2_entry.insert(0, "0")
        self.gui.j3_entry.delete(0, tk.END)
        self.gui.j3_entry.insert(0, "0")
        self.gui.j4_entry.delete(0, tk.END)
        self.gui.j4_entry.insert(0, "90") # <-- ÚJ: Szervó alapértelmezett értéke
        
        self.gui.x_entry.delete(0, tk.END)
        self.gui.x_entry.insert(0, "131.5")
        self.gui.y_entry.delete(0, tk.END)
        self.gui.y_entry.insert(0, "0.0")
        self.gui.z_entry.delete(0, tk.END)
        self.gui.z_entry.insert(0, "118.0")

    def alaphelyzetbe_kenyszerites(self):
        self.aktualis_x = 131.5
        self.aktualis_y = 0.0
        self.aktualis_z = 118.0
        self._feluleti_ertekek_alaphelyzetbe()
        self.log_erkezett("[RENDSZER] Felület és memória szinkronizálva a fizikai nullaponthoz!\n")

    def frissit_xyz_mezok_lepesbol(self, j1, j2, j3):
        x, y, z = self.kinematika.direkt_kinematika(j1, j2, j3)
        self.aktualis_x = x
        self.aktualis_y = y
        self.aktualis_z = z
        
        self.gui.x_entry.delete(0, tk.END)
        self.gui.x_entry.insert(0, str(x))
        self.gui.y_entry.delete(0, tk.END)
        self.gui.y_entry.insert(0, str(y))
        self.gui.z_entry.delete(0, tk.END)
        self.gui.z_entry.insert(0, str(z))

    def esemeny_kapcsolodas(self):
        if self.soros.ser is None:
            port = self.gui.port_combobox.get()
            if not port:
                messagebox.showwarning("Figyelem", "Válassz ki egy COM portot!")
                return
            if self.soros.kapcsolodas(port):
                self.gui.connect_btn.config(text="Lecsatlakozás")
                self.gui.set_controls_state("normal")
                self.log_erkezett(f"[RENDSZER] Sikeres kapcsolat: {port}\n")
        else:
            self.soros.lecsatlakozas()
            self.gui.connect_btn.config(text="Csatlakozás")
            self.gui.set_controls_state("disabled")

    def esemeny_homing(self):
        self.alaphelyzetbe_kenyszerites()
        self.homing_folyamatban = True
        self.soros.parancs_kuldes("HOME")
        self.log_erkezett("[PARANCS] Homing parancs kiküldve, számítások zárolva...\n")

    def esemeny_koordinata_kuldes(self):
        """Tengelyek küldése gomb - MÓDOSÍTVA 4 PARAMÉTERRE"""
        if self.homing_folyamatban: return
        try:
            j1 = int(self.gui.j1_entry.get())
            j2 = int(self.gui.j2_entry.get())
            j3 = int(self.gui.j3_entry.get())
            j4 = int(self.gui.j4_entry.get()) # <-- ÚJ: Szervó fok kiolvasása
            
            self.frissit_xyz_mezok_lepesbol(j1, j2, j3)
            
            cmd = f"MOVE {j1} {j2} {j3} {j4}" # <-- ÚJ: 4 paraméteres parancs
            self.soros.parancs_kuldes(cmd)
        except ValueError:
            messagebox.showwarning("Hiba", "Kérlek csak egész számokat adj meg!")

    def esemeny_xyz_kuldes(self):
        """XYZ koordináták küldése - MÓDOSÍTVA 4 PARAMÉTERRE"""
        if self.homing_folyamatban: return
        try:
            x = float(self.gui.x_entry.get())
            y = float(self.gui.y_entry.get())
            z = float(self.gui.z_entry.get())
            j4 = int(self.gui.j4_entry.get()) # <-- ÚJ: Szervó megtartja az aktuális értékét
            
            self.aktualis_x = x
            self.aktualis_y = y
            self.aktualis_z = z
            
            eredmeny = self.kinematika.koordinata_szamitas(x, y, z)
            
            if eredmeny is not None:
                j1_steps, j2_steps, j3_steps = eredmeny
                
                self.gui.j1_entry.delete(0, tk.END)
                self.gui.j1_entry.insert(0, str(j1_steps))
                self.gui.j2_entry.delete(0, tk.END)
                self.gui.j2_entry.insert(0, str(j2_steps))
                self.gui.j3_entry.delete(0, tk.END)
                self.gui.j3_entry.insert(0, str(j3_steps))
                
                self.log_erkezett(f"[IK] Pont kiszámolva -> J1:{j1_steps} J2:{j2_steps} J3:{j3_steps}\n")
                
                cmd = f"MOVE {j1_steps} {j2_steps} {j3_steps} {j4}" # <-- ÚJ: 4 paraméteres parancs
                self.soros.parancs_kuldes(cmd)
            else:
                self.log_erkezett(f"[FIGYELEM] Elérhetetlen pont: (X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f})\n")
                messagebox.showwarning("Munkatéren kívül", "A robotkar mechanikailag nem éri el a megadott koordinátát!")
                
        except ValueError:
            messagebox.showwarning("Hiba", "Érvénytelen koordináta formátum a beviteli mezőkben!")
    def esemeny_xyz_leptetes(self, tengely_id, mm_valtozas):
        if self.homing_folyamatban: return
        t_nav = "".join(tengely_id)
        
        target_x = self.aktualis_x
        target_y = self.aktualis_y
        target_z = self.aktualis_z
        
        if "X" in t_nav:
            target_x += mm_valtozas
        elif "Y" in t_nav:
            target_y += mm_valtozas
        elif "Z" in t_nav:
            target_z += mm_valtozas
            
        eredmeny = self.kinematika.koordinata_szamitas(target_x, target_y, target_z)
        
        if eredmeny is not None:
            self.aktualis_x = target_x
            self.aktualis_y = target_y
            self.aktualis_z = target_z
            
            self.gui.x_entry.delete(0, tk.END)
            self.gui.x_entry.insert(0, f"{self.aktualis_x:.1f}")
            self.gui.y_entry.delete(0, tk.END)
            self.gui.y_entry.insert(0, f"{self.aktualis_y:.1f}")
            self.gui.z_entry.delete(0, tk.END)
            self.gui.z_entry.insert(0, f"{self.aktualis_z:.1f}")
            
            eredmeny_steps = self.kinematika.koordinata_szamitas(self.aktualis_x, self.aktualis_y, self.aktualis_z)
            if eredmeny_steps is not None:
                j1_s, j2_s, j3_s = eredmeny_steps
                j4 = int(self.gui.j4_entry.get()) # <-- ÚJ: Szervó megtartja az értékét XYZ léptetésnél is
                
                self.gui.j1_entry.delete(0, tk.END)
                self.gui.j1_entry.insert(0, str(j1_s))
                self.gui.j2_entry.delete(0, tk.END)
                self.gui.j2_entry.insert(0, str(j2_s))
                self.gui.j3_entry.delete(0, tk.END)
                self.gui.j3_entry.insert(0, str(j3_s))
                
                cmd = f"MOVE {j1_s} {j2_s} {j3_s} {j4}" # <-- ÚJ: 4 paraméter
                self.soros.parancs_kuldes(cmd)
        else:
            self.log_erkezett(f"[FIGYELEM] Tiltás: A pont (X:{target_x:.1f}, Y:{target_y:.1f}, Z:{target_z:.1f}) kívül esik a kar munkaterén!\n")

    def esemeny_leptetes(self, motor_id, fok_valtozas):
        """Manuális léptetés - MÓDOSÍTVA A J4 (SZERVÓ) KEZELÉSÉVEL"""
        if self.homing_folyamatban: return
        m_nev = "".join(motor_id)
        
        try:
            # Ha a szervót léptetjük (J4), akkor nem lépésszámot számolunk, hanem közvetlenül fokot módosítunk
            if "J4" in m_nev:
                uj_fok = int(self.gui.j4_entry.get()) + int(fok_valtozas)
                # Korlátozzuk a szervót biztonsági okokból 0 és 180 fok közé
                if uj_fok < 0: uj_fok = 0
                if uj_fok > 180: uj_fok = 180
                
                self.gui.j4_entry.delete(0, tk.END)
                self.gui.j4_entry.insert(0, str(uj_fok))
                
                # Azonnali küldés a többi motor jelenlegi pozíciójával együtt
                j1 = int(self.gui.j1_entry.get())
                j2 = int(self.gui.j2_entry.get())
                j3 = int(self.gui.j3_entry.get())
                cmd = f"MOVE {j1} {j2} {j3} {uj_fok}"
                self.soros.parancs_kuldes(cmd)
                return

            # Alapértelmezett léptetőmotoros logika (J1, J2, J3)
            steps_delta = int(fok_valtozas * self.kinematika.STEPS_PER_DEGREE)
            if "J1" in m_nev:
                uj_ert = int(self.gui.j1_entry.get()) + steps_delta
                self.gui.j1_entry.delete(0, tk.END)
                self.gui.j1_entry.insert(0, str(uj_ert))
            elif "J2" in m_nev:
                uj_ert = int(self.gui.j2_entry.get()) + steps_delta
                self.gui.j2_entry.delete(0, tk.END)
                self.gui.j2_entry.insert(0, str(uj_ert))
            elif "J3" in m_nev:
                uj_ert = int(self.gui.j3_entry.get()) + steps_delta
                self.gui.j3_entry.delete(0, tk.END)
                self.gui.j3_entry.insert(0, str(uj_ert))
                
            self.esemeny_koordinata_kuldes()
        except ValueError:
            messagebox.showwarning("Hiba", "Érvénytelen érték a léptetéshez!")

    def esemeny_s_gorbe_inditas(self):
        if self.homing_folyamatban: return
        self.log_erkezett("[QUEUE] 10x folyamatos korpálya indítása lassított időzítéssel...\n")
        
        palyapontok = self.kinematika.kor_palya_generalas(cx=143.0, cy=45.0, cz=-60.0, atmero=60.0, felbontas=36, ismetles=10)
        
        if not palyapontok:
            self.log_erkezett("[HIBA] A körpálya pontjait nem sikerült kiszámítani!\n")
            return
            
        j4 = int(self.gui.j4_entry.get()) # <-- ÚJ: Körpálya alatt a szervó megtartja a beállított fix pozícióját
        for pont in palyapontok:
            j1, j2, j3 = pont
            cmd = f"QMOVE {j1} {j2} {j3} {j4}" # <-- ÚJ: 4 paraméter
            self.soros.parancs_kuldes(cmd)
            time.sleep(0.08) 
            
        self.log_erkezett("[QUEUE] 10x körívpálya pontjai sikeresen leküldve!\n")
        
        if palyapontok:
            self.aktualis_x = 173.0
            self.aktualis_y = 45.0
            self.aktualis_z = -60.0
            
            self.gui.x_entry.delete(0, tk.END)
            self.gui.x_entry.insert(0, str(self.aktualis_x))
            self.gui.y_entry.delete(0, tk.END)
            self.gui.y_entry.insert(0, str(self.aktualis_y))
            self.gui.z_entry.delete(0, tk.END)
            self.gui.z_entry.insert(0, str(self.aktualis_z))

    def esemeny_qstop(self):
        self.soros.parancs_kuldes("QSTOP")
        self.log_erkezett("[VÉSZLEÁLLÍTÁS] Soros QSTOP parancs kiküldve!\n")

    def inditas(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = RobotkarAlkalmazas()
    app.inditas()
