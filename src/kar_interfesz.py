import tkinter as tk
from tkinter import messagebox
import time
from soros_kezelo import SorosKezelo
from inverz_kinematika import InverzKinematika
from felhasznaloi_felulet import FelhasznaloiFelulet
from automatizacio import KarAutomatizacio 
from hengerkoordinata import HengerKinematika, UtvonalTervezo, PalyaEteto

class RobotkarAlkalmazas:
    def __init__(self):
        self.root = tk.Tk()
        
        # Az új geometriai modell szerinti induló koordináták (L2=118mm, Z_offset=67mm)
        # Hengerkoordináták: r [mm], phi [fok], z [mm]
        self.aktualis_r = 118.0
        self.aktualis_phi = 0.0
        self.aktualis_z = 51.0
        self.homing_folyamatban = False
        
        self.soros = SorosKezelo(log_callback=self.log_erkezett)
        self.kinematika = InverzKinematika()
        self.henger = HengerKinematika(self.kinematika)
        self.tervezo = UtvonalTervezo(self.henger)
        self.eteto = PalyaEteto(self.soros, self.tervezo.DT)
        self.automatizacio = KarAutomatizacio(interfesz_app=self)
        
        # JAVÍTVA: Az on_s_curve teljesen el lett távolítva, így megszűnik a TypeError hiba!
        self.gui = FelhasznaloiFelulet(
            root=self.root,
            on_connect=self.esemeny_kapcsolodas,
            on_home=self.esemeny_homing,
            on_send=self.esemeny_koordinata_kuldes,
            on_jog=self.esemeny_leptetes,
            on_send_henger=self.esemeny_henger_kuldes,
            on_jog_henger=self.esemeny_henger_leptetes,
            on_qstop=self.esemeny_qstop,
            on_auto_sequence=self.esemeny_automatizacio_inditas
        )
        
        self._feluleti_ertekek_alaphelyzetbe()
        self.gui.portok_frissitese(self.soros.aktiv_portok_lekerese())

    def log_erkezett(self, szoveg):
        if szoveg.startswith("SOR: OK"):
            return  # pályakövetés közben 30 ms-onként jön, ne árassza el a monitort
        self.gui.log_kiiras(szoveg)
        if "STATUSZ" in szoveg or "KESZ" in szoveg or "ALAPHELYZETBEN" in szoveg:
            self.homing_folyamatban = False

    def _feluleti_ertekek_alaphelyzetbe(self):
        self.gui.j1_entry.delete(0, tk.END); self.gui.j1_entry.insert(0, "0")
        self.gui.j2_entry.delete(0, tk.END); self.gui.j2_entry.insert(0, "0")
        self.gui.j3_entry.delete(0, tk.END); self.gui.j3_entry.insert(0, "0")
        self.gui.j4_entry.delete(0, tk.END); self.gui.j4_entry.insert(0, "90") 
        self.gui.j5_entry.delete(0, tk.END); self.gui.j5_entry.insert(0, "90") 
        
        # A felületi mezők induló értékei a fizikai nullaponthoz (R:118, φ:0, Z:51) igazodnak
        self._henger_mezok_kiirasa(118.0, 0.0, 51.0)
    def alaphelyzetbe_kenyszerites(self):
        self.aktualis_r = 118.0
        self.aktualis_phi = 0.0
        self.aktualis_z = 51.0
        self._feluleti_ertekek_alaphelyzetbe()
        self.log_erkezett("[RENDSZER] Felület és memória szinkronizálva a nullaponthoz!\n")

    def _henger_mezok_kiirasa(self, r, phi, z):
        self.gui.r_entry.delete(0, tk.END); self.gui.r_entry.insert(0, f"{r:.1f}")
        self.gui.phi_entry.delete(0, tk.END); self.gui.phi_entry.insert(0, f"{phi:.1f}")
        self.gui.z_entry.delete(0, tk.END); self.gui.z_entry.insert(0, f"{z:.1f}")

    def _lepes_mezok_kiirasa(self, j1, j2, j3):
        self.gui.j1_entry.delete(0, tk.END); self.gui.j1_entry.insert(0, str(j1))
        self.gui.j2_entry.delete(0, tk.END); self.gui.j2_entry.insert(0, str(j2))
        self.gui.j3_entry.delete(0, tk.END); self.gui.j3_entry.insert(0, str(j3))

    def frissit_henger_mezok_lepesbol(self, j1, j2, j3):
        r, phi, z = self.henger.lepes_to_henger(j1, j2, j3)
        self.aktualis_r, self.aktualis_phi, self.aktualis_z = r, phi, z
        self._henger_mezok_kiirasa(r, phi, z)

    def mozgas_utvonalon(self, pontok, j4, j5, blokkolo=False):
        """
        pontok: célpontok hengerkoordinátában [(r, phi, z), ...]; a jelenlegi pozícióból indul.
        blokkolo=True: a hívó szálán fut (automatizáció), különben háttérszálon.
        Visszatér: True, ha a pálya elindult / végigment.
        """
        if self.eteto.foglalt() and not blokkolo:
            self.log_erkezett("[FIGYELEM] Mozgás folyamatban, várd meg a végét vagy QSTOP!\n")
            return False
        if not blokkolo:
            self.eteto.uj_mozgas()
        start = (self.aktualis_r, self.aktualis_phi, self.aktualis_z)
        mintak, hiba = self.tervezo.tervez([start] + list(pontok))
        if mintak is None:
            self.root.after(0, lambda: messagebox.showwarning("Munkatéren kívül", hiba))
            self.log_erkezett(f"[HIBA] {hiba}\n")
            return False
        cel_r, cel_phi, cel_z = pontok[-1]
        vege = mintak[-1]

        def kesz(ok):
            if ok:
                self.aktualis_r, self.aktualis_phi, self.aktualis_z = cel_r, cel_phi, cel_z
                self.root.after(0, lambda: (self._henger_mezok_kiirasa(cel_r, cel_phi, cel_z),
                                            self._lepes_mezok_kiirasa(*vege)))
            else:
                self._pozicio_leallitas_utan()

        if blokkolo:
            ok = self.eteto.futtat(mintak, j4, j5)
            kesz(ok)
            return ok
        self.eteto.inditas_hatterben(mintak, j4, j5, kesz)
        return True

    def _pozicio_leallitas_utan(self):
        """QSTOP után a pozíció az utoljára kiküldött pályapontból becsülve (a fékút miatt közelítő)."""
        if self.eteto.utolso_minta is None:
            return
        j1, j2, j3 = self.eteto.utolso_minta
        r, phi, z = self.henger.lepes_to_henger(j1, j2, j3)
        self.aktualis_r, self.aktualis_phi, self.aktualis_z = r, phi, z
        self.root.after(0, lambda: (self._henger_mezok_kiirasa(r, phi, z), self._lepes_mezok_kiirasa(j1, j2, j3)))
        self.log_erkezett("[FIGYELEM] Mozgás megszakítva, a pozíció közelítő (fékút). Szükség esetén homing!\n")

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
        self.eteto.leallit()     # futó pálya leállítása (QSTOP)
        self.eteto.uj_mozgas()   # a homing után újra lehessen mozogni
        self.alaphelyzetbe_kenyszerites()
        self.homing_folyamatban = True
        self.soros.parancs_kuldes("HOME")
        self.log_erkezett("[PARANCS] Homing parancs kiküldve...\n")

    def esemeny_koordinata_kuldes(self):
        if self.homing_folyamatban or self.eteto.foglalt(): return
        try:
            j1 = int(self.gui.j1_entry.get())
            j2 = int(self.gui.j2_entry.get())
            j3 = int(self.gui.j3_entry.get())
            j4 = int(self.gui.j4_entry.get()) 
            j5 = int(self.gui.j5_entry.get()) 
            
            self.frissit_henger_mezok_lepesbol(j1, j2, j3)
            cmd = f"MOVE {j1} {j2} {j3} {j4} {j5}" 
            self.soros.parancs_kuldes(cmd)
        except ValueError:
            messagebox.showwarning("Hiba", "Kérlek csak egész számokat adj meg!")

    def esemeny_henger_kuldes(self):
        if self.homing_folyamatban: return
        try:
            r = float(self.gui.r_entry.get())
            phi = float(self.gui.phi_entry.get())
            z = float(self.gui.z_entry.get())
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
        except ValueError:
            messagebox.showwarning("Hiba", "Érvénytelen koordináta formátum!")
            return
        self.mozgas_utvonalon([(r, phi, z)], j4, j5)

    def esemeny_henger_leptetes(self, tengely_id, valtozas):
        # R és Z: mm, φ: fok. Kis léptetés, közvetlen MOVE-val (mint eddig).
        if self.homing_folyamatban or self.eteto.foglalt(): return
        cel_r, cel_phi, cel_z = self.aktualis_r, self.aktualis_phi, self.aktualis_z
        if "R" in tengely_id: cel_r += valtozas
        elif "φ" in tengely_id: cel_phi += valtozas
        elif "Z" in tengely_id: cel_z += valtozas

        eredmeny_steps = self.henger.henger_to_lepes(cel_r, cel_phi, cel_z)
        if eredmeny_steps is not None:
            self.aktualis_r, self.aktualis_phi, self.aktualis_z = cel_r, cel_phi, cel_z
            self._henger_mezok_kiirasa(cel_r, cel_phi, cel_z)
            j1_s, j2_s, j3_s = eredmeny_steps
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
            self._lepes_mezok_kiirasa(j1_s, j2_s, j3_s)
            self.soros.parancs_kuldes(f"MOVE {j1_s} {j2_s} {j3_s} {j4} {j5}")
        else:
            self.log_erkezett("[FIGYELEM] A léptetés célja munkatéren / csuklókorláton kívül esik.\n")

    def esemeny_leptetes(self, motor_id, fok_valtozas):
        if self.homing_folyamatban: return
        try:
            if "J5" in motor_id:
                uj_szog = int(self.gui.j5_entry.get()) + int(fok_valtozas)
                uj_szog = max(0, min(180, uj_szog))
                self.gui.j5_entry.delete(0, tk.END); self.gui.j5_entry.insert(0, str(uj_szog))
                self.esemeny_koordinata_kuldes()
                return
            elif "J4" in motor_id:
                uj_szog = int(self.gui.j4_entry.get()) + int(fok_valtozas)
                uj_szog = max(0, min(180, uj_szog))
                self.gui.j4_entry.delete(0, tk.END); self.gui.j4_entry.insert(0, str(uj_szog))
                self.esemeny_koordinata_kuldes()
                return

            steps_delta = int(fok_valtozas * self.kinematika.STEPS_PER_DEGREE)
            if "J1" in motor_id:
                uj_ert = int(self.gui.j1_entry.get()) + steps_delta
                self.gui.j1_entry.delete(0, tk.END); self.gui.j1_entry.insert(0, str(uj_ert))
            elif "J2" in motor_id:
                uj_ert = int(self.gui.j2_entry.get()) + steps_delta
                self.gui.j2_entry.delete(0, tk.END); self.gui.j2_entry.insert(0, str(uj_ert))
            elif "J3" in motor_id:
                uj_ert = int(self.gui.j3_entry.get()) + steps_delta
                self.gui.j3_entry.delete(0, tk.END); self.gui.j3_entry.insert(0, str(uj_ert))
                
            self.esemeny_koordinata_kuldes()
        except ValueError:
            messagebox.showwarning("Hiba", "Érvénytelen érték!")

    def esemeny_qstop(self):
        # A Python oldali etetést és a QSTOP-ot egyszerre, zár alatt állítjuk le,
        # így a QSTOP után már egyetlen QMOVE sem mehet ki, ami újraindítaná a kart.
        self.eteto.leallit()
        self.log_erkezett("[VÉSZLEÁLLÍTÁS] QSTOP kiküldve.\n")

    # Az automata kávécukor-pakolási folyamat indítása
    def esemeny_automatizacio_inditas(self):
        self.automatizacio.futtat_munkafolyamat()

    def inditas(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = RobotkarAlkalmazas()
    app.inditas()