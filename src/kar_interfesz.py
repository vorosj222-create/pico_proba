import tkinter as tk
from tkinter import messagebox
import time
import threading
import os
import sys
import subprocess
from soros_kezelo import SorosKezelo
from inverz_kinematika import InverzKinematika
from felhasznaloi_felulet import FelhasznaloiFelulet
from automatizacio import KarAutomatizacio 
from hengerkoordinata import HengerKinematika, UtvonalTervezo, PalyaEteto
import malom_tabla

class RobotkarAlkalmazas:
    def __init__(self):
        self.root = tk.Tk()
        
        # Az új geometriai modell szerinti induló koordináták (L2=118mm, Z_offset=67mm)
        # Hengerkoordináták: r [mm], phi [fok], z [mm]
        self.aktualis_r = 118.0
        self.aktualis_phi = 0.0
        self.aktualis_z = 51.0
        self.homing_folyamatban = False
        self.kamera = None   # a Kamera_nezo.py folyamata (inditas() állítja be)
        
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
            on_auto_sequence=self.esemeny_automatizacio_inditas,
            on_malom_pont=self.esemeny_malom_pont,
            on_babu_athelyezes=self.esemeny_babu_athelyezes,
            on_kamera_kalibracio=self.esemeny_kamera_kalibracio
        )
        
        self._feluleti_ertekek_alaphelyzetbe()
        self._malom_pontok_betoltese()
        self.gui.portok_frissitese(self.soros.aktiv_portok_lekerese())

    def log_erkezett(self, szoveg):
        if szoveg.startswith("SOR: OK"):
            return  # pályakövetés közben 30 ms-onként jön, ne árassza el a monitort
        if szoveg.startswith("VESZLEALLITAS: MOTOROK ARAMTALANITVA"):
            # Hardveres vészleállító: a Python oldali pályaküldést is azonnal leállítjuk
            self.eteto.leallit()
            self.root.after(0, lambda: messagebox.showwarning(
                "Vészleállítás", "A vészleállító gombot megnyomták, a motorok áramtalanítva.\n"
                                 "A pozíció elveszett: engedd ki a gombot, majd Homing!"))
        if threading.current_thread() is threading.main_thread():
            self.gui.log_kiiras(szoveg)
        else:
            # háttérszálból (soros port, kamera, mozgás) a kiírás a fő szálon történjen
            self.root.after(0, self.gui.log_kiiras, szoveg)
        # A homing csak a legvégén érkező "STATUSZ: ALAPHELYZETBEN" üzenettel ér véget
        # (az egyes tengelyek "... HOMING KESZ" üzenete még nem jelenti a teljes homing végét)
        if "ALAPHELYZETBEN" in szoveg:
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

    def _malom_pontok_betoltese(self):
        """A 8 bemért pontból kiszámolja a 24 malompontot, és jelzi, ha valami gyanús."""
        self.malom_pontok = malom_tabla.malom_pontok_szamitasa()
        for p, e in malom_tabla.ellenorzes().items():
            if e > 8.0:
                self.log_erkezett(f"[MALOM FIGYELEM] A(z) {p}. bemért pont {e:.0f} mm-re esik a szabályos "
                                  f"táblától - elírás vagy felcserélt pont? (malom_tabla.py)\n")
        for p, (r, phi, z) in self.malom_pontok.items():
            if (self.henger.henger_to_lepes(r, phi, z) is None or
                    self.henger.henger_to_lepes(r, phi, malom_tabla.FELSO_Z + malom_tabla.KANYAR_MM) is None):
                self.log_erkezett(f"[MALOM FIGYELEM] A(z) {p}. pont (r={r:.0f}, φ={phi:.1f}) nem elérhető!\n")

    def esemeny_malom_pont(self):
        """A kart a megadott malomponthoz viszi: felemelkedik, átmegy fölé, majd leereszkedik."""
        if self.homing_folyamatban: return
        try:
            pont = int(self.gui.malom_spin.get())
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
        except ValueError:
            messagebox.showwarning("Hiba", "A pont száma 0 és 41 közötti egész szám legyen (24-41: tartalék helyek)!")
            return
        if pont not in self.malom_pontok:
            messagebox.showwarning("Hiba", "A pont száma 0 és 41 közötti egész szám legyen (24-41: tartalék helyek)!")
            return
        if self.eteto.foglalt():
            self.log_erkezett("[FIGYELEM] Mozgás folyamatban, várd meg a végét vagy QSTOP!\n")
            return
        r, phi, z = self.malom_pontok[pont]
        self.log_erkezett(f"[MALOM] Mozgás a(z) {pont}. ponthoz: r={r:.1f} φ={phi:.1f} z={z:.1f}\n")
        self.eteto.uj_mozgas()
        self.eteto.lefoglalva = True

        def folyamat():
            try:
                if not self._ponthoz_megy(pont, z, j4, j5):
                    self.log_erkezett("[MALOM] Mozgás megszakítva.\n")
            finally:
                self.eteto.lefoglalva = False
        threading.Thread(target=folyamat, daemon=True).start()

    def _ponthoz_megy(self, pont, z_lent, j4, j5):
        """
        Blokkoló: fel, át a pont fölé, le z_lent-re - egyetlen, lekerekített, megállás nélküli pályán.
        A kar FELSO_Z + KANYAR_MM magasan halad, a sarkokat pedig legfeljebb KANYAR_MM-rel kerekíti le,
        így a kanyar a ponton kívül sosem viszi FELSO_Z alá (a lekerekítés legfeljebb ennyit vág le).
        """
        utvonal = self._pont_feletti_utvonal(pont) + [(*self.malom_pontok[pont][:2], z_lent)]
        return self.mozgas_utvonalon(utvonal, j4, j5, blokkolo=True, lekerekites_mm=malom_tabla.KANYAR_MM)

    def _pont_feletti_utvonal(self, pont):
        """
        A haladási magasságba (FELSO_Z + KANYAR_MM), majd a pont fölé. Mindig ezen a magasságon halad
        akkor is, ha a kar éppen magasabban áll: magasan a kar rövidebbre ér el, így pl. az
        alaphelyzetből (z=51) a távoli pontok (pl. 23) fölé nem lehetne átmenni.
        """
        r, phi, _ = self.malom_pontok[pont]
        z_fent = malom_tabla.FELSO_Z + malom_tabla.KANYAR_MM
        return [(self.aktualis_r, self.aktualis_phi, z_fent), (r, phi, z_fent)]

    def esemeny_babu_athelyezes(self):
        """Bábu áthelyezése: honnan fölé, fogó nyit, le, fogó zár, hová (fel-át-le), fogó nyit."""
        if self.homing_folyamatban: return
        if self.eteto.foglalt():
            self.log_erkezett("[FIGYELEM] Mozgás folyamatban, várd meg a végét vagy QSTOP!\n")
            return
        try:
            honnan = int(self.gui.babu_honnan_spin.get())
            hova = int(self.gui.babu_hova_spin.get())
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
        except ValueError:
            messagebox.showwarning("Hiba", "A pontok száma 0 és 41 közötti egész szám legyen (24-41: tartalék helyek)!")
            return
        if honnan not in self.malom_pontok or hova not in self.malom_pontok or honnan == hova:
            messagebox.showwarning("Hiba", "Két különböző, 0 és 41 közötti pontot adj meg (24-41: tartalék helyek)!")
            return
        self.eteto.uj_mozgas()
        self.eteto.lefoglalva = True
        threading.Thread(target=self._babu_athelyezes_folyamat, args=(honnan, hova, j4, j5), daemon=True).start()

    def _babu_athelyezes_folyamat(self, honnan, hova, j4, j5):
        try:
            self.log_erkezett(f"[BÁBU] Áthelyezés: {honnan} -> {hova}\n")
            # 1. a "honnan" pont fölé (fel a biztonsági magasságba, majd át)
            r, phi, _ = self.malom_pontok[honnan]
            if not self.mozgas_utvonalon(self._pont_feletti_utvonal(honnan), j4, j5, blokkolo=True,
                                         lekerekites_mm=malom_tabla.KANYAR_MM): return self._babu_megszakitva()
            # 2. fogó nyit
            if not self._fogo_allitas(j4, malom_tabla.CLAW_NYITVA): return self._babu_megszakitva()
            # 3. le a bábuhoz
            if not self.mozgas_utvonalon([(r, phi, malom_tabla.BABU_Z)], j4, malom_tabla.CLAW_NYITVA, blokkolo=True): return self._babu_megszakitva()
            # 4. fogó zár (megfogás)
            if not self._fogo_allitas(j4, malom_tabla.CLAW_ZARVA): return self._babu_megszakitva()
            # 5. fel, át a "hová" pont fölé, le (lekerekítve, a pontok között sosem megy FELSO_Z alá)
            if not self._ponthoz_megy(hova, malom_tabla.BABU_Z, j4, malom_tabla.CLAW_ZARVA): return self._babu_megszakitva()
            # 6. fogó nyit (elengedés)
            if not self._fogo_allitas(j4, malom_tabla.CLAW_NYITVA): return self._babu_megszakitva()
            self.log_erkezett(f"[BÁBU] Kész: {honnan} -> {hova}\n")
        finally:
            self.eteto.lefoglalva = False

    def _babu_megszakitva(self):
        self.log_erkezett("[BÁBU] Áthelyezés megszakítva.\n")

    def _fogo_allitas(self, j4, j5):
        """Csak a fogót (J5) állítja, a kar helyben marad; utána vár. False, ha közben QSTOP jött."""
        steps = self.henger.henger_to_lepes(self.aktualis_r, self.aktualis_phi, self.aktualis_z)
        if steps is None or not self.eteto.kuld(f"MOVE {steps[0]} {steps[1]} {steps[2]} {j4} {j5}"):
            return False
        self.root.after(0, lambda: (self.gui.j5_entry.delete(0, tk.END), self.gui.j5_entry.insert(0, str(j5))))
        return not self.eteto._stop.wait(malom_tabla.CLAW_VARAKOZAS)

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

    def mozgas_utvonalon(self, pontok, j4, j5, blokkolo=False, lekerekites_mm=None):
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
        mintak, hiba = self.tervezo.tervez([start] + list(pontok), lekerekites_mm=lekerekites_mm)
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

    def _kamera_inditas(self):
        """
        A Kamera_nezo.py-t külön folyamatként indítja. A bemenetén (stdin) kap parancsot
        (pl. KALIBRALAS), a kimenetét (stdout) egy háttérszál olvassa és a soros monitorra írja.
        """
        kamera_fajl = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Kamera_nezo.py")
        if not os.path.exists(kamera_fajl):
            print(f"[KAMERA] Nem található: {kamera_fajl}")
            return None
        kornyezet = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        try:
            folyamat = subprocess.Popen([sys.executable, kamera_fajl], cwd=os.path.dirname(kamera_fajl),
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, encoding="utf-8", errors="replace", bufsize=1, env=kornyezet)
        except OSError as e:
            print(f"[KAMERA] Nem sikerült elindítani: {e}")
            return None
        threading.Thread(target=self._kamera_kimenet_olvaso, args=(folyamat,), daemon=True).start()
        return folyamat

    def _kamera_kimenet_olvaso(self, folyamat):
        """A kamera üzeneteit a soros monitorra írja; a kalibráció eredményét felugró ablakban is jelzi."""
        for sor in folyamat.stdout:
            sor = sor.rstrip()
            if not sor:
                continue
            if sor.startswith("KALIBRACIO_OK"):
                szoveg = sor[len("KALIBRACIO_OK"):].strip()
                self.log_erkezett(f"[KAMERA] Kalibráció kész. Illeszkedés: {szoveg}\n")
                self.root.after(0, lambda t=szoveg: messagebox.showinfo("Kamera kalibráció", f"Kalibráció kész.\nIlleszkedés: {t}"))
            elif sor.startswith("KALIBRACIO_HIBA"):
                szoveg = sor[len("KALIBRACIO_HIBA"):].strip()
                self.log_erkezett(f"[KAMERA HIBA] Kalibráció sikertelen: {szoveg}\n")
                self.root.after(0, lambda t=szoveg: messagebox.showwarning("Kamera kalibráció", f"Kalibráció sikertelen:\n{t}"))
            else:
                self.log_erkezett(f"[KAMERA] {sor}\n")
        self.log_erkezett("[KAMERA] A kamera program leállt.\n")

    def kamera_parancs(self, parancs):
        """Parancs küldése a kamera programnak. False, ha a kamera nem fut."""
        if self.kamera is None or self.kamera.poll() is not None:
            return False
        try:
            self.kamera.stdin.write(parancs + "\n")
            self.kamera.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    def esemeny_kamera_kalibracio(self):
        """A kart kiviszi a képből, majd a kamerával megjegyezteti a 24 malompont pixelhelyét."""
        if self.homing_folyamatban: return
        if self.kamera is None or self.kamera.poll() is not None:
            messagebox.showwarning("Kamera kalibráció", "A kamera program nem fut!")
            return
        if self.eteto.foglalt():
            self.log_erkezett("[FIGYELEM] Mozgás folyamatban, várd meg a végét vagy QSTOP!\n")
            return
        try:
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
        except ValueError:
            return
        self.eteto.uj_mozgas()
        self.eteto.lefoglalva = True

        def folyamat():
            try:
                self.log_erkezett("[KAMERA] Kalibráció: a kar kiáll a képből...\n")
                if not self.mozgas_utvonalon([malom_tabla.KALIBRACIOS_POZICIO], j4, j5, blokkolo=True):
                    self.log_erkezett("[KAMERA] Kalibráció megszakítva (a kar nem ért oda).\n")
                    return
                if self.eteto._stop.wait(0.5):      # rezgés lecsengése; QSTOP-ra megszakad
                    return
                if self.kamera_parancs("KALIBRALAS"):
                    self.log_erkezett("[KAMERA] Pontok keresése a kamerában...\n")
                else:
                    self.log_erkezett("[KAMERA HIBA] A kamera program nem fut!\n")
            finally:
                self.eteto.lefoglalva = False
        threading.Thread(target=folyamat, daemon=True).start()

    def inditas(self):
        self.kamera = self._kamera_inditas()
        try:
            self.root.mainloop()
        finally:
            # a robotkar ablakának bezárásakor a kamera is leáll
            if self.kamera is not None and self.kamera.poll() is None:
                self.kamera.terminate()

if __name__ == "__main__":
    app = RobotkarAlkalmazas()
    app.inditas()