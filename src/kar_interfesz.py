import tkinter as tk
from tkinter import messagebox
import time
import threading
import queue
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
        self.kamera_kalibralva = False
        self.tabla_uzenetek = queue.Queue()   # a kamera "TABLA ..." üzenetei (42 karakteres állapot vagy None)
        
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
            on_kamera_kalibracio=self.esemeny_kamera_kalibracio,
            on_malom_inditas=self.esemeny_malom_inditas
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
            self._babu_mozgatas(honnan, hova, j4, j5)
        finally:
            self.eteto.lefoglalva = False

    def _babu_mozgatas(self, honnan, hova, j4, j5):
        """Blokkoló: egy bábut áttesz a "honnan" pontról a "hová" pontra. True, ha végigment."""
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
        return True

    def _babu_megszakitva(self):
        self.log_erkezett("[BÁBU] Áthelyezés megszakítva.\n")
        return False

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
            if sor.startswith("TABLA"):
                # tábla állapot: 42 karakter ('P', 'Z', '.'), 0-23 tábla, 24-41 tartalék; nem írjuk ki
                self.tabla_uzenetek.put(sor[6:].strip() if sor.startswith("TABLA ") else None)
                continue
            if sor.startswith("KALIBRACIO_OK"):
                self.kamera_kalibralva = True
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
                if not self._malom_kar_kepbol_ki(j4, j5):
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

    # =====================================================================
    # MALOM JÁTÉK - 1. lépés: kezdés (ki kezd?) és az ember első lépésének felismerése
    # =====================================================================
    def esemeny_malom_inditas(self):
        if self.homing_folyamatban: return
        if self.kamera is None or self.kamera.poll() is not None:
            messagebox.showwarning("Malom", "A kamera program nem fut!")
            return
        if not self.kamera_kalibralva:
            messagebox.showwarning("Malom", "Előbb kalibráld a kamerát (KAMERA KALIBRÁCIÓ, üres táblán)!")
            return
        if self.eteto.foglalt():
            self.log_erkezett("[FIGYELEM] Mozgás folyamatban, várd meg a végét vagy QSTOP!\n")
            return
        try:
            j4 = int(self.gui.j4_entry.get())
            j5 = int(self.gui.j5_entry.get())
        except ValueError:
            return
        nehezseg = self._nehezseg_valasztas()
        if nehezseg is None:
            return                                           # mégsem
        self.eteto.uj_mozgas()
        self.eteto.lefoglalva = True
        threading.Thread(target=self._malom_kezdes, args=(j4, j5, nehezseg), daemon=True).start()

    def _nehezseg_valasztas(self):
        """Kis ablak három gombbal: Könnyű / Közepes / Nehéz. Visszatér: 'konnyu' / 'kozepes' / 'nehez' / None."""
        valasztas = {"ertek": None}
        ablak = tk.Toplevel(self.root)
        ablak.title("Malom - nehézség")
        ablak.configure(bg="#1e272e")
        ablak.resizable(False, False)
        ablak.transient(self.root)
        tk.Label(ablak, text="Milyen erős legyen a robot?", bg="#1e272e", fg="white",
                 font=('Helvetica', 12, 'bold')).pack(padx=25, pady=(18, 10))
        sor = tk.Frame(ablak, bg="#1e272e")
        sor.pack(padx=20, pady=(0, 18))
        for kulcs, nev, szin in (("konnyu", "Könnyű", "#0be881"), ("kozepes", "Közepes", "#ffa801"),
                                 ("nehez", "Nehéz", "#ff3f34")):
            tk.Button(sor, text=nev, width=10, font=('Helvetica', 11, 'bold'), bg=szin, fg="black",
                      command=lambda k=kulcs: (valasztas.update(ertek=k), ablak.destroy())
                      ).pack(side="left", padx=6)
        ablak.update_idletasks()                             # a főablak közepére
        x = self.root.winfo_rootx() + (self.root.winfo_width() - ablak.winfo_reqwidth()) // 2
        y = self.root.winfo_rooty() + (self.root.winfo_height() - ablak.winfo_reqheight()) // 3
        ablak.geometry(f"+{max(0, x)}+{max(0, y)}")
        ablak.grab_set()                                     # amíg nyitva van, a főablak nem kezelhető
        self.root.wait_window(ablak)
        return valasztas["ertek"]

    def _malom_kar_kepbol_ki(self, j4, j5):
        """
        A kart kiviszi a kamera látószögéből (a kalibrációs helyzetbe). False, ha nem sikerült.
        Ha a kar lent van (pl. épp letett egy bábut), előbb függőlegesen felemelkedik a haladási
        magasságba, és csak onnan megy ki, különben elsodorná a bábukat. A sarkot ugyanúgy
        kerekíti le, mint a bábumozgatásnál, így a ponton kívül sosem megy FELSO_Z alá.
        """
        z_fent = max(self.aktualis_z, malom_tabla.FELSO_Z + malom_tabla.KANYAR_MM)
        utvonal = [(self.aktualis_r, self.aktualis_phi, z_fent), malom_tabla.KALIBRACIOS_POZICIO]
        return self.mozgas_utvonalon(utvonal, j4, j5, blokkolo=True, lekerekites_mm=malom_tabla.KANYAR_MM)

    def _tabla_allapot_lekerese(self, varakozas=3.0):
        """Lekéri a kamerától a jelenlegi tartós táblaállapotot (42 karakter), vagy None."""
        while not self.tabla_uzenetek.empty():          # régi üzenetek eldobása
            self.tabla_uzenetek.get_nowait()
        if not self.kamera_parancs("ALLAPOT"):
            return None
        try:
            return self.tabla_uzenetek.get(timeout=varakozas)
        except queue.Empty:
            return None

    def _malom_uzenet(self, szoveg, felugro=False):
        self.log_erkezett(f"[MALOM] {szoveg}\n")
        if felugro:
            self.root.after(0, lambda: messagebox.showinfo("Malom", szoveg))

    def _ember_lepesre_var(self, alap, hatarido=None):
        """
        Megvárja, amíg az ember egy piros bábut tesz egy üres táblapontra (új stabil helyzet a kamerától).
        alap: az előző stabil helyzet (42 karakter). hatarido: time.time() érték vagy None (nincs időkorlát).
        Visszatér: (új_állapot, pont), vagy (None, None) időtúllépéskor / QSTOP-ra.
        Nem értelmezhető változásnál szól, és tovább vár (az ember kijavíthatja).
        """
        jelzett = None
        while True:
            if self.eteto._stop.is_set():
                return None, None
            maradek = None if hatarido is None else hatarido - time.time()
            if maradek is not None and maradek <= 0:
                return None, None
            try:
                allapot = self.tabla_uzenetek.get(timeout=0.25 if maradek is None else min(0.25, maradek))
            except queue.Empty:
                continue
            if allapot is None:
                continue
            uj = [p for p in range(24) if alap[p] == "." and allapot[p] != "."]
            eltunt = [p for p in range(24) if alap[p] != "." and allapot[p] == "."]
            valtozott = [p for p in range(24) if alap[p] != "." and allapot[p] not in (".", alap[p])]
            if not uj and not eltunt and not valtozott:
                continue                                   # visszaállt az előző helyzet: semmi
            if len(uj) == 1 and not eltunt and not valtozott and allapot[uj[0]] == malom_tabla.EMBER_SZIN:
                return allapot, uj[0]
            leiras = (f"új: {[(p, allapot[p]) for p in uj]}, eltűnt: {eltunt}, színt váltott: {valtozott}")
            if leiras != jelzett:
                jelzett = leiras
                self._malom_uzenet(f"Nem értelmezhető változás a táblán ({leiras}). "
                                   f"Egy piros bábut tegyél egy üres pontra!")

    def _stabil_allapotra_var(self, feltetel, idotullepes=6.0):
        """Lekéri a táblaállapotot, és vár, amíg a feltétel teljesül (vagy lejár az idő). Az utolsó állapot."""
        utolso = self._tabla_allapot_lekerese()
        hatarido = time.time() + idotullepes
        while not (utolso is not None and feltetel(utolso)) and time.time() < hatarido:
            if self.eteto._stop.is_set():
                break
            try:
                uj = self.tabla_uzenetek.get(timeout=0.25)
            except queue.Empty:
                continue
            if uj is not None:
                utolso = uj
        return utolso

    def _tabla_egyezesre_var(self, motor):
        """
        A robot lépése után: megvárja, hogy a kamera által látott tábla megegyezzen a motor állapotával.
        Eltérésnél kiírja a különbséget, és vár a kézi javításra (QSTOP-ra megszakad).
        Visszatér: a kamera állapota (42 karakter), vagy None.
        """
        vart = motor.tabla()
        allapot = self._stabil_allapotra_var(lambda a: a[:24] == vart)
        jelzett = None
        while allapot is None or allapot[:24] != vart:
            if self.eteto._stop.is_set():
                return None
            if allapot is not None:
                elteres = ", ".join(f"{p}: várt '{vart[p]}', látott '{allapot[p]}'"
                                    for p in range(24) if allapot[p] != vart[p])
                if elteres != jelzett:
                    jelzett = elteres
                    self._malom_uzenet(f"A tábla nem egyezik a játékállással ({elteres}). "
                                       f"Kérlek, javítsd ki kézzel!")
            try:
                uj = self.tabla_uzenetek.get(timeout=0.25)
            except queue.Empty:
                continue
            if uj is not None:
                allapot = uj
        return allapot

    def _robot_lep(self, motor, alap, j4, j5):
        """
        A robot lépése a malom motor szerint: lerakás a 33-41 tartalékból, vagy tologatás / ugrás a táblán,
        malom esetén a levett piros bábu a 24-32 helyek következőjére. Visszatér: a kamera állapota vagy None.
        """
        self._malom_uzenet("A robot gondolkodik...")
        lepes = motor.robot_lepese()
        honnan, hova = motor.lepes_bontasa(lepes)
        if honnan is None:                                   # lerakás: bábu a tartalékból
            tartalek = [p for p in malom_tabla.ALSO_TARTALEK if alap[p] == malom_tabla.ROBOT_SZIN]
            if not tartalek:
                self._malom_uzenet("Nincs több zöld bábu a 33-41 tartalékhelyeken!", True)
                return None
            honnan = tartalek[0]
            self._malom_uzenet(f"A robot lép: zöld bábu a(z) {honnan}. tartalékhelyről a(z) {hova}. pontra.")
        else:
            self._malom_uzenet(f"A robot lép: {honnan} -> {hova}.")
        if not self._babu_mozgatas(honnan, hova, j4, j5):
            return None
        motor.lep(lepes)

        if motor.robot_jon() and motor.levetel_jon():        # malom: le kell venni egy piros bábut
            levetel = motor.robot_lepese()
            _, pont = motor.lepes_bontasa(levetel)
            if self._levett_db >= len(malom_tabla.FELSO_TARTALEK):
                self._malom_uzenet("Nincs több hely a levett bábuknak (24-32)!", True)
                return None
            hely = malom_tabla.FELSO_TARTALEK[self._levett_db]
            self._malom_uzenet(f"Malom! A robot leveszi a(z) {pont}. ponton álló piros bábut "
                               f"(a(z) {hely}. helyre teszi).")
            if not self._babu_mozgatas(pont, hely, j4, j5):
                return None
            motor.lep(levetel)
            self._levett_db += 1

        if not self._malom_kar_kepbol_ki(j4, j5):
            return None
        return self._tabla_egyezesre_var(motor)

    def _ember_lep(self, motor):
        """
        Megvárja az ember szabályos lépését (és ha malmot rakott, a levételt is). A kamera minden új stabil
        helyzetét összeveti a motor szabályos lépéseivel. Visszatér: a kamera állapota vagy None (QSTOP).
        """
        self._malom_uzenet(f"Te jössz: {motor.ember_feladata()}.")
        jelzett = None
        while True:
            if self.eteto._stop.is_set():
                return None
            try:
                allapot = self.tabla_uzenetek.get(timeout=0.25)
            except queue.Empty:
                continue
            if allapot is None or allapot[:24] == motor.tabla():
                continue                                     # nincs változás a táblán
            lepesek = motor.ember_lepesei(allapot[:24])
            if lepesek is None:
                elteres = ", ".join(f"{p}: '{motor.tabla()[p]}' -> '{allapot[p]}'"
                                    for p in range(24) if allapot[p] != motor.tabla()[p])
                if elteres != jelzett:
                    jelzett = elteres
                    self._malom_uzenet(f"Ez nem szabályos lépés ({elteres}). Feladat: {motor.ember_feladata()}.")
                continue
            for lepes in lepesek:
                self._malom_uzenet(f"Lépésed: {motor.lepes_szovege(lepes)}.")
                motor.lep(lepes)
            if motor.vege() or motor.robot_jon():
                return allapot
            # malmot rakott, de még nem vette le a bábut
            self._malom_uzenet(f"Malom! Most {motor.ember_feladata()}.")
            jelzett = None

    def _malom_kezdes(self, j4, j5, nehezseg="kozepes"):
        """Malom: kezdés (ki kezd?), majd a játék a malom motorral (OpenSpiel) a végéig."""
        global malom_motor
        try:
            try:
                import malom_motor                           # OpenSpiel kell hozzá (pip install open-spiel)
            except ImportError as e:
                self._malom_uzenet(f"A malom motor nem tölthető be ({e}). Telepítés: pip install open-spiel", True)
                return
            self._malom_uzenet(f"Új játék ({malom_motor.NEHEZSEGEK[nehezseg]['nev']} szint). "
                               f"A kar kiáll a kamera látószögéből...")
            if not self._malom_kar_kepbol_ki(j4, j5):
                self._malom_uzenet("Megszakítva: a kar nem tudott kiállni a képből.", True)
                return
            # a kar kimozdulása után a kamerának idő kell, hogy a tábla állapota újra stabil legyen
            if self.eteto._stop.wait(2.0):
                return
            alap = self._tabla_allapot_lekerese()
            if alap is None:
                self._malom_uzenet("Megszakítva: a kamera nem adott táblaállapotot.", True)
                return
            foglalt = [p for p in range(24) if alap[p] != "."]
            if foglalt:
                self._malom_uzenet(f"Megszakítva: a tábla nem üres, bábu van ezeken a pontokon: {foglalt}", True)
                return
            robot_babuk = sum(1 for p in malom_tabla.ALSO_TARTALEK if alap[p] == malom_tabla.ROBOT_SZIN)
            if robot_babuk != len(malom_tabla.ALSO_TARTALEK):
                self._malom_uzenet(f"Figyelem: a 33-41 tartalékhelyeken {robot_babuk} zöld bábut látok "
                                   f"{len(malom_tabla.ALSO_TARTALEK)} helyett.")

            # --- ki kezd? ---
            ido = malom_tabla.KEZDES_VARAKOZAS
            self._malom_uzenet(f"Ha te kezdesz, lépj {ido:.0f} másodpercen belül! Különben a robot kezd.")
            # a stabil helyzet a lerakás után TABLA_STABIL_MP-vel (1 s) később jön, ezért kis ráhagyás
            uj, pont = self._ember_lepesre_var(alap, hatarido=time.time() + ido + 2.0)
            if self.eteto._stop.is_set():
                self._malom_uzenet("Megszakítva (QSTOP).")
                return
            if uj is not None:
                self._malom_uzenet(f"Te kezdtél: piros bábu a(z) {pont}. pontra.")
                alap = uj
            else:
                self._malom_uzenet("Nem léptél: a robot kezd.")

            # --- a játék a malom motorral (OpenSpiel): felváltva, a játék végéig ---
            motor = malom_motor.MalomMotor(robot_kezd=(uj is None), nehezseg=nehezseg,
                                           robot_szin=malom_tabla.ROBOT_SZIN, ember_szin=malom_tabla.EMBER_SZIN)
            self._levett_db = 0
            if uj is not None:
                motor.lep(pont)                              # az ember első lerakása
            while not motor.vege():
                if motor.robot_jon():
                    alap = self._robot_lep(motor, alap, j4, j5)
                else:
                    alap = self._ember_lep(motor)
                if alap is None:
                    self._malom_uzenet("Játék megszakítva.")
                    return
            eredmeny = motor.eredmeny()
            szoveg = {"robot": "A robot nyert!", "ember": "Gratulálok, nyertél!",
                      "döntetlen": "Döntetlen."}[eredmeny]
            self._malom_uzenet(f"Vége a játéknak. {szoveg}", True)
        finally:
            self.eteto.lefoglalva = False

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