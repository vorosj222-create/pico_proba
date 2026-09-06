import time
import tkinter as tk

class KarAutomatizacio:
    def __init__(self, interfesz_app):
        """
        interfesz_app: A RobotkarAlkalmazas (kar_interfesz.py) fő példánya.
        """
        self.app = interfesz_app

    def futtat_munkafolyamat(self):
        if self.app.homing_folyamatban:
            self.app.log_erkezett("[FIGYELEM] Homing folyamatban, az automatizáció le van zárva!\n")
            return

        self.app.log_erkezett("[AUTOMATIZÁCIÓ] Kávécukor-pakolás indítása...\n")
        j4 = int(self.app.gui.j4_entry.get())

        # =====================================================================
        # 1. LÉPÉS: Elmegy a cukor FÖLÉ (X: 159, Y: -77, Z: -47), fogó NYITVA (85°)
        # =====================================================================
        self.app.log_erkezett("[AUTOMATIZÁCIÓ] 1. Fázis: Mozgás a cukor fölé...\n")
        p1_steps = self.app.kinematika.koordinata_szamitas(159.0, -77.0, -47.0)
        
        if p1_steps is not None:
            j1, j2, j3 = p1_steps
            cmd = f"MOVE {j1} {j2} {j3} {j4} 85"
            self.app.soros.parancs_kuldes(cmd)
            self._gui_xyz_frissites(159.0, -77.0, -47.0, 85)
            time.sleep(1.2)  # Megvárjuk, amíg odaér a cukor fölé
        else:
            self.app.log_erkezett("[HIBA] A cukor feletti pont nem érhető el!\n")
            return

        # =====================================================================
        # 2. LÉPÉS: TELJESEN LEMEGY A CUKORHOZ (X: 159, Y: -77, Z: -67), fogó MÉG NYITVA (85°)
        # =====================================================================
        self.app.log_erkezett("[AUTOMATIZÁCIÓ] 2. Fázis: Lesüllyedés a cukorhoz...\n")
        p2_steps = self.app.kinematika.koordinata_szamitas(159.0, -77.0, -67.0)
        
        if p2_steps is not None:
            j1, j2, j3 = p2_steps
            cmd = f"MOVE {j1} {j2} {j3} {j4} 85"
            self.app.soros.parancs_kuldes(cmd)
            self._gui_xyz_frissites(159.0, -77.0, -67.0, 85)
            time.sleep(0.8)  # Idő a fizikai leérkezésre és a teljes megállásra
            
            # === MEGFOGÁS MENTEN ===
            self.app.log_erkezett("[AUTOMATIZÁCIÓ] Megállás lent: Fogó zárása (110°)...\n")
            cmd = f"MOVE {j1} {j2} {j3} {j4} 110"
            self.app.soros.parancs_kuldes(cmd)
            self._gui_xyz_frissites(159.0, -77.0, -67.0, 110)
            time.sleep(0.3)  # A kért 0.3 másodperces fix várakozás a biztos megfogásra
        else:
            self.app.log_erkezett("[HIBA] A cukor fogási pontja nem érhető el!\n")
            return
        # =====================================================================
        # 3. LÉPÉS: FOLYAMATOS MOZGÁS A POHÁR FÖLÉ (X: -21, Y: -97, Z: 23), fogó ZÁRVA (110°)
        # A kar a cukor feletti ponton megállás nélkül, egyetlen ívben suhan át a pohárhoz!
        # =====================================================================
        self.app.log_erkezett("[AUTOMATIZÁCIÓ] 3. Fázis: Megállás nélküli felemelés és átvonulás a pohár fölé...\n")
        p4_steps = self.app.kinematika.koordinata_szamitas(-21.0, -97.0, 23.0)
        
        if p4_steps is not None:
            j1, j2, j3 = p4_steps
            # Közvetlenül a pohár feletti célpontot adjuk meg azonnali MOVE paraccsal
            cmd = f"MOVE {j1} {j2} {j3} {j4} 110"
            self.app.soros.parancs_kuldes(cmd)
            self._gui_xyz_frissites(-21.0, -97.0, 23.0, 110)
            
            # Hagyunk elég időt a fizikai átlendülésre és a pohár feletti teljes megállásra (hosszabb szakasz)
            time.sleep(1.8)
            
            # === BELEDOBÁS ===
            self.app.log_erkezett("[AUTOMATIZÁCIÓ] Megállás a pohár felett: Fogó nyitása (85°), cukor bedobva!\n")
            cmd = f"MOVE {j1} {j2} {j3} {j4} 85"
            self.app.soros.parancs_kuldes(cmd)
            self._gui_xyz_frissites(-21.0, -97.0, 23.0, 85)
            
            # Hagyunk 0.3 másodpercet a fogónak a teljes kinyílásra
            time.sleep(0.3)
        else:
            self.app.log_erkezett("[HIBA] A pohár feletti végpont nem érhető el!\n")
            return

        self.app.log_erkezett("[AUTOMATIZÁCIÓ] Gördülékeny munkafolyamat sikeresen végrehajtva!\n")

    def _gui_xyz_frissites(self, x, y, z, j5):
        """Segédfüggvény a belső memória és a felület szinkronizálására"""
        self.app.aktualis_x = x
        self.app.aktualis_y = y
        self.app.aktualis_z = z
        
        self.app.gui.x_entry.delete(0, tk.END); self.app.gui.x_entry.insert(0, str(x))
        self.app.gui.y_entry.delete(0, tk.END); self.app.gui.y_entry.insert(0, str(y))
        self.app.gui.z_entry.delete(0, tk.END); self.app.gui.z_entry.insert(0, str(z))
        
        self.app.gui.j5_entry.delete(0, tk.END); self.app.gui.j5_entry.insert(0, str(j5))
