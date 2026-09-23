import threading
import time
import tkinter as tk

class KarAutomatizacio:
    # Munkapontok hengerkoordinátában (r mm, phi fok, z mm)
    # (a korábbi XYZ pontokból átszámolva: cukor X159 Y-77, pohár X-21 Y-97)
    CUKOR_FOLOTT = (176.7, -25.8, -47.0)
    CUKOR_LENT = (176.7, -25.8, -67.0)
    POHAR_FOLOTT = (99.3, -102.2, 23.0)

    FOGO_NYITVA = 85
    FOGO_ZARVA = 110

    def __init__(self, interfesz_app):
        """
        interfesz_app: A RobotkarAlkalmazas (kar_interfesz.py) fő példánya.
        """
        self.app = interfesz_app

    def futtat_munkafolyamat(self):
        if self.app.homing_folyamatban:
            self.app.log_erkezett("[FIGYELEM] Homing folyamatban, az automatizáció le van zárva!\n")
            return
        if self.app.eteto.foglalt():
            self.app.log_erkezett("[FIGYELEM] Mozgás folyamatban, az automatizáció nem indítható!\n")
            return
        j4 = int(self.app.gui.j4_entry.get())
        self.app.eteto.uj_mozgas()
        self.app.eteto.lefoglalva = True
        # Háttérszálon fut, hogy a felület (és a QSTOP gomb) közben is működjön
        threading.Thread(target=self._munkafolyamat, args=(j4,), daemon=True).start()

    def _munkafolyamat(self, j4):
        try:
            self._lepesek(j4)
        finally:
            self.app.eteto.lefoglalva = False

    def _lepesek(self, j4):
        app = self.app
        app.log_erkezett("[AUTOMATIZÁCIÓ] Kávécukor-pakolás indítása...\n")

        # 1. FÁZIS: a cukor fölé, majd le a cukorhoz – egy pályán, a fogó NYITVA
        app.log_erkezett("[AUTOMATIZÁCIÓ] 1. Fázis: Mozgás a cukor fölé és lesüllyedés...\n")
        if not app.mozgas_utvonalon([self.CUKOR_FOLOTT, self.CUKOR_LENT], j4, self.FOGO_NYITVA, blokkolo=True):
            app.log_erkezett("[AUTOMATIZÁCIÓ] Megszakítva.\n")
            return

        # MEGFOGÁS: helyben állva zárjuk a fogót
        app.log_erkezett("[AUTOMATIZÁCIÓ] Megállás lent: Fogó zárása...\n")
        if not self._fogo(j4, self.FOGO_ZARVA) or self._var(0.3):  # 0.3 s a biztos megfogásra
            app.log_erkezett("[AUTOMATIZÁCIÓ] Megszakítva.\n")
            return

        # 2. FÁZIS: fel a cukor fölé, és lekerekített ívben megállás nélkül tovább a pohár fölé
        app.log_erkezett("[AUTOMATIZÁCIÓ] 2. Fázis: Felemelés és átvonulás a pohár fölé...\n")
        if not app.mozgas_utvonalon([self.CUKOR_FOLOTT, self.POHAR_FOLOTT], j4, self.FOGO_ZARVA, blokkolo=True):
            app.log_erkezett("[AUTOMATIZÁCIÓ] Megszakítva.\n")
            return

        # BELEDOBÁS
        app.log_erkezett("[AUTOMATIZÁCIÓ] Megállás a pohár felett: Fogó nyitása, cukor bedobva!\n")
        if not self._fogo(j4, self.FOGO_NYITVA) or self._var(0.3):  # idő a fogónak a teljes kinyílásra
            app.log_erkezett("[AUTOMATIZÁCIÓ] Megszakítva.\n")
            return

        app.log_erkezett("[AUTOMATIZÁCIÓ] Gördülékeny munkafolyamat sikeresen végrehajtva!\n")

    def _fogo(self, j4, j5):
        """Csak a fogót mozgatja: a léptetőmotorok célja a jelenlegi pozíció marad."""
        app = self.app
        steps = app.henger.henger_to_lepes(app.aktualis_r, app.aktualis_phi, app.aktualis_z)
        if steps is None:
            return False
        j1, j2, j3 = steps
        if not app.eteto.kuld(f"MOVE {j1} {j2} {j3} {j4} {j5}"):
            return False
        app.root.after(0, lambda: self._gui_fogo_frissites(j5))
        return True

    def _var(self, mp):
        """Várakozás, ami QSTOP-ra azonnal megszakad. True = megszakítva."""
        return self.app.eteto._stop.wait(mp)

    def _gui_fogo_frissites(self, j5):
        self.app.gui.j5_entry.delete(0, tk.END); self.app.gui.j5_entry.insert(0, str(j5))