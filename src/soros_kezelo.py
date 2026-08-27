import serial
import serial.tools.list_ports
import threading
import time

class SorosKezelo:
    def __init__(self, log_callback):
        self.ser = None
        self.running = False
        self.log_callback = log_callback # Függvény a GUI-ba való kiíráshoz

    @staticmethod
    def aktiv_portok_lekerese():
        return [port.device for port in serial.tools.list_ports.comports()]

    def kapcsolodas(self, port):
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.running = True
            
            # Háttérszál indítása az olvasáshoz
            self.read_thread = threading.Thread(target=self._olvasasi_ciklus, daemon=True)
            self.read_thread.start()
            return True
        except Exception as e:
            self.log_callback(f"[HIBA] Nem sikerült kapcsolódni: {str(e)}\n")
            self.ser = None
            return False

    def lecsatlakozas(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.log_callback("[RENDSZER] Kapcsolat lezárva.\n")

    def parancs_kuldes(self, parancs_sztring):
        if self.ser and self.ser.is_open:
            # Biztosítjuk az újsor karaktert a végén
            if not parancs_sztring.endswith('\n'):
                parancs_sztring += '\n'
            self.ser.write(parancs_sztring.encode('utf-8'))

    def _olvasasi_ciklus(self):
        while self.running:
            if self.ser and self.ser.is_open and self.ser.in_waiting:
                try:
                    sor = self.ser.readline().decode('utf-8', errors='ignore')
                    if sor:
                        self.log_callback(sor)
                except Exception:
                    break
            time.sleep(0.02)
