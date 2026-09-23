import tkinter as tk
from tkinter import ttk

class FelhasznaloiFelulet:
    def __init__(self, root, on_connect, on_home, on_send, on_jog, on_send_henger, on_jog_henger, on_qstop, on_auto_sequence):
        self.root = root
        self.root.title("Arduino Robotkar Vezérlés (5-Axis UI)")
        self.root.geometry("580x950")  
        self.root.configure(bg="#1e272e")
        
        self.on_connect = on_connect
        self.on_home = on_home
        self.on_send = on_send
        self.on_jog = on_jog
        self.on_send_henger = on_send_henger
        self.on_jog_henger = on_jog_henger
        self.on_qstop = on_qstop
        self.on_auto_sequence = on_auto_sequence

        self._stilusok_beallitasa()
        self._panelek_letrehozasa()
        self.set_controls_state("disabled")

    def _stilusok_beallitasa(self):
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('.', background='#1e272e', foreground='#ffffff')
        self.style.configure('TLabel', background='#1e272e', foreground='#ffffff', font=('Helvetica', 10, 'bold'))
        self.style.configure('TButton', font=('Helvetica', 10, 'bold'), background='#3d3d3d', foreground='#ffffff')
        self.style.map('TButton', background=[('active', '#05c46b')])
        
        self.style.configure('Home.TButton', background='#ffdd59', foreground='#000000', font=('Helvetica', 11, 'bold'))
        self.style.map('Home.TButton', background=[('active', '#ffd32a')])
        
        self.style.configure('Send.TButton', background='#0be881', foreground='#000000', font=('Helvetica', 11, 'bold'))
        self.style.map('Send.TButton', background=[('active', '#05c46b')])

        self.style.configure('XYZ.TButton', background='#ffa801', foreground='#000000', font=('Helvetica', 11, 'bold'))
        self.style.map('XYZ.TButton', background=[('active', '#ffc048')])

        self.style.configure('Auto.TButton', background='#9b59b6', foreground='#ffffff', font=('Helvetica', 11, 'bold'))
        self.style.map('Auto.TButton', background=[('active', '#8e44ad')])

        self.style.configure('QStop.TButton', background='#ff3f34', foreground='#ffffff', font=('Helvetica', 11, 'bold'))
        self.style.map('QStop.TButton', background=[('active', '#ee5253')])

        self.style.configure('JogNeg.TButton', background='#ea2027', foreground='#ffffff', font=('Helvetica', 9, 'bold'))
        self.style.map('JogNeg.TButton', background=[('active', '#ff3f34')])
        self.style.configure('JogPos.TButton', background='#0652dd', foreground='#ffffff', font=('Helvetica', 9, 'bold'))
        self.style.map('JogPos.TButton', background=[('active', '#12cbc4')])

    def _panelek_letrehozasa(self):
        # 1. Kapcsolódás panel
        conn_frame = tk.LabelFrame(self.root, text=" 🔌 Kapcsolódás ", bg="#1e272e", fg="#0be881", font=('Helvetica', 10, 'bold'), bd=2)
        conn_frame.pack(fill="x", padx=15, pady=8)
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.port_combobox = ttk.Combobox(conn_frame, width=12, font=('Helvetica', 10))
        self.port_combobox.grid(row=0, column=1, padx=5, pady=10)
        self.connect_btn = ttk.Button(conn_frame, text="Csatlakozás", command=self.on_connect)
        self.connect_btn.grid(row=0, column=2, padx=10, pady=10)

        # 2. Kalibráció panel
        self.home_frame = tk.LabelFrame(self.root, text=" 🏠 Kalibráció ", bg="#1e272e", fg="#0be881", font=('Helvetica', 10, 'bold'), bd=2)
        self.home_frame.pack(fill="x", padx=15, pady=5)
        self.home_btn = ttk.Button(self.home_frame, text="🤖 HOMING (Nullázás) INDÍTÁSA", style="Home.TButton", command=self.on_home)
        self.home_btn.pack(fill="x", padx=15, pady=10)
        # 3. Pozíció és léptető panel (J1-J5 gombok)
        self.move_frame = tk.LabelFrame(self.root, text=" 🕹️ Pozíció Vezérlés (Lépések és Fokok) ", bg="#1e272e", fg="#0be881", font=('Helvetica', 10, 'bold'), bd=2)
        self.move_frame.pack(fill="x", padx=15, pady=5)
        
        self.j1_entry = self._motor_sor_letrehozasa("J1 (X):", 0, "0")
        self.j2_entry = self._motor_sor_letrehozasa("J2 (E):", 1, "0")
        self.j3_entry = self._motor_sor_letrehozasa("J3 (Y):", 2, "0")
        self.j4_entry = self._motor_sor_letrehozasa("J4 (Servo):", 3, "90") 
        self.j5_entry = self._motor_sor_letrehozasa("J5 (Claw):", 4, "90") 
        
        self.send_btn = ttk.Button(self.move_frame, text="🚀 Tengelyek Küldése (Azonnali MOVE)", style="Send.TButton", command=self.on_send)
        self.send_btn.grid(row=5, column=1, columnspan=5, pady=10, sticky="ew")

        # 4. Térbeli koordináta panel (hengerkoordináták)
        self.xyz_frame = tk.LabelFrame(self.root, text=" 📐 Hengerkoordináta Pozíció (R mm, φ fok, Z mm) ", bg="#1e272e", fg="#ffa801", font=('Helvetica', 10, 'bold'), bd=2)
        self.xyz_frame.pack(fill="x", padx=15, pady=5)
        self.r_entry = self._xyz_sor_letrehozasa("R:", 0, "118.0", "mm")
        self.phi_entry = self._xyz_sor_letrehozasa("φ:", 1, "0.0", "°")
        self.z_entry = self._xyz_sor_letrehozasa("Z:", 2, "51.0", "mm")
        self.xyz_send_btn = ttk.Button(self.xyz_frame, text="🎯 Pozícióra Küldés (Útvonaltervezéssel)", style="XYZ.TButton", command=self.on_send_henger)
        self.xyz_send_btn.grid(row=3, column=1, columnspan=5, pady=10, sticky="ew")

        # 5. Queue panel (Az automata pakoló folyamat)
        self.queue_frame = tk.LabelFrame(self.root, text=" ⛓️ Queue (Sor) Folyamatos Etetés ", bg="#1e272e", fg="#ffdd59", font=('Helvetica', 10, 'bold'), bd=2)
        self.queue_frame.pack(fill="x", padx=15, pady=5)
        
        self.auto_btn = ttk.Button(self.queue_frame, text="📦 AUTOMATA FOLYAMAT (Pakolás) INDÍTÁSA", style="Auto.TButton", command=self.on_auto_sequence)
        self.auto_btn.pack(fill="x", padx=15, pady=5)
        
        self.qstop_btn = ttk.Button(self.queue_frame, text="🛑 QUEUE VÉSZLEÁLLÍTÁS (QSTOP)", style="QStop.TButton", command=self.on_qstop)
        self.qstop_btn.pack(fill="x", padx=15, pady=5)

        # 6. Monitor panel
        term_frame = tk.LabelFrame(self.root, text=" 🖥️ Soros Monitor ", bg="#1e272e", fg="#0be881", font=('Helvetica', 10, 'bold'), bd=2)
        term_frame.pack(fill="both", expand=True, padx=15, pady=10)
        self.terminal = tk.Text(term_frame, bg="#2f3640", fg="#0be881", font=('Courier New', 9), state="disabled", wrap="word")
        self.terminal.pack(fill="both", expand=True, padx=5, pady=5)

    def _motor_sor_letrehozasa(self, nev, sor_idx, alapert_ert):
        ttk.Label(self.move_frame, text=nev).grid(row=sor_idx, column=0, padx=10, pady=12, sticky="e")
        m_id = nev.split(" ") 
        
        btn_n10 = ttk.Button(self.move_frame, text="-10°", width=5, style="JogNeg.TButton", command=lambda: self.on_jog(m_id, -10))
        btn_n10.grid(row=sor_idx, column=1, padx=2)
        btn_n1 = ttk.Button(self.move_frame, text="-1°", width=4, style="JogNeg.TButton", command=lambda: self.on_jog(m_id, -1))
        btn_n1.grid(row=sor_idx, column=2, padx=2)
        
        entry = tk.Entry(self.move_frame, width=8, font=('Helvetica', 11, 'bold'), bg="#2f3640", fg="#ffdd59", insertbackground="white", bd=2, justify="center")
        entry.insert(0, alapert_ert)
        entry.grid(row=sor_idx, column=3, padx=5)
        
        btn_p1 = ttk.Button(self.move_frame, text="+1°", width=4, style="JogPos.TButton", command=lambda: self.on_jog(m_id, 1))
        btn_p1.grid(row=sor_idx, column=4, padx=2)
        btn_p10 = ttk.Button(self.move_frame, text="+10°", width=5, style="JogPos.TButton", command=lambda: self.on_jog(m_id, 10))
        btn_p10.grid(row=sor_idx, column=5, padx=2)
        
        if not hasattr(self, 'jog_gombok'): self.jog_gombok = []
        self.jog_gombok.extend([btn_n10, btn_n1, btn_p1, btn_p10])
        return entry

    def _xyz_sor_letrehozasa(self, tengely_name, sor_idx, alapert_ert, egyseg):
        ttk.Label(self.xyz_frame, text=tengely_name).grid(row=sor_idx, column=0, padx=15, pady=12, sticky="e")
        t_id = tengely_name.replace(":", "")
        
        btn_n10 = ttk.Button(self.xyz_frame, text=f"-10{egyseg}", width=7, style="JogNeg.TButton", command=lambda: self.on_jog_henger(t_id, -10))
        btn_n10.grid(row=sor_idx, column=1, padx=2)
        btn_n1 = ttk.Button(self.xyz_frame, text=f"-1{egyseg}", width=6, style="JogNeg.TButton", command=lambda: self.on_jog_henger(t_id, -1))
        btn_n1.grid(row=sor_idx, column=2, padx=2)
        
        entry = tk.Entry(self.xyz_frame, width=8, font=('Helvetica', 11, 'bold'), bg="#2f3640", fg="#ffa801", insertbackground="white", bd=2, justify="center")
        entry.insert(0, alapert_ert)
        entry.grid(row=sor_idx, column=3, padx=5)
        
        btn_p1 = ttk.Button(self.xyz_frame, text=f"+1{egyseg}", width=6, style="JogPos.TButton", command=lambda: self.on_jog_henger(t_id, 1))
        btn_p1.grid(row=sor_idx, column=4, padx=2)
        btn_p10 = ttk.Button(self.xyz_frame, text=f"+10{egyseg}", width=7, style="JogPos.TButton", command=lambda: self.on_jog_henger(t_id, 10))
        btn_p10.grid(row=sor_idx, column=5, padx=2)
        
        if not hasattr(self, 'xyz_jog_gombok'): self.xyz_jog_gombok = []
        self.xyz_jog_gombok.extend([btn_n10, btn_n1, btn_p1, btn_p10])
        return entry

    def set_controls_state(self, state):
        self.home_btn.config(state=state)
        self.send_btn.config(state=state)
        self.xyz_send_btn.config(state=state)
        self.auto_btn.config(state=state)
        self.qstop_btn.config(state=state)
        self.j1_entry.config(state=state)
        self.j2_entry.config(state=state)
        self.j3_entry.config(state=state)
        self.j4_entry.config(state=state)
        self.j5_entry.config(state=state)
        self.r_entry.config(state=state)
        self.phi_entry.config(state=state)
        self.z_entry.config(state=state)
        for btn in self.jog_gombok: btn.config(state=state)
        for btn in self.xyz_jog_gombok: btn.config(state=state)

    def log_kiiras(self, szoveg):
        self.terminal.config(state="normal")
        self.terminal.insert(tk.END, szoveg)
        self.terminal.see(tk.END)
        self.terminal.config(state="disabled")

    def portok_frissitese(self, port_lista):
        self.port_combobox['values'] = port_lista
        if port_lista and self.port_combobox.get() == "":
            self.port_combobox.current(0)