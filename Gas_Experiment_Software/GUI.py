"""
Mass Flow Controller GUI
Uses Tkinter to display and control two input gas flows and one output gas flow.
Designed to interface with hardware (e.g., LabJack U12) for MFC control.
"""

import tkinter as tk
from tkinter import filedialog, messagebox
import os
import csv
import time
import threading
from datetime import datetime


# ─────────────────────────────────────────────
#  Application State
# ─────────────────────────────────────────────
class AppState:
    STOPPED  = "stopped"
    RUNNING  = "running"
    PAUSED   = "paused"


# ─────────────────────────────────────────────
#  Main Application
# ─────────────────────────────────────────────
class MFCApp(tk.Frame):
    POLL_INTERVAL_MS = 50    # display refresh rate (~20 Hz); hardware reads run in background
    CSV_INTERVAL_S   = 1.0   # write one CSV row per second

    # Button colors: (normal_bg, active_bg, dim_bg)
    _C_START = ("#388e3c", "#43a047", "#4a7c4e")
    _C_PAUSE = ("#f57f17", "#fb8c00", "#b87020")
    _C_STOP  = ("#c62828", "#e53935", "#963030")
    _FG      = "#e0e0e0"
    _DIM_FG  = "#aaaaaa"

    def __init__(self, master, device):
        super().__init__(master)
        self.pack(fill=tk.BOTH, expand=True)

        master.title("Mass Flow Controller")
        master.resizable(False, False)
        master.configure(bg="#2b2b2b")

        # ── Internal state ──────────────────
        self.device       = device
        self.state        = AppState.STOPPED
        self.save_file    = None
        self.csv_writer   = None
        self.csv_fh       = None
        self._poll_job       = None
        self._clock_job      = None
        self._status_text    = "Stopped"
        self._last_csv_write = 0.0

        # Background reader state
        self._latest_reading = (0.0, 0.0, 0.0)
        self._read_lock      = threading.Lock()
        self._stop_reader    = threading.Event()
        self._reader_thread  = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # ── Flow StringVars (updated by poll loop) ──
        self.flow1_var    = tk.StringVar(value="---")
        self.flow2_var    = tk.StringVar(value="---")
        self.flow_out_var = tk.StringVar(value="---")

        self._build_ui()
        master.protocol("WM_DELETE_WINDOW", self._on_exit)
        self._poll_flows()  # always-on display loop
        self._tick_clock()  # always-on status clock

    # ─────────────────────────────────────────
    #  UI Construction
    # ─────────────────────────────────────────
    def _build_ui(self):
        PAD    = 12
        BG     = "#2b2b2b"
        PANEL  = "#3c3f41"
        FG     = self._FG
        ACCENT = "#4fc3f7"

        # ── Top button bar ───────────────────
        btn_frame = tk.Frame(self, bg=BG, pady=PAD)
        btn_frame.pack(fill=tk.X, padx=PAD)

        btn_cfg = dict(font=("Segoe UI", 10, "bold"), width=10,
                       relief=tk.FLAT, cursor="hand2", padx=6, pady=5)

        self.btn_save = tk.Button(btn_frame, text="💾  Save File",
                                  bg="#546e7a", fg=FG, activebackground="#607d8b",
                                  command=self._on_save_file, **btn_cfg)
        self.btn_save.pack(side=tk.LEFT, padx=(0, 6))

        # Start disabled until a save file is selected — use dim color
        self.btn_start = tk.Button(btn_frame, text="▶  Start",
                                   bg=self._C_START[2], fg=FG,
                                   activebackground=self._C_START[1],
                                   disabledforeground=self._DIM_FG,
                                   command=self._on_start, state=tk.DISABLED, **btn_cfg)
        self.btn_start.pack(side=tk.LEFT, padx=6)

        self.btn_pause = tk.Button(btn_frame, text="⏸  Pause",
                                   bg=self._C_PAUSE[2], fg=FG,
                                   activebackground=self._C_PAUSE[1],
                                   disabledforeground=self._DIM_FG,
                                   command=self._on_pause, state=tk.DISABLED, **btn_cfg)
        self.btn_pause.pack(side=tk.LEFT, padx=6)

        self.btn_stop = tk.Button(btn_frame, text="⏹  Stop",
                                  bg=self._C_STOP[2], fg=FG,
                                  activebackground=self._C_STOP[1],
                                  disabledforeground=self._DIM_FG,
                                  command=self._on_stop, state=tk.DISABLED, **btn_cfg)
        self.btn_stop.pack(side=tk.LEFT, padx=6)

        self.btn_exit = tk.Button(btn_frame, text="✕  Exit",
                                  bg="#4a148c", fg=FG, activebackground="#6a1b9a",
                                  command=self._on_exit, **btn_cfg)
        self.btn_exit.pack(side=tk.RIGHT, padx=(6, 0))

        # ── Filename label (shows selected file near Save button) ──
        self.savefile_label = tk.Label(btn_frame, text="", bg=BG, fg="#aaaaaa",
                                       font=("Segoe UI", 9))
        self.savefile_label.pack(side=tk.LEFT, padx=12)

        # ── Status bar ───────────────────────
        self.status_var = tk.StringVar(value="Status: Stopped")
        status_bar = tk.Label(self, textvariable=self.status_var,
                              bg="#1c1c1c", fg="#aaaaaa",
                              font=("Segoe UI", 9), anchor=tk.W, pady=4)
        status_bar.pack(fill=tk.X, padx=0)

        # ── Flow display area ────────────────
        flow_frame = tk.Frame(self, bg=BG, padx=PAD, pady=PAD)
        flow_frame.pack(fill=tk.BOTH, expand=True)

        # Input Gas 1
        self._make_flow_panel(flow_frame, col=0,
                              label="Input Gas (1) Flow",
                              var=self.flow1_var,
                              accent=ACCENT, bg=PANEL, fg=FG)

        # Input Gas 2
        self._make_flow_panel(flow_frame, col=1,
                              label="Input Gas (2) Flow",
                              var=self.flow2_var,
                              accent=ACCENT, bg=PANEL, fg=FG)

        # Vertical separator
        sep = tk.Frame(flow_frame, width=3, bg="#555555")
        sep.grid(row=0, column=2, sticky="ns", padx=8)

        # Output Gas
        self._make_flow_panel(flow_frame, col=3,
                              label="Output Gas Flow",
                              var=self.flow_out_var,
                              accent="#a5d6a7", bg=PANEL, fg=FG)

        # ── Save-file path label ─────────────
        self.filepath_var = tk.StringVar(value="No save file selected.")
        fp_label = tk.Label(self, textvariable=self.filepath_var,
                            bg="#1c1c1c", fg="#777777",
                            font=("Segoe UI", 8), anchor=tk.W, pady=3)
        fp_label.pack(fill=tk.X, padx=0)

    def _make_flow_panel(self, parent, col, label, var, accent, bg, fg):
        """Create a labelled flow display panel in a given column."""
        frame = tk.Frame(parent, bg=bg, bd=0,
                         highlightbackground="#555555", highlightthickness=1)
        frame.grid(row=0, column=col, padx=(0 if col == 0 else 4), pady=4,
                   ipadx=14, ipady=14, sticky="nsew")
        parent.columnconfigure(col, weight=1)

        tk.Label(frame, text=label, bg=bg, fg=accent,
                 font=("Segoe UI", 11, "bold"), pady=8).pack()

        tk.Label(frame, textvariable=var, bg=bg, fg=fg,
                 font=("Courier New", 20, "bold"),
                 relief=tk.SUNKEN, bd=2, width=14,
                 pady=10, padx=8).pack(padx=10, pady=(0, 10))

    # ─────────────────────────────────────────
    #  Button Handlers
    # ─────────────────────────────────────────
    def _on_save_file(self):
        path = filedialog.asksaveasfilename(
            title="Choose save file",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if path:
            self.save_file = path
            self.filepath_var.set(f"Save file: {path}")
            self.savefile_label.config(text=os.path.basename(path))
            self.btn_start.config(state=tk.NORMAL,
                                  bg=self._C_START[0], fg=self._FG)

    def _on_start(self):
        if self.state == AppState.RUNNING:
            return
        if self.save_file:
            self._open_csv()
        self._last_csv_write = 0.0  # write first row immediately
        self.state = AppState.RUNNING
        self._update_status("Running")
        self.btn_save.config(state=tk.DISABLED)
        self.btn_start.config(state=tk.DISABLED,
                              bg=self._C_START[2],
                              disabledforeground=self._DIM_FG)
        self.btn_pause.config(state=tk.NORMAL, text="⏸  Pause",
                              bg=self._C_PAUSE[0], fg=self._FG)
        self.btn_stop.config(state=tk.NORMAL,
                             bg=self._C_STOP[0], fg=self._FG)

    def _on_pause(self):
        if self.state == AppState.RUNNING:
            self.state = AppState.PAUSED
            self._update_status("Paused")
            self.btn_pause.config(text="▶  Resume")
        elif self.state == AppState.PAUSED:
            self.state = AppState.RUNNING
            self._update_status("Running")
            self.btn_pause.config(text="⏸  Pause")

    def _on_stop(self):
        self.state = AppState.STOPPED
        self._close_csv()
        self._update_status("Stopped")
        self.btn_save.config(state=tk.NORMAL)
        self.btn_start.config(state=tk.DISABLED,
                              bg=self._C_START[2],
                              disabledforeground=self._DIM_FG)
        self.btn_pause.config(state=tk.DISABLED, text="⏸  Pause",
                              bg=self._C_PAUSE[2],
                              disabledforeground=self._DIM_FG)
        self.btn_stop.config(state=tk.DISABLED,
                             bg=self._C_STOP[2],
                             disabledforeground=self._DIM_FG)

    def _on_exit(self):
        self._stop_reader.set()
        if self._poll_job:
            self.after_cancel(self._poll_job)
        if self._clock_job:
            self.after_cancel(self._clock_job)
        self._close_csv()
        self.master.destroy()

    # ─────────────────────────────────────────
    #  Hardware Read & Poll Loop
    # ─────────────────────────────────────────
    def _reader_loop(self):
        """Background thread: reads hardware as fast as possible (~60 ms/sample)."""
        while not self._stop_reader.is_set():
            try:
                v0 = self.device.eAnalogIn(channel=0)['voltage']
                v1 = self.device.eAnalogIn(channel=1)['voltage']
                v2 = self.device.eAnalogIn(channel=2)['voltage']
                with self._read_lock:
                    self._latest_reading = (v0, v1, v2)
            except Exception:
                pass

    def _poll_flows(self):
        """Main-thread loop — updates display at POLL_INTERVAL_MS, writes CSV once/sec."""
        with self._read_lock:
            v0, v1, v2 = self._latest_reading

        self.flow1_var.set(f"{v0:>8.4f} V")
        self.flow2_var.set(f"{v1:>8.4f} V")
        self.flow_out_var.set(f"{v2:>8.4f} V")

        if self.state == AppState.RUNNING and self.csv_writer:
            now = time.monotonic()
            if now - self._last_csv_write >= self.CSV_INTERVAL_S:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                self.csv_writer.writerow([ts, v0, v1, v2])
                self.csv_fh.flush()
                self._last_csv_write = now

        self._poll_job = self.after(self.POLL_INTERVAL_MS, self._poll_flows)

    # ─────────────────────────────────────────
    #  CSV Logging
    # ─────────────────────────────────────────
    def _open_csv(self):
        try:
            self.csv_fh = open(self.save_file, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow(
                ["Timestamp", "Voltage_AI0_V", "Voltage_AI1_V", "Voltage_AI2_V"]
            )
        except OSError as e:
            messagebox.showerror("File Error", f"Could not open save file:\n{e}")
            self.csv_writer = None
            self.csv_fh = None

    def _close_csv(self):
        if self.csv_fh:
            try:
                self.csv_fh.close()
            except OSError:
                pass
        self.csv_writer = None
        self.csv_fh = None

    # ─────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────
    def _update_status(self, text):
        self._status_text = text
        self.status_var.set(f"Status: {text}  —  {datetime.now().strftime('%H:%M:%S')}")

    def _tick_clock(self):
        """Updates the time in the status bar every second regardless of state."""
        self.status_var.set(
            f"Status: {self._status_text}  —  {datetime.now().strftime('%H:%M:%S')}"
        )
        self._clock_job = self.after(1000, self._tick_clock)
