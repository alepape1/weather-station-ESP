#!/usr/bin/env python3
"""
Aquantia Flash Tool
Compila y flashea el firmware ESP32 por USB serie u OTA.
Requiere: Python 3.8+, pyserial (pip install pyserial)
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import os
import sys
import glob
import shutil

# ── Rutas ─────────────────────────────────────────────────────────────────────

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
SKETCH_DIR  = os.path.join(REPO_ROOT, "ESP_monitor_server")
BUILD_DIR   = os.path.join(os.environ.get("TEMP", "/tmp"), "aquantia_build")

ARDUINO_CLI_CANDIDATES = [
    r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
    r"C:\Program Files (x86)\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
    "arduino-cli",
]

ESPOTA_GLOB_PATTERNS = [
    os.path.expanduser(
        r"~\AppData\Local\Arduino15\packages\esp32\hardware\esp32\*\tools\espota.py"
    ),
    "/root/.arduino15/packages/esp32/hardware/esp32/*/tools/espota.py",
    "/home/*/.arduino15/packages/esp32/hardware/esp32/*/tools/espota.py",
]

FQBN = "esp32:esp32:esp32"

PROFILES = {
    "METEO  — 1 relay  (pantalla TFT)": "1",
    "IRRIGATION — 4 relays (sin pantalla)": "2",
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def find_arduino_cli():
    for path in ARDUINO_CLI_CANDIDATES:
        if os.path.isfile(path):
            return path
        found = shutil.which(path)
        if found:
            return found
    return None


def find_espota():
    for pattern in ESPOTA_GLOB_PATTERNS:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[-1]  # versión más reciente
    return shutil.which("espota.py") or shutil.which("espota")


def list_com_ports():
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except ImportError:
        pass
    # Fallback Windows: probar COM1..COM20
    if sys.platform == "win32":
        import serial
        ports = []
        for i in range(1, 21):
            try:
                s = serial.Serial(f"COM{i}")
                s.close()
                ports.append(f"COM{i}")
            except Exception:
                pass
        return ports
    # Fallback Unix
    return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")


# ── Aplicación ────────────────────────────────────────────────────────────────

class FlasherApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Aquantia Flash Tool")
        self.resizable(False, False)
        self._build_ui()
        self._arduino_cli = find_arduino_cli()
        self._espota       = find_espota()
        self._busy         = False
        self._bin_path     = None
        self._check_tools()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        PAD = dict(padx=10, pady=4)

        # ── Cabecera ──
        hdr = tk.Frame(self, bg="#0c8ecc")
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew")
        tk.Label(hdr, text="💧  Aquantia Flash Tool",
                 bg="#0c8ecc", fg="white",
                 font=("Segoe UI", 14, "bold"),
                 pady=10).pack()

        # ── Perfil ──
        tk.Label(self, text="Perfil de dispositivo:",
                 font=("Segoe UI", 9, "bold")).grid(row=1, column=0, sticky="w", **PAD)
        self._profile_var = tk.StringVar(value=list(PROFILES.keys())[0])
        ttk.Combobox(self, textvariable=self._profile_var,
                     values=list(PROFILES.keys()),
                     state="readonly", width=38).grid(row=1, column=1, sticky="ew", **PAD)

        # ── Puerto COM ──
        tk.Label(self, text="Puerto serie (USB):",
                 font=("Segoe UI", 9, "bold")).grid(row=2, column=0, sticky="w", **PAD)
        port_frame = tk.Frame(self)
        port_frame.grid(row=2, column=1, sticky="ew", **PAD)
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(port_frame, textvariable=self._port_var,
                                      state="readonly", width=28)
        self._port_cb.pack(side="left")
        ttk.Button(port_frame, text="⟳", width=3,
                   command=self._refresh_ports).pack(side="left", padx=4)
        self._refresh_ports()

        # ── IP OTA ──
        tk.Label(self, text="IP del dispositivo (OTA):",
                 font=("Segoe UI", 9, "bold")).grid(row=3, column=0, sticky="w", **PAD)
        self._ip_var = tk.StringVar(value="")
        ttk.Entry(self, textvariable=self._ip_var, width=40).grid(
            row=3, column=1, sticky="ew", **PAD)

        # ── Botones ──
        btn_frame = tk.Frame(self)
        btn_frame.grid(row=4, column=0, columnspan=2, pady=8)

        self._btn_compile = ttk.Button(btn_frame, text="⚙  Compilar",
                                       width=16, command=self._compile)
        self._btn_compile.grid(row=0, column=0, padx=6)

        self._btn_serial = ttk.Button(btn_frame, text="🔌  Flash USB",
                                      width=16, command=self._flash_serial)
        self._btn_serial.grid(row=0, column=1, padx=6)

        self._btn_ota = ttk.Button(btn_frame, text="📡  Flash OTA",
                                   width=16, command=self._flash_ota)
        self._btn_ota.grid(row=0, column=2, padx=6)

        # ── Estado ──
        self._status_var = tk.StringVar(value="Listo")
        tk.Label(self, textvariable=self._status_var,
                 fg="#555", font=("Segoe UI", 8, "italic")).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=10)

        # ── Log ──
        tk.Label(self, text="Log de salida:",
                 font=("Segoe UI", 9, "bold")).grid(
            row=6, column=0, columnspan=2, sticky="w", padx=10, pady=(6, 0))
        self._log = scrolledtext.ScrolledText(
            self, height=18, width=72,
            bg="#1e1e1e", fg="#d4d4d4",
            font=("Consolas", 9),
            state="disabled")
        self._log.grid(row=7, column=0, columnspan=2, padx=10, pady=(0, 10))

        self.columnconfigure(1, weight=1)

    # ── Helpers de UI ─────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = list_com_ports()
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _log_write(self, text, tag=None):
        self._log.config(state="normal")
        self._log.insert("end", text)
        if tag:
            start = self._log.index("end-1l linestart")
            self._log.tag_add(tag, start, "end-1c")
        self._log.see("end")
        self._log.config(state="disabled")

    def _log_line(self, text, color=None):
        self._log.config(state="normal")
        tag = None
        if color:
            tag = f"col_{color}"
            self._log.tag_config(tag, foreground=color)
        self._log.insert("end", text + "\n", tag or "")
        self._log.see("end")
        self._log.config(state="disabled")

    def _clear_log(self):
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        self._log.config(state="disabled")

    def _set_status(self, msg):
        self._status_var.set(msg)
        self.update_idletasks()

    def _set_busy(self, busy):
        self._busy = busy
        state = "disabled" if busy else "normal"
        for btn in (self._btn_compile, self._btn_serial, self._btn_ota):
            btn.config(state=state)

    def _check_tools(self):
        if not self._arduino_cli:
            self._log_line("⚠  arduino-cli no encontrado. Instala Arduino IDE 2.x.", "#f0a000")
        else:
            self._log_line(f"✓  arduino-cli: {self._arduino_cli}", "#4ec9b0")

        if not self._espota:
            self._log_line("⚠  espota.py no encontrado (OTA no disponible).", "#f0a000")
        else:
            self._log_line(f"✓  espota.py: {self._espota}", "#4ec9b0")

        self._log_line(f"✓  Sketch:  {SKETCH_DIR}", "#4ec9b0")
        self._log_line(f"✓  Build:   {BUILD_DIR}", "#4ec9b0")
        self._log_line("")

    # ── Ejecución de subprocesos ───────────────────────────────────────────────

    def _run_cmd(self, cmd, cwd=None):
        """Ejecuta comando y vuelca stdout/stderr al log en tiempo real. Retorna (ok, returncode)."""
        self._log_line(f"$ {' '.join(cmd)}", "#569cd6")
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace",
                cwd=cwd or SKETCH_DIR
            )
            for line in proc.stdout:
                line = line.rstrip("\n")
                color = "#f44747" if ("error:" in line.lower() or "fatal" in line.lower()) \
                        else "#ce9178" if "warning:" in line.lower() \
                        else None
                self._log_line(line, color)
            proc.wait()
            return proc.returncode == 0, proc.returncode
        except FileNotFoundError as e:
            self._log_line(f"ERROR: {e}", "#f44747")
            return False, -1

    # ── Acciones ──────────────────────────────────────────────────────────────

    def _get_profile_flag(self):
        key = self._profile_var.get()
        return PROFILES[key]

    def _do_compile(self):
        """Compila el sketch. Retorna True si OK."""
        if not self._arduino_cli:
            messagebox.showerror("Error", "arduino-cli no encontrado.")
            return False

        profile = self._get_profile_flag()
        os.makedirs(BUILD_DIR, exist_ok=True)

        self._log_line(f"\n{'─'*60}", "#444")
        self._log_line(f"  COMPILANDO  — perfil {profile}", "#dcdcaa")
        self._log_line(f"{'─'*60}\n", "#444")
        self._set_status("Compilando...")

        cmd = [
            self._arduino_cli, "compile",
            "--fqbn", FQBN,
            "--build-property", f"build.extra_flags=-DDEVICE_PROFILE={profile}",
            "--build-path", BUILD_DIR,
            SKETCH_DIR,
        ]
        ok, _ = self._run_cmd(cmd)
        if ok:
            self._bin_path = os.path.join(BUILD_DIR, "ESP_monitor_server.ino.bin")
            self._log_line(f"\n✓  Compilación OK → {self._bin_path}\n", "#4ec9b0")
            self._set_status("Compilación OK")
        else:
            self._log_line("\n✗  Compilación fallida.\n", "#f44747")
            self._set_status("Error de compilación")
            self._bin_path = None
        return ok

    def _compile(self):
        if self._busy:
            return
        self._clear_log()
        self._set_busy(True)
        def run():
            self._do_compile()
            self._set_busy(False)
        threading.Thread(target=run, daemon=True).start()

    def _flash_serial(self):
        if self._busy:
            return
        port = self._port_var.get()
        if not port:
            messagebox.showwarning("Puerto requerido", "Selecciona un puerto COM.")
            return
        self._clear_log()
        self._set_busy(True)

        def run():
            ok = self._do_compile()
            if not ok:
                self._set_busy(False)
                return

            self._log_line(f"\n{'─'*60}", "#444")
            self._log_line(f"  FLASH USB  — {port}", "#dcdcaa")
            self._log_line(f"{'─'*60}", "#444")
            self._log_line("  ⚡  Asegúrate de haber entrado en modo bootloader:", "#f0a000")
            self._log_line("      1. Mantén pulsado GPIO0 (BOOT)", "#f0a000")
            self._log_line("      2. Pulsa y suelta EN/RST", "#f0a000")
            self._log_line("      3. Suelta GPIO0\n", "#f0a000")
            self._set_status(f"Flasheando por USB en {port}...")

            cmd = [
                self._arduino_cli, "upload",
                "--fqbn", FQBN,
                "--port", port,
                "--input-dir", BUILD_DIR,
                SKETCH_DIR,
            ]
            ok, _ = self._run_cmd(cmd)
            if ok:
                self._log_line("\n✓  Flash USB completado.\n", "#4ec9b0")
                self._set_status("Flash USB completado")
            else:
                self._log_line("\n✗  Flash USB fallido.\n", "#f44747")
                self._set_status("Error en flash USB")
            self._set_busy(False)

        threading.Thread(target=run, daemon=True).start()

    def _flash_ota(self):
        if self._busy:
            return
        ip = self._ip_var.get().strip()
        if not ip:
            messagebox.showwarning("IP requerida", "Introduce la IP del dispositivo.")
            return
        if not self._espota:
            messagebox.showerror("Error", "espota.py no encontrado.\nInstala el core ESP32 de Arduino.")
            return
        self._clear_log()
        self._set_busy(True)

        def run():
            ok = self._do_compile()
            if not ok:
                self._set_busy(False)
                return

            self._log_line(f"\n{'─'*60}", "#444")
            self._log_line(f"  FLASH OTA  — {ip}:3232", "#dcdcaa")
            self._log_line(f"{'─'*60}\n", "#444")
            self._set_status(f"Flasheando por OTA a {ip}...")

            # espota.py o espota (ejecutable compilado)
            if self._espota.endswith(".py"):
                cmd = [sys.executable, self._espota,
                       "-i", ip, "-p", "3232", "-f", self._bin_path]
            else:
                cmd = [self._espota,
                       "-i", ip, "-p", "3232", "-f", self._bin_path]

            ok, _ = self._run_cmd(cmd)
            if ok:
                self._log_line("\n✓  Flash OTA completado.\n", "#4ec9b0")
                self._set_status("Flash OTA completado")
            else:
                self._log_line("\n✗  Flash OTA fallido.\n", "#f44747")
                self._set_status("Error en flash OTA")
            self._set_busy(False)

        threading.Thread(target=run, daemon=True).start()


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = FlasherApp()
    app.mainloop()
