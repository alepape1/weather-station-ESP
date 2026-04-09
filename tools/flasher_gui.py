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
import json
import datetime
import tempfile

# ── Rutas ─────────────────────────────────────────────────────────────────────

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT    = os.path.dirname(SCRIPT_DIR)
SKETCH_NAME  = "ESP_monitor_server"
SKETCH_DIR   = os.path.join(REPO_ROOT, SKETCH_NAME)
BUILD_DIR    = os.path.join(os.environ.get("TEMP", "/tmp"), "aquantia_build")
METADATA_FILE = os.path.join(BUILD_DIR, "build_meta.json")

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

# ── Git helpers ───────────────────────────────────────────────────────────────

def _git(*args, cwd=None):
    """Ejecuta un comando git y devuelve stdout, o '' si falla."""
    try:
        r = subprocess.run(
            ["git"] + list(args),
            cwd=cwd or REPO_ROOT,
            capture_output=True, text=True, encoding="utf-8", errors="replace"
        )
        return r.stdout.strip() if r.returncode == 0 else ""
    except FileNotFoundError:
        return ""


def get_current_version():
    """Devuelve (hash_corto, label, is_dirty)."""
    h   = _git("rev-parse", "--short", "HEAD") or "?"
    dirty = bool(_git("status", "--porcelain"))
    desc = _git("describe", "--tags", "--always", "--dirty") or h
    return h, desc, dirty


def get_version_list():
    """Lista de (label_display, git_ref | None) para el selector de versión."""
    versions = []

    h, desc, dirty = get_current_version()
    label = f"Actual — {desc}" + (" (cambios sin commit)" if dirty else "")
    versions.append((label, None))          # None = working copy

    # Últimas 5 etiquetas
    tags = _git("tag", "--sort=-creatordate").splitlines()
    for tag in tags[:5]:
        if tag:
            versions.append((f"Tag: {tag}", tag))

    # Últimos 10 commits
    log = _git("log", "--oneline", "-10").splitlines()
    for line in log:
        if line:
            ref  = line[:7]
            msg  = line[8:50]
            versions.append((f"{ref}  —  {msg}", ref))

    return versions


def export_version_to_temp(git_ref):
    """
    Extrae el sketch de un commit/tag a un directorio temporal.
    Devuelve la ruta al sketch extraído o None si falla.
    """
    tmp = tempfile.mkdtemp(prefix="aquantia_ver_")
    archive_cmd = ["git", "archive", git_ref, SKETCH_NAME]
    try:
        archive = subprocess.run(
            archive_cmd, cwd=REPO_ROOT,
            capture_output=True
        )
        if archive.returncode != 0:
            return None
        # Extraer tar en memoria
        import tarfile, io
        with tarfile.open(fileobj=io.BytesIO(archive.stdout)) as tf:
            tf.extractall(tmp)
        extracted = os.path.join(tmp, SKETCH_NAME)
        if os.path.isdir(extracted):
            # Copiar secrets.h desde el working copy (no está en git)
            secrets_src = os.path.join(SKETCH_DIR, "secrets.h")
            if os.path.exists(secrets_src):
                shutil.copy2(secrets_src, extracted)
            return extracted
        return None
    except Exception:
        return None


# ── Herramientas ──────────────────────────────────────────────────────────────

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
            return matches[-1]
    return shutil.which("espota.py") or shutil.which("espota")


def list_com_ports():
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except ImportError:
        pass
    if sys.platform == "win32":
        ports = []
        for i in range(1, 21):
            try:
                import serial
                s = serial.Serial(f"COM{i}"); s.close()
                ports.append(f"COM{i}")
            except Exception:
                pass
        return ports
    return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")


# ── Estado del binario ────────────────────────────────────────────────────────

def load_metadata():
    try:
        with open(METADATA_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return None


def save_metadata(meta):
    os.makedirs(BUILD_DIR, exist_ok=True)
    with open(METADATA_FILE, "w") as f:
        json.dump(meta, f, indent=2)


def source_files_newer_than_binary(bin_path):
    """True si algún archivo fuente es más reciente que el binario."""
    if not os.path.exists(bin_path):
        return True
    bin_mtime = os.path.getmtime(bin_path)
    for fname in os.listdir(SKETCH_DIR):
        if fname.endswith((".ino", ".h", ".cpp", ".c")):
            fpath = os.path.join(SKETCH_DIR, fname)
            if os.path.getmtime(fpath) > bin_mtime:
                return True
    return False


def binary_is_valid(profile, git_ref):
    """
    Comprueba si el binario compilado coincide con perfil+versión seleccionada
    y los fuentes no han cambiado desde la compilación.
    """
    bin_path = os.path.join(BUILD_DIR, f"{SKETCH_NAME}.ino.bin")
    if not os.path.exists(bin_path):
        return False, "No compilado"

    meta = load_metadata()
    if not meta:
        return False, "Sin metadatos"

    if meta.get("profile") != profile:
        return False, f"Perfil diferente (compilado: {meta.get('profile')})"

    compiled_ref = meta.get("git_ref")   # None = working copy
    if git_ref != compiled_ref:
        return False, "Versión diferente"

    # Para working copy: comprobar si los fuentes son más nuevos
    if git_ref is None and source_files_newer_than_binary(bin_path):
        return False, "Fuentes modificados desde última compilación"

    built_at = meta.get("built_at", "")
    size_kb   = meta.get("bin_size_bytes", 0) // 1024
    return True, f"OK  ·  {size_kb} KB  ·  compilado {built_at}"


# ── Aplicación ────────────────────────────────────────────────────────────────

class FlasherApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Aquantia Flash Tool")
        self.resizable(False, False)
        self._arduino_cli = find_arduino_cli()
        self._espota       = find_espota()
        self._busy         = False
        self._version_list = get_version_list()
        self._tmp_sketch   = None   # directorio temporal si se usa versión histórica
        self._build_ui()
        self._check_tools()
        self._refresh_binary_status()

    # ── Construcción UI ───────────────────────────────────────────────────────

    def _build_ui(self):
        PAD = dict(padx=10, pady=4)

        # ── Cabecera ──
        hdr = tk.Frame(self, bg="#0c8ecc")
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew")
        tk.Label(hdr, text="💧  Aquantia Flash Tool",
                 bg="#0c8ecc", fg="white",
                 font=("Segoe UI", 14, "bold"), pady=10).pack()

        # ── Perfil ──
        tk.Label(self, text="Perfil:",
                 font=("Segoe UI", 9, "bold")).grid(row=1, column=0, sticky="w", **PAD)
        self._profile_var = tk.StringVar(value=list(PROFILES.keys())[1])  # default: irrigation
        profile_cb = ttk.Combobox(self, textvariable=self._profile_var,
                                  values=list(PROFILES.keys()),
                                  state="readonly", width=38)
        profile_cb.grid(row=1, column=1, sticky="ew", **PAD)
        profile_cb.bind("<<ComboboxSelected>>", lambda _: self._refresh_binary_status())

        # ── Versión ──
        tk.Label(self, text="Versión:",
                 font=("Segoe UI", 9, "bold")).grid(row=2, column=0, sticky="w", **PAD)
        ver_labels = [v[0] for v in self._version_list]
        self._version_var = tk.StringVar(value=ver_labels[0] if ver_labels else "")
        self._version_cb  = ttk.Combobox(self, textvariable=self._version_var,
                                         values=ver_labels, state="readonly", width=38)
        self._version_cb.grid(row=2, column=1, sticky="ew", **PAD)
        self._version_cb.bind("<<ComboboxSelected>>", lambda _: self._refresh_binary_status())

        # ── Estado del binario ──
        tk.Label(self, text="Binario:",
                 font=("Segoe UI", 9, "bold")).grid(row=3, column=0, sticky="w", **PAD)
        bin_frame = tk.Frame(self)
        bin_frame.grid(row=3, column=1, sticky="ew", **PAD)
        self._bin_status_var = tk.StringVar(value="Comprobando...")
        self._bin_status_lbl = tk.Label(bin_frame, textvariable=self._bin_status_var,
                                        font=("Segoe UI", 9), anchor="w")
        self._bin_status_lbl.pack(side="left", fill="x", expand=True)
        ttk.Button(bin_frame, text="🗑", width=3,
                   command=self._clean_build).pack(side="right")

        # ── Puerto COM ──
        tk.Label(self, text="Puerto USB:",
                 font=("Segoe UI", 9, "bold")).grid(row=4, column=0, sticky="w", **PAD)
        port_frame = tk.Frame(self)
        port_frame.grid(row=4, column=1, sticky="ew", **PAD)
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(port_frame, textvariable=self._port_var,
                                      state="readonly", width=28)
        self._port_cb.pack(side="left")
        ttk.Button(port_frame, text="⟳", width=3,
                   command=self._refresh_ports).pack(side="left", padx=4)
        self._refresh_ports()

        # ── IP OTA ──
        tk.Label(self, text="IP OTA:",
                 font=("Segoe UI", 9, "bold")).grid(row=5, column=0, sticky="w", **PAD)
        self._ip_var = tk.StringVar()
        ttk.Entry(self, textvariable=self._ip_var, width=40).grid(
            row=5, column=1, sticky="ew", **PAD)

        # ── Botones ──
        btn_frame = tk.Frame(self)
        btn_frame.grid(row=6, column=0, columnspan=2, pady=8)
        self._btn_compile = ttk.Button(btn_frame, text="⚙  Compilar",
                                       width=16, command=self._compile)
        self._btn_compile.grid(row=0, column=0, padx=6)
        self._btn_serial  = ttk.Button(btn_frame, text="🔌  Flash USB",
                                       width=16, command=self._flash_serial)
        self._btn_serial.grid(row=0, column=1, padx=6)
        self._btn_ota     = ttk.Button(btn_frame, text="📡  Flash OTA",
                                       width=16, command=self._flash_ota)
        self._btn_ota.grid(row=0, column=2, padx=6)

        # ── Estado ──
        self._status_var = tk.StringVar(value="Listo")
        tk.Label(self, textvariable=self._status_var,
                 fg="#555", font=("Segoe UI", 8, "italic")).grid(
            row=7, column=0, columnspan=2, sticky="w", padx=10)

        # ── Log ──
        tk.Label(self, text="Log:",
                 font=("Segoe UI", 9, "bold")).grid(
            row=8, column=0, columnspan=2, sticky="w", padx=10, pady=(6, 0))
        self._log = scrolledtext.ScrolledText(
            self, height=16, width=74,
            bg="#1e1e1e", fg="#d4d4d4",
            font=("Consolas", 9), state="disabled")
        self._log.grid(row=9, column=0, columnspan=2, padx=10, pady=(0, 10))
        self.columnconfigure(1, weight=1)

    # ── Helpers UI ────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = list_com_ports()
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

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
        if not busy:
            self._refresh_binary_status()

    def _get_selected_ref(self):
        """Devuelve (git_ref | None, label). None = working copy."""
        sel = self._version_var.get()
        for label, ref in self._version_list:
            if label == sel:
                return ref, label
        return None, sel

    def _refresh_binary_status(self):
        profile = PROFILES.get(self._profile_var.get(), "?")
        git_ref, _ = self._get_selected_ref()
        valid, msg = binary_is_valid(profile, git_ref)
        if valid:
            self._bin_status_var.set(f"✓  {msg}")
            self._bin_status_lbl.config(fg="#4ec9b0")
        else:
            self._bin_status_var.set(f"⚠  {msg}")
            self._bin_status_lbl.config(fg="#f0a000")

    def _clean_build(self):
        try:
            if os.path.isdir(BUILD_DIR):
                shutil.rmtree(BUILD_DIR)
            self._log_line("🗑  Build limpiado.", "#888")
        except Exception as e:
            self._log_line(f"Error limpiando build: {e}", "#f44747")
        self._refresh_binary_status()

    def _check_tools(self):
        if not self._arduino_cli:
            self._log_line("⚠  arduino-cli no encontrado. Instala Arduino IDE 2.x.", "#f0a000")
        else:
            self._log_line(f"✓  arduino-cli: {self._arduino_cli}", "#4ec9b0")
        if not self._espota:
            self._log_line("⚠  espota.py no encontrado (OTA no disponible).", "#f0a000")
        else:
            self._log_line(f"✓  espota.py: {self._espota}", "#4ec9b0")
        _, ver_desc, dirty = get_current_version()
        self._log_line(f"✓  Repositorio: {ver_desc}" + (" [modificado]" if dirty else ""), "#4ec9b0")
        self._log_line(f"✓  Build dir:   {BUILD_DIR}", "#4ec9b0")
        self._log_line("")

    # ── Subprocesos ───────────────────────────────────────────────────────────

    def _run_cmd(self, cmd, cwd=None):
        self._log_line(f"$ {' '.join(str(x) for x in cmd)}", "#569cd6")
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace",
                cwd=cwd or SKETCH_DIR
            )
            for line in proc.stdout:
                line = line.rstrip("\n")
                color = ("#f44747" if ("error:" in line.lower() or "fatal" in line.lower())
                         else "#ce9178" if "warning:" in line.lower()
                         else None)
                self._log_line(line, color)
            proc.wait()
            return proc.returncode == 0, proc.returncode
        except FileNotFoundError as e:
            self._log_line(f"ERROR: {e}", "#f44747")
            return False, -1

    # ── Lógica principal ──────────────────────────────────────────────────────

    def _get_sketch_for_ref(self, git_ref):
        """
        Devuelve la ruta del sketch a compilar:
        - None = working copy (SKETCH_DIR)
        - git_ref = extraer a temp y devolver esa ruta
        """
        if git_ref is None:
            return SKETCH_DIR
        # Limpiar temp anterior
        if self._tmp_sketch and os.path.isdir(os.path.dirname(self._tmp_sketch)):
            try:
                shutil.rmtree(os.path.dirname(self._tmp_sketch))
            except Exception:
                pass
        path = export_version_to_temp(git_ref)
        self._tmp_sketch = path
        return path

    def _do_compile(self, force=False):
        """
        Compila el sketch si es necesario.
        Si force=False y el binario ya está válido, omite la compilación.
        Devuelve (ok, bin_path).
        """
        if not self._arduino_cli:
            messagebox.showerror("Error", "arduino-cli no encontrado.")
            return False, None

        profile = PROFILES.get(self._profile_var.get(), "2")
        git_ref, ver_label = self._get_selected_ref()
        bin_path = os.path.join(BUILD_DIR, f"{SKETCH_NAME}.ino.bin")

        # ¿Podemos saltarnos la compilación?
        if not force:
            valid, reason = binary_is_valid(profile, git_ref)
            if valid:
                self._log_line(f"✓  Binario en caché válido — omitiendo compilación", "#4ec9b0")
                self._log_line(f"   {reason}", "#888")
                return True, bin_path

        # Preparar directorio de sketch
        sketch_path = self._get_sketch_for_ref(git_ref)
        if sketch_path is None:
            self._log_line(f"✗  No se pudo extraer la versión '{ver_label}'", "#f44747")
            return False, None

        os.makedirs(BUILD_DIR, exist_ok=True)

        self._log_line(f"\n{'─'*60}", "#444")
        self._log_line(f"  COMPILANDO  perfil={profile}  versión={ver_label}", "#dcdcaa")
        self._log_line(f"{'─'*60}\n", "#444")
        self._set_status("Compilando...")

        cmd = [
            self._arduino_cli, "compile",
            "--fqbn", FQBN,
            "--build-property", f"build.extra_flags=-DDEVICE_PROFILE={profile}",
            "--build-path", BUILD_DIR,
            sketch_path,
        ]
        ok, _ = self._run_cmd(cmd, cwd=sketch_path)

        if ok and os.path.exists(bin_path):
            size = os.path.getsize(bin_path)
            now  = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
            save_metadata({
                "version_label": ver_label,
                "git_ref":       git_ref,
                "profile":       profile,
                "built_at":      now,
                "bin_size_bytes": size,
            })
            self._log_line(f"\n✓  Compilación OK — {size // 1024} KB", "#4ec9b0")
            self._set_status(f"Compilado: {ver_label}  ({size // 1024} KB)")
        else:
            self._log_line("\n✗  Compilación fallida.\n", "#f44747")
            self._set_status("Error de compilación")
            return False, None

        return True, bin_path

    def _compile(self):
        if self._busy:
            return
        self._clear_log()
        self._set_busy(True)
        def run():
            self._do_compile(force=True)
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
            ok, bin_path = self._do_compile()
            if not ok:
                self._set_busy(False)
                return

            self._log_line(f"\n{'─'*60}", "#444")
            self._log_line(f"  FLASH USB — {port}", "#dcdcaa")
            self._log_line(f"{'─'*60}", "#444")
            self._log_line("  ⚡  Entra en modo bootloader:", "#f0a000")
            self._log_line("      1. Mantén pulsado GPIO0 (BOOT)", "#f0a000")
            self._log_line("      2. Pulsa y suelta EN/RST", "#f0a000")
            self._log_line("      3. Suelta GPIO0\n", "#f0a000")
            self._set_status(f"Flasheando por USB en {port}...")

            cmd = [self._arduino_cli, "upload",
                   "--fqbn", FQBN, "--port", port,
                   "--input-dir", BUILD_DIR, SKETCH_DIR]
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
            messagebox.showerror("Error", "espota.py no encontrado.")
            return
        self._clear_log()
        self._set_busy(True)

        def run():
            ok, bin_path = self._do_compile()
            if not ok:
                self._set_busy(False)
                return

            self._log_line(f"\n{'─'*60}", "#444")
            self._log_line(f"  FLASH OTA — {ip}:3232", "#dcdcaa")
            self._log_line(f"{'─'*60}\n", "#444")
            self._set_status(f"Flasheando OTA a {ip}...")

            cmd = ([sys.executable, self._espota] if self._espota.endswith(".py")
                   else [self._espota])
            cmd += ["-i", ip, "-p", "3232", "-f", bin_path]
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
