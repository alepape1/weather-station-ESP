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
import secrets as _secrets
import csv
import urllib.request
import re
import socket
import time
try:
    import qrcode
    from PIL import ImageTk
    _QR_AVAILABLE = True
except ImportError:
    _QR_AVAILABLE = False
try:
    from zeroconf import ServiceBrowser, Zeroconf
    _ZEROCONF = True
except ImportError:
    _ZEROCONF = False

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

BACKEND_URL  = "https://meteo.aquantialab.com"
APP_BASE_URL = "https://meteo.aquantialab.com"   # URL base del QR de claim

REGISTRY_FILE = os.path.join(SCRIPT_DIR, "devices_registry.csv")
REGISTRY_QR_DIR = os.path.join(SCRIPT_DIR, "devices_qr")

# ── Rutas de herramientas adicionales ─────────────────────────────────────────

ESPTOOL_CANDIDATES = [
    r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\esptool\esptool.exe",
    r"C:\Program Files (x86)\Arduino IDE\resources\app\lib\backend\resources\esptool\esptool.exe",
    "esptool.exe",
    "esptool",
    "esptool.py",
]

NVS_GEN_GLOB_PATTERNS = [
    os.path.expanduser(
        r"~\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\*\tools\nvs_partition_gen.py"
    ),
    os.path.expanduser(
        r"~\AppData\Local\Arduino15\packages\esp32\hardware\esp32\*\tools\nvs_partition_gen.py"
    ),
]

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


def get_commit_details(git_ref):
    """Devuelve dict con hash, date, author y body del commit dado."""
    ref = git_ref if git_ref else "HEAD"
    raw = _git("log", "-1",
               "--format=%H%n%ad%n%an%n%B",
               "--date=format:%Y-%m-%d %H:%M", ref)
    if not raw:
        return None
    parts = raw.split("\n", 3)
    return {
        "hash":   parts[0][:12]     if len(parts) > 0 else "",
        "date":   parts[1]          if len(parts) > 1 else "",
        "author": parts[2]          if len(parts) > 2 else "",
        "body":   parts[3].strip()  if len(parts) > 3 else "",
    }


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

def find_esptool():
    for path in ESPTOOL_CANDIDATES:
        if os.path.isfile(path):
            return path
        found = shutil.which(path)
        if found:
            return found
    return None


def find_nvs_gen():
    for pattern in NVS_GEN_GLOB_PATTERNS:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[-1]
    return None


# ── Partition table helpers ───────────────────────────────────────────────────

# La tabla de particiones siempre está en 0x8000 en todos los ESP32/S2/S3/C3/C6.
_PARTITION_TABLE_OFFSET = 0x8000
_PARTITION_TABLE_SIZE   = 0xC00    # 3 KB: más que suficiente para cualquier tabla
_PARTITION_ENTRY_MAGIC  = b'\xAA\x50'
_PARTITION_ENTRY_SIZE   = 32

# type / subtype para NVS
_PART_TYPE_DATA    = 0x01
_PART_SUBTYPE_NVS  = 0x02


def read_partition_table(esptool, port):
    """
    Lee la tabla de particiones del chip conectado al puerto serie.
    Devuelve lista de dicts {name, type, subtype, offset, size}
    o None si la lectura o el parseo fallan.
    La tabla está en 0x8000 para todos los ESP32 (incluidos S2, S3, C3, C6).
    """
    import struct, tempfile

    cmd = ([sys.executable, esptool] if esptool.endswith(".py") else [esptool])
    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bin")
    os.close(tmp_fd)
    try:
        cmd += ["--port", port, "read_flash",
                hex(_PARTITION_TABLE_OFFSET),
                hex(_PARTITION_TABLE_SIZE),
                tmp_path]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
        if r.returncode != 0:
            return None
        with open(tmp_path, "rb") as f:
            data = f.read()
    except Exception:
        return None
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass

    partitions = []
    for i in range(0, len(data) - _PARTITION_ENTRY_SIZE + 1, _PARTITION_ENTRY_SIZE):
        entry = data[i:i + _PARTITION_ENTRY_SIZE]
        if entry[:2] != _PARTITION_ENTRY_MAGIC:
            continue  # fin de tabla o padding
        p_type    = entry[2]
        p_subtype = entry[3]
        p_offset  = struct.unpack_from("<I", entry, 4)[0]
        p_size    = struct.unpack_from("<I", entry, 8)[0]
        p_name    = entry[12:28].rstrip(b"\x00").decode("ascii", errors="replace")
        partitions.append({
            "name":    p_name,
            "type":    p_type,
            "subtype": p_subtype,
            "offset":  p_offset,
            "size":    p_size,
        })
    return partitions or None


def find_nvs_partition(partitions):
    """Devuelve la primera partición NVS (type=data=0x01, subtype=nvs=0x02) o None."""
    for p in partitions:
        if p["type"] == _PART_TYPE_DATA and p["subtype"] == _PART_SUBTYPE_NVS:
            return p
    return None


# ── Factory provision helpers ─────────────────────────────────────────────────

def fp_read_mac(esptool, port):
    """Lee la MAC del ESP32 vía esptool. Devuelve 'AA:BB:CC:DD:EE:FF' o None."""
    cmd = ([sys.executable, esptool] if esptool.endswith(".py") else [esptool])
    cmd += ["--port", port, "read_mac"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        for line in r.stdout.splitlines():
            if "MAC:" in line.upper():
                parts = line.upper().split("MAC:")
                mac = parts[-1].strip().split()[0]
                return mac.replace("-", ":").upper()
    except Exception:
        pass
    return None


def fp_read_flash_id(esptool, port):
    """Lee el flash_id del ESP32 vía esptool. Devuelve hex string u None."""
    cmd = ([sys.executable, esptool] if esptool.endswith(".py") else [esptool])
    cmd += ["--port", port, "flash_id"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        for line in r.stdout.splitlines():
            low = line.lower()
            if "manufacturer" in low or "flash id" in low or "device" in low:
                # Extraer el primer token hexadecimal de la línea
                for token in line.split():
                    token = token.strip(":").replace("0x", "")
                    if all(c in "0123456789abcdefABCDEF" for c in token) and len(token) >= 2:
                        return token.upper().zfill(8)
    except Exception:
        pass
    return None


def fp_generate_token():
    return _secrets.token_urlsafe(32)


def fp_hash_token(token):
    try:
        import bcrypt as _bcrypt
        return _bcrypt.hashpw(token.encode(), _bcrypt.gensalt(rounds=12)).decode()
    except ImportError:
        return None


def fp_register_backend(mac, token_hash, serial_number, backend_url=BACKEND_URL):
    payload = json.dumps({
        "mac": mac,
        "token_hash": token_hash,
        "serial_number": serial_number,
    }).encode()
    req = urllib.request.Request(
        f"{backend_url}/api/devices/register_factory",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read())


def show_qr_window(serial, mac):
    """Abre una ventana con el QR del serial del dispositivo."""
    if not _QR_AVAILABLE:
        messagebox.showwarning(
            "qrcode no instalado",
            "Instala la librería para generar QR:\n\npip install qrcode[pil]"
        )
        return

    win = tk.Toplevel()
    win.title(f"Etiqueta — {serial}")
    win.resizable(False, False)
    win.configure(bg="#1e1e1e")

    img = qrcode.make(f"{APP_BASE_URL}/claim?serial={serial}")
    img = img.resize((220, 220))
    tk_img = ImageTk.PhotoImage(img)

    lbl_qr = tk.Label(win, image=tk_img, bg="#1e1e1e", bd=0)
    lbl_qr.image = tk_img  # mantener referencia
    lbl_qr.pack(padx=20, pady=(20, 8))

    tk.Label(win, text=serial, font=("Courier New", 13, "bold"),
             bg="#1e1e1e", fg="#dcdcaa").pack()
    tk.Label(win, text=mac, font=("Segoe UI", 9),
             bg="#1e1e1e", fg="#888").pack(pady=(2, 12))

    def save_png():
        from tkinter import filedialog
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            initialfile=f"{serial}.png",
            filetypes=[("PNG", "*.png")],
        )
        if path:
            qrcode.make(f"{APP_BASE_URL}/claim?serial={serial}").save(path)

    ttk.Button(win, text="Guardar PNG", command=save_png).pack(pady=(0, 16))


def fp_save_registry(serial, mac, profile_label, ver_label):
    """Añade una fila al CSV de registro y guarda el PNG del QR en devices_qr/."""
    file_exists = os.path.isfile(REGISTRY_FILE)
    with open(REGISTRY_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["fecha", "serial", "mac", "perfil", "version_firmware", "qr_png"])
        qr_path = ""
        if _QR_AVAILABLE:
            os.makedirs(REGISTRY_QR_DIR, exist_ok=True)
            qr_path = os.path.join(REGISTRY_QR_DIR, f"{serial}.png")
            if not os.path.isfile(qr_path):
                qrcode.make(f"{APP_BASE_URL}/claim?serial={serial}").save(qr_path)
        writer.writerow([
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            serial,
            mac,
            profile_label,
            ver_label,
            qr_path,
        ])


def fp_write_nvs(esptool, nvs_gen, port, serial_number, token):
    """Genera la partición NVS y la flashea a 0x9000."""
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = os.path.join(tmpdir, "nvs.csv")
        bin_path = os.path.join(tmpdir, "nvs.bin")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows([
                ["key", "type", "encoding", "value"],
                ["namespace", "namespace", "", "aquantia"],
                ["mqtt_token",    "data", "string", token],
                ["serial_number", "data", "string", serial_number],
            ])

        # Intentar nvs_partition_gen.py de Arduino ESP32
        ok = False
        if nvs_gen:
            r = subprocess.run(
                [sys.executable, nvs_gen, "generate", csv_path, bin_path, "0x6000"],
                capture_output=True, text=True, timeout=30
            )
            ok = r.returncode == 0

        if not ok:
            # Fallback: módulo pip esp_idf_nvs_partition_gen
            # Sintaxis: generate <input> <output> <size>
            r = subprocess.run(
                [sys.executable, "-m", "esp_idf_nvs_partition_gen",
                 "generate", csv_path, bin_path, "0x6000"],
                capture_output=True, text=True, timeout=30
            )
            ok = r.returncode == 0

        if not ok:
            raise RuntimeError("nvs_partition_gen no disponible. "
                               "Instala: pip install esp-idf-nvs-partition-gen")

        # Flashear NVS a 0x9000
        cmd = ([sys.executable, esptool] if esptool.endswith(".py") else [esptool])
        cmd += ["--port", port, "write_flash", "0x9000", bin_path]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if r.returncode != 0:
            raise RuntimeError(f"Error flash NVS: {r.stderr.strip()}")


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


def discover_ota_devices(timeout=4):
    """
    Descubre dispositivos ESP32 con ArduinoOTA activo en la red local.
    Usa mDNS (_arduino._tcp) via zeroconf si está instalado,
    o resuelve hostnames conocidos como fallback.
    Devuelve lista de (nombre, ip, puerto).
    """
    found = []
    if _ZEROCONF:
        class _Listener:
            def add_service(self, zc, type_, name):
                info = zc.get_service_info(type_, name)
                if info and info.addresses:
                    ip = socket.inet_ntoa(info.addresses[0])
                    label = name.replace("._arduino._tcp.local.", "")
                    found.append((label, ip, info.port or 3232))
            def remove_service(self, *_): pass
            def update_service(self, *_): pass
        zc = Zeroconf()
        ServiceBrowser(zc, "_arduino._tcp.local.", _Listener())
        time.sleep(timeout)
        zc.close()
    else:
        # Fallback: intentar resolver hostnames conocidos
        for hostname in ["meteostation-esp32", "aquantia-device"]:
            try:
                ip = socket.gethostbyname(f"{hostname}.local")
                found.append((hostname, ip, 3232))
            except socket.gaierror:
                pass
    return found


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
        self._esptool      = find_esptool()
        self._nvs_gen      = find_nvs_gen()
        self._busy         = False
        self._version_list = get_version_list()
        self._tmp_sketch   = None
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
                 font=("Segoe UI", 14, "bold"), pady=6).pack()
        _, ver_desc, dirty = get_current_version()
        ver_text = ver_desc + (" [modificado]" if dirty else "")
        self._hdr_ver_var = tk.StringVar(value=f"firmware  {ver_text}")
        tk.Label(hdr, textvariable=self._hdr_ver_var,
                 bg="#0c8ecc", fg="#cce8f7",
                 font=("Consolas", 8), pady=(0)).pack()

        # ── Perfil ──
        tk.Label(self, text="Perfil:",
                 font=("Segoe UI", 9, "bold")).grid(row=1, column=0, sticky="w", **PAD)
        self._profile_var = tk.StringVar(value=list(PROFILES.keys())[1])
        profile_cb = ttk.Combobox(self, textvariable=self._profile_var,
                                  values=list(PROFILES.keys()),
                                  state="readonly", width=38)
        profile_cb.grid(row=1, column=1, sticky="ew", **PAD)
        profile_cb.bind("<<ComboboxSelected>>", lambda _: self._refresh_binary_status())

        # ── Versión ──
        tk.Label(self, text="Versión:",
                 font=("Segoe UI", 9, "bold")).grid(row=2, column=0, sticky="w", **PAD)
        ver_frame = tk.Frame(self)
        ver_frame.grid(row=2, column=1, sticky="ew", **PAD)
        ver_labels = [v[0] for v in self._version_list]
        self._version_var = tk.StringVar(value=ver_labels[0] if ver_labels else "")
        self._version_cb  = ttk.Combobox(ver_frame, textvariable=self._version_var,
                                         values=ver_labels, state="readonly", width=33)
        self._version_cb.pack(side="left")
        self._version_cb.bind("<<ComboboxSelected>>", self._on_version_selected)
        self._btn_refresh_ver = ttk.Button(ver_frame, text="⟳", width=3,
                                           command=self._refresh_versions)
        self._btn_refresh_ver.pack(side="left", padx=4)

        # ── Info del commit ──
        tk.Label(self, text="Commit:",
                 font=("Segoe UI", 9, "bold")).grid(row=3, column=0, sticky="nw", **PAD)
        commit_frame = tk.Frame(self, bg="#252526", bd=1, relief="sunken")
        commit_frame.grid(row=3, column=1, sticky="ew", padx=10, pady=(2, 4))
        self._commit_text = tk.Text(
            commit_frame, height=3, width=40,
            bg="#252526", fg="#cccccc", font=("Consolas", 8),
            state="disabled", wrap="word", bd=0,
            selectbackground="#264f78")
        self._commit_text.pack(fill="both", expand=True, padx=4, pady=2)
        # Configurar tags de color
        self._commit_text.tag_config("hash",   foreground="#569cd6")
        self._commit_text.tag_config("meta",   foreground="#888888")
        self._commit_text.tag_config("body",   foreground="#d4d4d4")
        self._commit_text.tag_config("dirty",  foreground="#f0a000")
        self._update_commit_info(None)

        # ── Estado del binario ──
        tk.Label(self, text="Binario:",
                 font=("Segoe UI", 9, "bold")).grid(row=4, column=0, sticky="w", **PAD)
        bin_frame = tk.Frame(self)
        bin_frame.grid(row=4, column=1, sticky="ew", **PAD)
        self._bin_status_var = tk.StringVar(value="Comprobando...")
        self._bin_status_lbl = tk.Label(bin_frame, textvariable=self._bin_status_var,
                                        font=("Segoe UI", 9), anchor="w")
        self._bin_status_lbl.pack(side="left", fill="x", expand=True)
        ttk.Button(bin_frame, text="🗑", width=3,
                   command=self._clean_build).pack(side="right")

        # ── Puerto COM ──
        tk.Label(self, text="Puerto USB:",
                 font=("Segoe UI", 9, "bold")).grid(row=5, column=0, sticky="w", **PAD)
        port_frame = tk.Frame(self)
        port_frame.grid(row=5, column=1, sticky="ew", **PAD)
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(port_frame, textvariable=self._port_var,
                                      state="readonly", width=28)
        self._port_cb.pack(side="left")
        ttk.Button(port_frame, text="⟳", width=3,
                   command=self._refresh_ports).pack(side="left", padx=4)
        self._refresh_ports()

        # ── IP OTA + descubrimiento de dispositivos ──
        tk.Label(self, text="IP OTA:",
                 font=("Segoe UI", 9, "bold")).grid(row=6, column=0, sticky="w", **PAD)
        ota_frame = tk.Frame(self)
        ota_frame.grid(row=6, column=1, sticky="ew", **PAD)
        self._ip_var = tk.StringVar()
        tk.Entry(ota_frame, textvariable=self._ip_var, width=18,
                 font=("Consolas", 9)).pack(side="left")
        self._btn_discover_ota = ttk.Button(ota_frame, text="🔍 Buscar",
                                            width=10, command=self._discover_ota)
        self._btn_discover_ota.pack(side="left", padx=(6, 0))
        # Combobox de dispositivos encontrados (oculto hasta que haya resultados)
        self._ota_devices_var = tk.StringVar()
        self._ota_devices_cb  = ttk.Combobox(ota_frame, textvariable=self._ota_devices_var,
                                              state="readonly", width=24)
        self._ota_devices_cb.pack(side="left", padx=(6, 0))
        self._ota_devices_cb.bind("<<ComboboxSelected>>", self._on_ota_device_selected)
        self._ota_found = []   # lista de (nombre, ip, puerto)

        # ── Barra de progreso ──
        self._pb = ttk.Progressbar(self, orient="horizontal",
                                   mode="indeterminate", length=400)
        self._pb.grid(row=7, column=0, columnspan=2, padx=10, pady=(2, 0), sticky="ew")

        # ── Botones ──
        btn_frame = tk.Frame(self)
        btn_frame.grid(row=8, column=0, columnspan=2, pady=8)
        # ── Modo DEV / PROD ──
        self._dev_mode_var = tk.BooleanVar(value=self._read_dev_mode())
        mode_frame = tk.Frame(btn_frame)
        mode_frame.grid(row=0, column=0, columnspan=3, pady=(0, 6))
        tk.Label(mode_frame, text="Modo:", font=("Segoe UI", 9, "bold")).pack(side="left")
        self._btn_dev  = tk.Button(mode_frame, text="🛠 DEV",  width=10,
                                   relief="sunken", bg="#2d6a9f", fg="white",
                                   font=("Segoe UI", 9, "bold"),
                                   command=self._set_dev_mode)
        self._btn_dev.pack(side="left", padx=2)
        self._btn_prod = tk.Button(mode_frame, text="🏭 PROD", width=10,
                                   relief="raised", bg="#e0e0e0",
                                   font=("Segoe UI", 9),
                                   command=self._set_prod_mode)
        self._btn_prod.pack(side="left", padx=2)
        self._mode_lbl = tk.Label(mode_frame, font=("Segoe UI", 8, "italic"), fg="#888")
        self._mode_lbl.pack(side="left", padx=8)
        self._update_mode_ui()

        self._btn_compile = ttk.Button(btn_frame, text="⚙  Compilar",
                                       width=16, command=self._compile)
        self._btn_compile.grid(row=1, column=0, padx=6)
        self._btn_serial  = ttk.Button(btn_frame, text="🔌  Flash USB",
                                       width=16, command=self._flash_serial)
        self._btn_serial.grid(row=1, column=1, padx=6)
        self._btn_ota     = ttk.Button(btn_frame, text="📡  Flash OTA",
                                       width=16, command=self._flash_ota)
        self._btn_ota.grid(row=1, column=2, padx=6)

        # ── Factory Provision ──
        sep = tk.Frame(self, height=1, bg="#dde3ea")
        sep.grid(row=9, column=0, columnspan=2, sticky="ew", padx=10, pady=(4, 0))

        fp_frame = tk.Frame(self)
        fp_frame.grid(row=10, column=0, columnspan=2, sticky="ew", padx=10, pady=4)

        tk.Label(fp_frame, text="Backend URL:",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self._backend_var = tk.StringVar(value=BACKEND_URL)
        ttk.Entry(fp_frame, textvariable=self._backend_var, width=28).pack(
            side="left", padx=(4, 10))
        self._btn_factory = ttk.Button(fp_frame, text="🏷  Provisionar fábrica",
                                       width=22, command=self._factory_provision)
        self._btn_factory.pack(side="left")
        self._btn_erase_nvs = ttk.Button(fp_frame, text="🗑 Borrar NVS",
                                         width=14, command=self._erase_nvs)
        self._btn_erase_nvs.pack(side="left", padx=(8, 0))

        # ── Estado ──
        self._status_var = tk.StringVar(value="Listo")
        tk.Label(self, textvariable=self._status_var,
                 fg="#555", font=("Segoe UI", 8, "italic")).grid(
            row=11, column=0, columnspan=2, sticky="w", padx=10)

        # ── Log ──
        tk.Label(self, text="Log:",
                 font=("Segoe UI", 9, "bold")).grid(
            row=12, column=0, columnspan=2, sticky="w", padx=10, pady=(6, 0))
        self._log = scrolledtext.ScrolledText(
            self, height=16, width=74,
            bg="#1e1e1e", fg="#d4d4d4",
            font=("Consolas", 9), state="disabled")
        self._log.grid(row=13, column=0, columnspan=2, padx=10, pady=(0, 10))
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
        for btn in (self._btn_compile, self._btn_serial, self._btn_ota,
                    self._btn_factory, self._btn_erase_nvs, self._btn_refresh_ver,
                    self._btn_discover_ota):
            btn.config(state=state)
        if not busy:
            self._refresh_binary_status()

    def _refresh_versions(self):
        """git fetch + recarga el selector de versiones."""
        if self._busy:
            return
        self._btn_refresh_ver.config(state="disabled")
        self._set_status("Buscando versiones nuevas...")

        def run():
            self._log_line("\n● git fetch...", "#569cd6")
            _git("fetch", "--tags", "--prune")
            self._version_list = get_version_list()
            ver_labels = [v[0] for v in self._version_list]
            current = ver_labels[0] if ver_labels else ""
            self._version_cb["values"] = ver_labels
            self._version_var.set(current)
            _, ver_desc, dirty = get_current_version()
            ver_text = ver_desc + (" [modificado]" if dirty else "")
            self._hdr_ver_var.set(f"firmware  {ver_text}")
            self._log_line(f"✓  Versiones actualizadas — {ver_text}", "#4ec9b0")
            self._set_status("Versiones actualizadas")
            self._btn_refresh_ver.config(state="normal")
            self._refresh_binary_status()

        threading.Thread(target=run, daemon=True).start()

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

    def _update_commit_info(self, git_ref):
        """Actualiza el panel de info del commit seleccionado."""
        details = get_commit_details(git_ref)
        self._commit_text.config(state="normal")
        self._commit_text.delete("1.0", "end")
        if details:
            self._commit_text.insert("end", details["hash"], "hash")
            self._commit_text.insert("end", f"  {details['date']}  {details['author']}\n", "meta")
            body = details["body"]
            if body:
                # Primera línea = asunto
                lines = body.split("\n", 1)
                subject = lines[0]
                rest    = lines[1].strip() if len(lines) > 1 else ""
                self._commit_text.insert("end", subject + "\n", "body")
                if rest:
                    self._commit_text.insert("end", rest[:120], "meta")
        else:
            _, _, dirty = get_current_version()
            if dirty:
                self._commit_text.insert("end", "Working copy — cambios sin commit", "dirty")
            else:
                self._commit_text.insert("end", "(sin información de commit)", "meta")
        self._commit_text.config(state="disabled")

    def _on_version_selected(self, _event=None):
        """Maneja la selección de versión: actualiza estado y commit info."""
        git_ref, _ = self._get_selected_ref()
        self._refresh_binary_status()
        self._update_commit_info(git_ref)

    def _start_progress(self, mode="indeterminate"):
        """Arranca la barra de progreso (seguro desde cualquier hilo)."""
        def _do():
            self._pb.config(mode=mode, value=0)
            if mode == "indeterminate":
                self._pb.start(12)
            else:
                self._pb.config(maximum=100)
        self.after(0, _do)

    def _stop_progress(self, success=True):
        """Para la barra de progreso (seguro desde cualquier hilo)."""
        def _do():
            self._pb.stop()
            self._pb.config(mode="determinate", value=100 if success else 0)
        self.after(0, _do)

    def _set_progress(self, value):
        """Actualiza el valor de la barra (0-100) desde cualquier hilo."""
        self.after(0, lambda v=value: self._pb.config(value=v))

    def _discover_ota(self):
        """Busca dispositivos OTA en la red local (en background)."""
        if self._busy:
            return
        self._btn_discover_ota.config(state="disabled")
        self._ota_devices_cb["values"] = []
        self._ota_devices_var.set("")
        self._set_status("Buscando dispositivos OTA en la red...")
        self._log_line("\n● Buscando dispositivos ArduinoOTA (_arduino._tcp)...", "#569cd6")
        if not _ZEROCONF:
            self._log_line("  zeroconf no instalado — usando resolución .local de fallback", "#888")
            self._log_line("  Instala con: pip install zeroconf", "#888")

        def run():
            devices = discover_ota_devices(timeout=4)
            def on_done():
                self._ota_found = devices
                if devices:
                    labels = [f"{name}  ({ip}:{port})"
                              for name, ip, port in devices]
                    self._ota_devices_cb["values"] = labels
                    self._ota_devices_var.set(labels[0])
                    # Auto-rellenar IP si solo hay uno
                    if len(devices) == 1:
                        self._ip_var.set(devices[0][1])
                    self._log_line(f"✓  {len(devices)} dispositivo(s) encontrado(s):", "#4ec9b0")
                    for name, ip, port in devices:
                        self._log_line(f"   {name}  →  {ip}:{port}", "#dcdcaa")
                    self._set_status(f"{len(devices)} dispositivo(s) OTA encontrado(s)")
                else:
                    self._log_line("⚠  No se encontraron dispositivos OTA.", "#f0a000")
                    self._log_line("   Asegúrate de que el ESP32 está encendido y en la misma red.", "#888")
                    self._set_status("Sin dispositivos OTA")
                self._btn_discover_ota.config(state="normal")
            self.after(0, on_done)

        threading.Thread(target=run, daemon=True).start()

    def _on_ota_device_selected(self, _event=None):
        """Rellena el campo IP al seleccionar un dispositivo OTA."""
        sel = self._ota_devices_var.get()
        for label, (name, ip, port) in zip(
                self._ota_devices_cb["values"], self._ota_found):
            if label == sel:
                self._ip_var.set(ip)
                break

    def _check_tools(self):
        if not self._arduino_cli:
            self._log_line("⚠  arduino-cli no encontrado. Instala Arduino IDE 2.x.", "#f0a000")
        else:
            self._log_line(f"✓  arduino-cli: {self._arduino_cli}", "#4ec9b0")
        if not self._espota:
            self._log_line("⚠  espota.py no encontrado (OTA no disponible).", "#f0a000")
        else:
            self._log_line(f"✓  espota.py: {self._espota}", "#4ec9b0")
        if not self._esptool:
            self._log_line("⚠  esptool no encontrado (factory provision no disponible).", "#f0a000")
        else:
            self._log_line(f"✓  esptool:    {self._esptool}", "#4ec9b0")
        if not self._nvs_gen:
            self._log_line("⚠  nvs_partition_gen no encontrado (se usará pip fallback).", "#888")
        else:
            self._log_line(f"✓  nvs_gen:    {self._nvs_gen}", "#4ec9b0")
        _, ver_desc, dirty = get_current_version()
        self._log_line(f"✓  Repositorio: {ver_desc}" + (" [modificado]" if dirty else ""), "#4ec9b0")
        self._log_line(f"✓  Build dir:   {BUILD_DIR}", "#4ec9b0")
        self._log_line("")

    # ── Subprocesos ───────────────────────────────────────────────────────────

    def _run_cmd(self, cmd, cwd=None, progress_mode="determinate"):
        self._log_line(f"$ {' '.join(str(x) for x in cmd)}", "#569cd6")
        # Hitos de compilación → porcentaje aproximado
        _compile_milestones = {
            "compiling libraries":           20,
            "compiling sketch":              55,
            "generating compilation":        75,
            "linking everything together":   88,
        }
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

                # ── Progreso compilación (milestones conocidos) ──
                ll = line.lower()
                for milestone, pct in _compile_milestones.items():
                    if milestone in ll:
                        self._set_progress(pct)
                        break

                # ── Progreso flash (formatos reales de esptool / espota) ──
                # esptool USB:  "Writing at 0x00010000... (10 %)"
                # espota OTA:   "[=========   ] 35%"
                m = re.search(r'\(\s*(\d+)\s*%\s*\)', line)   # esptool: (10 %)
                if not m:
                    m = re.search(r'\]\s*(\d+)%', line)        # espota:  ] 35%
                if m:
                    pct = min(int(m.group(1)), 100)
                    self._set_progress(pct)

            proc.wait()
            return proc.returncode == 0, proc.returncode
        except FileNotFoundError as e:
            self._log_line(f"ERROR: {e}", "#f44747")
            return False, -1

    # ── DEV / PROD mode ───────────────────────────────────────────────────────

    _SECRETS_PATH = os.path.join(SKETCH_DIR, "secrets.h")

    def _read_dev_mode(self):
        """Lee si #define DEV_MODE está activo en secrets.h."""
        try:
            with open(self._SECRETS_PATH, "r", encoding="utf-8") as f:
                for line in f:
                    s = line.strip()
                    if s == "#define DEV_MODE":
                        return True
                    if s == "// #define DEV_MODE":
                        return False
        except Exception:
            pass
        return True  # por defecto dev

    def _write_dev_mode(self, dev):
        """Activa o desactiva #define DEV_MODE en secrets.h."""
        try:
            with open(self._SECRETS_PATH, "r", encoding="utf-8") as f:
                lines = f.readlines()
            with open(self._SECRETS_PATH, "w", encoding="utf-8") as f:
                for line in lines:
                    s = line.strip()
                    if s in ("#define DEV_MODE", "// #define DEV_MODE"):
                        f.write(("#define DEV_MODE\n") if dev else ("// #define DEV_MODE\n"))
                    else:
                        f.write(line)
            return True
        except Exception as e:
            self._log_line(f"Error escribiendo secrets.h: {e}", "#f44747")
            return False

    def _set_dev_mode(self):
        if self._write_dev_mode(True):
            self._dev_mode_var.set(True)
            self._update_mode_ui()
            self._refresh_binary_status()

    def _set_prod_mode(self):
        if self._write_dev_mode(False):
            self._dev_mode_var.set(False)
            self._update_mode_ui()
            self._refresh_binary_status()

    def _update_mode_ui(self):
        dev = self._dev_mode_var.get()
        if dev:
            self._btn_dev.config(relief="sunken",  bg="#2d6a9f", fg="white")
            self._btn_prod.config(relief="raised", bg="#e0e0e0", fg="black")
            self._mode_lbl.config(text="Arranca directo con secrets.h", fg="#4ec9b0")
        else:
            self._btn_dev.config(relief="raised",  bg="#e0e0e0", fg="black")
            self._btn_prod.config(relief="sunken", bg="#8b4513", fg="white")
            self._mode_lbl.config(text="Portal SoftAP + NVS + token de fábrica", fg="#f0a000")

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
            "--build-property", "build.partitions=min_spiffs",
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
            self._start_progress("indeterminate")
            ok, _ = self._do_compile(force=True)
            self._stop_progress(ok)
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
            self._start_progress("indeterminate")
            ok, bin_path = self._do_compile()
            if not ok:
                self._stop_progress(False)
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
            self._start_progress("determinate")

            cmd = [self._arduino_cli, "upload",
                   "--fqbn", FQBN, "--port", port,
                   "--input-dir", BUILD_DIR, SKETCH_DIR]
            ok, _ = self._run_cmd(cmd)
            self._stop_progress(ok)
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
            self._start_progress("indeterminate")
            ok, bin_path = self._do_compile()
            if not ok:
                self._stop_progress(False)
                self._set_busy(False)
                return

            self._log_line(f"\n{'─'*60}", "#444")
            self._log_line(f"  FLASH OTA — {ip}:3232", "#dcdcaa")
            self._log_line(f"{'─'*60}\n", "#444")
            self._set_status(f"Flasheando OTA a {ip}...")
            self._start_progress("determinate")

            py_exe = sys.executable.replace("pythonw.exe", "python.exe")
            cmd = ([py_exe, self._espota] if self._espota.endswith(".py")
                   else [self._espota])
            cmd += ["-i", ip, "-p", "3232", "-f", bin_path]
            ok, _ = self._run_cmd(cmd)
            self._stop_progress(ok)
            if ok:
                self._log_line("\n✓  Flash OTA completado.\n", "#4ec9b0")
                self._set_status("Flash OTA completado")
            else:
                self._log_line("\n✗  Flash OTA fallido.\n", "#f44747")
                self._set_status("Error en flash OTA")
            self._set_busy(False)

        threading.Thread(target=run, daemon=True).start()


    # ── Factory Provision ─────────────────────────────────────────────────────

    def _factory_provision(self):
        """Flujo completo de provisioning de fábrica:
        1. Lee MAC del dispositivo vía esptool
        2. Lee Flash Chip ID vía esptool
        3. Genera token aleatorio + hash bcrypt
        4. Registra en backend (/api/devices/register_factory)
        5. Escribe token + serial en NVS (0x9000)
        6. Muestra serial resultante en el log
        """
        if self._busy:
            return
        port = self._port_var.get()
        if not port:
            messagebox.showwarning("Puerto requerido",
                                   "Selecciona el puerto COM del dispositivo.")
            return
        if not self._esptool:
            messagebox.showerror("Error",
                                 "esptool no encontrado.\n"
                                 "Instala Arduino IDE 2.x o pip install esptool.")
            return

        self._clear_log()
        self._set_busy(True)

        profile_label = self._profile_var.get()
        _, ver_label = self._get_selected_ref()

        def run():
            try:
                backend_url = self._backend_var.get().strip().rstrip("/")

                self._log_line("─" * 60, "#444")
                self._log_line("  FACTORY PROVISION", "#dcdcaa")
                self._log_line(f"  Perfil:   {profile_label}", "#888")
                self._log_line(f"  Firmware: {ver_label}", "#888")
                self._log_line("─" * 60, "#444")

                # 1. Leer MAC
                self._log_line("\n● Leyendo MAC del dispositivo...", "#569cd6")
                mac = fp_read_mac(self._esptool, port)
                if not mac:
                    self._log_line("✗  No se pudo leer la MAC. Comprueba el puerto y el bootloader.", "#f44747")
                    return
                self._log_line(f"  MAC: {mac}", "#4ec9b0")

                # 2. Construir serial (AQ-{MAC sin dos puntos})
                mac_clean = mac.replace(":", "")
                serial = f"AQ-{mac_clean}"
                self._log_line(f"  Serial: {serial}", "#dcdcaa")

                # 3. Generar token
                self._log_line("\n● Generando token...", "#569cd6")
                token = fp_generate_token()
                token_hash = fp_hash_token(token)
                if not token_hash:
                    self._log_line("✗  bcrypt no instalado. Ejecuta: pip install bcrypt", "#f44747")
                    return
                self._log_line(f"  Token: {token[:12]}…  (hash bcrypt generado)", "#4ec9b0")

                # 4. Registrar en backend
                self._log_line(f"\n● Registrando en backend ({backend_url})...", "#569cd6")
                backend_ok = False
                try:
                    result = fp_register_backend(mac, token_hash, serial, backend_url)
                    self._log_line(f"  Backend: {result}", "#4ec9b0")
                    backend_ok = True
                except Exception as e:
                    self._log_line(f"⚠  Backend no disponible: {e}", "#f0a000")
                    self._log_line("   NVS se escribirá igualmente.", "#888")
                    self._log_line("   Registra en backend cuando el servidor esté online.", "#888")

                # 5. Escribir NVS
                self._log_line(f"\n● Escribiendo NVS en {port} (0x9000)...", "#569cd6")
                try:
                    fp_write_nvs(self._esptool, self._nvs_gen, port, serial, token)
                    self._log_line("✓  NVS escrito correctamente.", "#4ec9b0")
                except Exception as e:
                    self._log_line(f"✗  Error NVS: {e}", "#f44747")
                    return

                # 6. Guardar en registro local CSV
                try:
                    fp_save_registry(serial, mac, profile_label, ver_label)
                    self._log_line(f"✓  Registro guardado: {REGISTRY_FILE}", "#4ec9b0")
                except Exception as e:
                    self._log_line(f"⚠  No se pudo guardar el registro: {e}", "#f0a000")

                # 7. Resultado final
                self._log_line("\n" + "═" * 60, "#444")
                self._log_line("  DISPOSITIVO PROVISIONADO" + ("" if backend_ok else "  (pendiente registro backend)"), "#4ec9b0")
                self._log_line(f"  Serial:   {serial}", "#dcdcaa")
                self._log_line(f"  MAC:      {mac}", "#888")
                self._log_line(f"  Perfil:   {profile_label}", "#888")
                self._log_line(f"  Firmware: {ver_label}", "#888")
                self._log_line(f"  Registro: {REGISTRY_FILE}", "#888")
                self._log_line("═" * 60 + "\n", "#444")
                self._set_status(f"Provisionado: {serial}")
                self.after(100, lambda s=serial, m=mac: show_qr_window(s, m))

            except Exception as e:
                self._log_line(f"\n✗  Error inesperado: {e}", "#f44747")
                self._set_status("Error en factory provision")
            finally:
                self._set_busy(False)

        threading.Thread(target=run, daemon=True).start()

    # ── Borrar NVS ────────────────────────────────────────────────────────────

    def _erase_nvs(self):
        """
        Flujo en dos fases:
        1. Hilo background: lee la tabla de particiones real del chip.
        2. Hilo principal:  muestra confirmación con offset/size reales,
                            luego lanza el borrado si el usuario acepta.
        """
        if self._busy:
            return
        port = self._port_var.get()
        if not port:
            messagebox.showwarning("Puerto requerido",
                                   "Selecciona el puerto COM del dispositivo.")
            return
        if not self._esptool:
            messagebox.showerror("Error",
                                 "esptool no encontrado.\n"
                                 "Instala Arduino IDE 2.x o pip install esptool.")
            return

        self._clear_log()
        self._set_busy(True)

        def discover():
            """Fase 1 — lee la tabla de particiones (background)."""
            self._start_progress("indeterminate")
            self._log_line("─" * 60, "#444")
            self._log_line("  BORRAR NVS — leyendo tabla de particiones...", "#dcdcaa")
            self._log_line(f"  Puerto: {port}", "#888")
            self._log_line("─" * 60, "#444")
            self._set_status("Leyendo tabla de particiones del chip...")

            partitions = read_partition_table(self._esptool, port)
            nvs_part   = find_nvs_partition(partitions) if partitions else None

            # Volver al hilo principal para el diálogo de confirmación
            self.after(0, lambda: self._erase_nvs_confirm(port, partitions, nvs_part))

        threading.Thread(target=discover, daemon=True).start()

    def _erase_nvs_confirm(self, port, partitions, nvs_part):
        """Fase 2 — muestra confirmación con datos reales (hilo principal)."""
        self._stop_progress(success=True)

        if partitions is None:
            # No se pudo leer la tabla: advertir y ofrecer cancelar
            msg = (
                "No se pudo leer la tabla de particiones del chip.\n\n"
                "Posibles causas:\n"
                "  • El dispositivo no está en modo bootloader\n"
                "  • Puerto COM incorrecto\n\n"
                "Revisa la conexión y vuelve a intentarlo."
            )
            messagebox.showerror("Error leyendo particiones", msg)
            self._set_status("Error leyendo tabla de particiones")
            self._set_busy(False)
            return

        # Mostrar todas las particiones encontradas en el log
        self._log_line("\n● Tabla de particiones del chip:", "#569cd6")
        for p in partitions:
            marker = " ◀ NVS" if (p["type"] == _PART_TYPE_DATA and
                                   p["subtype"] == _PART_SUBTYPE_NVS) else ""
            self._log_line(
                f"   {p['name']:<16}  type=0x{p['type']:02X}  "
                f"sub=0x{p['subtype']:02X}  "
                f"offset=0x{p['offset']:05X}  "
                f"size=0x{p['size']:05X}{marker}",
                "#dcdcaa" if marker else "#888"
            )

        if nvs_part is None:
            messagebox.showerror(
                "NVS no encontrada",
                "La tabla de particiones del chip no contiene una partición NVS\n"
                "(type=0x01, subtype=0x02).\n\n"
                "Comprueba el log para ver las particiones detectadas."
            )
            self._set_status("Partición NVS no encontrada en este chip")
            self._set_busy(False)
            return

        offset_hex = f"0x{nvs_part['offset']:05X}"
        size_hex   = f"0x{nvs_part['size']:05X}"
        size_kb    = nvs_part['size'] // 1024

        confirmed = messagebox.askyesno(
            "Confirmar borrado NVS",
            f"¿Borrar la partición NVS del dispositivo?\n\n"
            f"Partición detectada en el chip:\n"
            f"  Nombre:  {nvs_part['name']}\n"
            f"  Offset:  {offset_hex}\n"
            f"  Tamaño:  {size_hex}  ({size_kb} KB)\n\n"
            "Esto eliminará:\n"
            "  • Credenciales WiFi configuradas\n"
            "  • Token MQTT de fábrica\n"
            "  • Serial number almacenado\n\n"
            "El dispositivo arrancará en modo SoftAP la próxima vez.\n\n"
            f"Puerto: {port}",
            icon="warning",
        )
        if not confirmed:
            self._log_line("\n  Borrado cancelado por el usuario.", "#888")
            self._set_status("Borrado NVS cancelado")
            self._set_busy(False)
            return

        # Fase 3 — borrado real (background)
        threading.Thread(
            target=self._erase_nvs_run,
            args=(port, nvs_part["offset"], nvs_part["size"]),
            daemon=True,
        ).start()

    def _erase_nvs_run(self, port, offset, size):
        """Fase 3 — ejecuta esptool erase_region con los valores reales del chip."""
        try:
            self._start_progress("indeterminate")
            self._log_line(
                f"\n● Borrando NVS: offset={hex(offset)}  size={hex(size)}...",
                "#569cd6"
            )
            self._set_status("Borrando NVS...")

            cmd = ([sys.executable, self._esptool]
                   if self._esptool.endswith(".py") else [self._esptool])
            cmd += ["--port", port, "erase_region", hex(offset), hex(size)]

            ok, _ = self._run_cmd(cmd)
            self._stop_progress(ok)
            if ok:
                self._log_line("\n✓  NVS borrada correctamente.", "#4ec9b0")
                self._log_line("   El dispositivo arrancará en modo SoftAP la próxima vez.", "#888")
                self._set_status("NVS borrada — configura WiFi por SoftAP o re-provisiona")
            else:
                self._log_line("\n✗  Error al borrar NVS.", "#f44747")
                self._log_line("   Asegúrate de que el dispositivo está en modo bootloader.", "#888")
                self._set_status("Error al borrar NVS")
        except Exception as e:
            self._log_line(f"\n✗  Error inesperado: {e}", "#f44747")
            self._set_status("Error al borrar NVS")
        finally:
            self._set_busy(False)


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = FlasherApp()
    app.mainloop()
