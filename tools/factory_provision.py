#!/usr/bin/env python3
"""
factory_provision.py — Script de aprovisionamiento de fábrica para dispositivos Aquantia.

Uso:
    python factory_provision.py --mac AA:BB:CC:DD:EE:FF --serial AQ-FCB467F37748

Genera un token aleatorio, lo hashea con bcrypt y lo registra en el backend via
POST /api/devices/register_factory (solo aceptado desde localhost).

Salida:
    Imprime el token en claro para grabarlo en el ESP32 via esptool/NVS.
    También puede grabar directamente en el ESP32 usando esptool nvs_flash si
    se pasa --port /dev/ttyUSB0.
"""

import argparse
import hashlib
import json
import secrets
import subprocess
import sys
import urllib.request

BACKEND_URL = "http://127.0.0.1:7000"


def generate_token() -> str:
    """Genera un token URL-safe de 32 bytes (43 caracteres base64url)."""
    return secrets.token_urlsafe(32)


def hash_token(token: str) -> str:
    """Hashea el token con bcrypt (coste 12)."""
    try:
        import bcrypt
        hashed = bcrypt.hashpw(token.encode(), bcrypt.gensalt(rounds=12))
        return hashed.decode()
    except ImportError:
        print("ERROR: instala bcrypt → pip install bcrypt", file=sys.stderr)
        sys.exit(1)


def register_in_backend(mac: str, token_hash: str, serial_number: str) -> dict:
    """Llama a POST /api/devices/register_factory y devuelve la respuesta JSON."""
    payload = json.dumps({
        "mac": mac,
        "token_hash": token_hash,
        "serial_number": serial_number,
    }).encode()
    req = urllib.request.Request(
        f"{BACKEND_URL}/api/devices/register_factory",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        print(f"ERROR HTTP {e.code}: {body}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERROR conectando al backend: {e}", file=sys.stderr)
        sys.exit(1)


def flash_nvs(port: str, mac: str, serial_number: str, token: str,
              finca_id: str = "") -> None:
    """Graba las credenciales en la NVS del ESP32 usando esptool + nvs_partition_gen.

    Requiere: pip install esptool
    """
    import csv
    import os
    import tempfile

    # Generar CSV para nvs_partition_gen
    nvs_csv = [
        ["key", "type", "encoding", "value"],
        ["namespace", "namespace", "", "aquantia"],
        ["mqtt_token", "data", "string", token],
        ["serial_number", "data", "string", serial_number],
    ]
    if finca_id:
        nvs_csv.append(["finca_id", "data", "string", finca_id])

    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = os.path.join(tmpdir, "nvs.csv")
        bin_path = os.path.join(tmpdir, "nvs.bin")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(nvs_csv)

        # Generar partición NVS binaria
        result = subprocess.run(
            [sys.executable, "-m", "esp_idf_nvs_partition_gen",
             "--input", csv_path,
             "--output", bin_path,
             "--size", "0x6000"],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            # Intentar con nvs_partition_gen.py de IDF si está en PATH
            result = subprocess.run(
                ["nvs_partition_gen.py", "generate", csv_path, bin_path, "0x6000"],
                capture_output=True, text=True
            )
        if result.returncode != 0:
            print("AVISO: no se pudo generar NVS binario automáticamente.", file=sys.stderr)
            print("Graba manualmente las credenciales en el firmware (secrets.h).", file=sys.stderr)
            return

        # Flash NVS a dirección 0x9000 (partición nvs por defecto)
        result = subprocess.run(
            ["esptool.py", "--port", port, "write_flash", "0x9000", bin_path],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"NVS grabado en {port} correctamente.")
        else:
            print(f"ERROR al flashear NVS: {result.stderr}", file=sys.stderr)


def main():
    global BACKEND_URL
    parser = argparse.ArgumentParser(description="Aprovisionamiento de fábrica Aquantia")
    parser.add_argument("--mac", required=True,
                        help="MAC del ESP32 (ej: AA:BB:CC:DD:EE:FF)")
    parser.add_argument("--serial", required=False,
                        help="Número de serie del dispositivo (ej: AQ-FCB467F37748)")
    parser.add_argument("--finca-id", default="",
                        help="finca_id a asignar (opcional, normalmente lo elige el usuario)")
    parser.add_argument("--port", default="",
                        help="Puerto serie del ESP32 para flash NVS (ej: /dev/ttyUSB0)")
    parser.add_argument("--backend-url", default=BACKEND_URL,
                        help=f"URL del backend (defecto: {BACKEND_URL})")
    args = parser.parse_args()

    BACKEND_URL = args.backend_url.rstrip("/")

    mac = args.mac.upper().replace("-", ":")
    if args.serial:
        serial_number = args.serial.upper()
    else:
        serial_number = f"AQ-{mac.replace(":", "")}"

    print(f"Aprovisionando dispositivo: MAC={mac}  SN={serial_number}")

    token = generate_token()
    print(f"Token generado:  {token}")

    token_hash = hash_token(token)
    print(f"Hash bcrypt:     {token_hash[:30]}…")

    result = register_in_backend(mac, token_hash, serial_number)
    print(f"Backend:         {result}")

    print()
    print("=" * 60)
    print("CREDENCIALES PARA GRABAR EN EL ESP32")
    print("=" * 60)
    print(f"  MAC:           {mac}")
    print(f"  Serial Number: {serial_number}")
    print(f"  MQTT Token:    {token}")
    if args.finca_id:
        print(f"  Finca ID:      {args.finca_id}")
    print("=" * 60)

    if args.port:
        print(f"\nFlasheando NVS en {args.port}…")
        flash_nvs(args.port, mac, serial_number, token, args.finca_id)
    else:
        print("\nPara flashear NVS automáticamente, usa: --port /dev/ttyUSB0")
        print("O copia el token en secrets.h del firmware manualmente.")


if __name__ == "__main__":
    main()
