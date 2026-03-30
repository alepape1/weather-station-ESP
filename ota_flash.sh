#!/bin/bash
# ota_flash.sh — Compila el sketch con arduino-cli y flashea el ESP32 por OTA
#
# Uso:
#   ./ota_flash.sh                        → compila + flashea (IP por defecto 192.168.1.9)
#   ./ota_flash.sh 192.168.1.X            → compila + flashea a IP personalizada
#   ./ota_flash.sh --upload-only          → solo flashea el último .bin compilado
#   ./ota_flash.sh --upload-only 192.168.1.X → idem con IP personalizada

set -e

# ── Parseo de argumentos ───────────────────────────────────────────────────────
COMPILE=true
ESP_IP="192.168.1.9"
for arg in "$@"; do
    if [[ "$arg" == "--upload-only" ]]; then
        COMPILE=false
    elif [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        ESP_IP="$arg"
    fi
done

ESP_PORT=3232
FQBN="esp32:esp32:lilygo_t_display"
SKETCH_DIR="$(cd "$(dirname "$0")/ESP_monitor_server" && pwd)"
SKETCH="ESP_monitor_server"

# ── Detección de OS y rutas ────────────────────────────────────────────────────
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || -n "$WINDIR" ]]; then
    # Windows (Git Bash / MSYS2)
    BUILD_DIR="$LOCALAPPDATA/Temp/esp_ota_build"
    ARDUINO_CLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
    PYTHON="/c/Users/Perfilador ResCoast/AppData/Local/Programs/Python/Python312/python.exe"
    ARDUINO15="$LOCALAPPDATA/Arduino15"
else
    # Linux / macOS
    BUILD_DIR="/tmp/esp_ota_build"
    # arduino-cli: buscar en rutas comunes
    if command -v arduino-cli &>/dev/null; then
        ARDUINO_CLI="arduino-cli"
    elif [ -f "$HOME/.local/bin/arduino-cli" ]; then
        ARDUINO_CLI="$HOME/.local/bin/arduino-cli"
    elif [ -f "/usr/local/bin/arduino-cli" ]; then
        ARDUINO_CLI="/usr/local/bin/arduino-cli"
    else
        echo "ERROR: arduino-cli no encontrado. Instálalo con:"
        echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
        exit 1
    fi
    # Python 3
    if command -v python3 &>/dev/null; then
        PYTHON="python3"
    else
        PYTHON="python"
    fi
    ARDUINO15="$HOME/.arduino15"
fi

BIN="${BUILD_DIR}/${SKETCH}.ino.bin"

# espota.py — detecta la versión del core ESP32 instalada automáticamente
ESP32_VER=$(ls "$ARDUINO15/packages/esp32/hardware/esp32/" 2>/dev/null | sort -V | tail -1)
if [ -z "$ESP32_VER" ]; then
    echo "ERROR: Core esp32 no encontrado en $ARDUINO15"
    echo "Instálalo con: arduino-cli core install esp32:esp32"
    exit 1
fi
ESPOTA="$ARDUINO15/packages/esp32/hardware/esp32/${ESP32_VER}/tools/espota.py"

echo "=== MeteoStation OTA Flash ==="
echo "Target  : ${ESP_IP}:${ESP_PORT}"
echo "Board   : ${FQBN}"
echo "Sketch  : ${SKETCH_DIR}"
echo "Core    : esp32 ${ESP32_VER}"
echo ""

# ── Compilar ──────────────────────────────────────────────────────────────────
if [ "$COMPILE" = true ]; then
    echo "Compilando con arduino-cli..."
    mkdir -p "$BUILD_DIR"
    "$ARDUINO_CLI" compile \
        --fqbn "$FQBN" \
        --build-path "$BUILD_DIR" \
        "$SKETCH_DIR"
else
    echo "Compilación omitida (--upload-only)"
fi

if [ ! -f "$BIN" ]; then
    echo "ERROR: No se encontró el .bin en ${BUILD_DIR}"
    echo "Compila primero sin --upload-only"
    exit 1
fi

BIN_SIZE=$(du -h "$BIN" | cut -f1)
echo ""
echo "Firmware compilado: ${BIN} (${BIN_SIZE})"
echo ""

# ── Verificar conectividad ─────────────────────────────────────────────────────
echo -n "Comprobando conexión con ${ESP_IP}... "
if ping -c 1 -W 2 "$ESP_IP" > /dev/null 2>&1 || ping -n 1 -w 2000 "$ESP_IP" > /dev/null 2>&1; then
    echo "OK"
else
    echo "FALLO"
    echo "El ESP32 no responde. Comprueba que está encendido y en la red."
    exit 1
fi

# ── Flash OTA ─────────────────────────────────────────────────────────────────
echo ""
echo "Iniciando carga OTA..."
"$PYTHON" "$ESPOTA" -i "$ESP_IP" -p "$ESP_PORT" -f "$BIN"

echo ""
echo "Listo. El ESP32 se reiniciará automáticamente."
