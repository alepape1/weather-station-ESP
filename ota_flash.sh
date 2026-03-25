#!/bin/bash
# ota_flash.sh — Compila el sketch con arduino-cli y flashea el ESP32 por OTA
#
# Uso:
#   ./ota_flash.sh              → usa IP por defecto (192.168.1.9)
#   ./ota_flash.sh 192.168.1.X  → IP personalizada

set -e

# ── Configuración ──────────────────────────────────────────────────────────────
ESP_IP="${1:-192.168.1.9}"
ESP_PORT=3232
FQBN="esp32:esp32:lilygo_t_display"
SKETCH_DIR="$(cd "$(dirname "$0")/ESP_monitor_server" && pwd)"
BUILD_DIR="${TEMP:-/tmp}/esp_build"
SKETCH="ESP_monitor_server"
BIN="${BUILD_DIR}/${SKETCH}.ino.bin"

# arduino-cli incluido en Arduino IDE
ARDUINO_CLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"

# Paquetes esp32 (detecta la versión instalada automáticamente)
ESP32_HW="/c/Users/$(whoami)/AppData/Local/Arduino15/packages/esp32/hardware/esp32"
ESP32_VER=$(ls "$ESP32_HW" 2>/dev/null | sort -V | tail -1)
LIB_DIR="${ESP32_HW}/${ESP32_VER}/libraries"
ESPOTA="${ESP32_HW}/${ESP32_VER}/tools/espota.py"

# Python (busca py, python, python3 en ese orden)
PYTHON=""
for cmd in py python python3; do
    if command -v "$cmd" &>/dev/null; then
        PYTHON="$cmd"
        break
    fi
done
if [ -z "$PYTHON" ]; then
    # Busca Python instalado en AppData
    PYTHON=$(find "/c/Users/$(whoami)/AppData/Local/Programs/Python" -name "python.exe" 2>/dev/null | head -1)
fi
if [ -z "$PYTHON" ]; then
    echo "ERROR: No se encontró Python. Instálalo desde python.org"
    exit 1
fi

echo ""
echo "=== MeteoStation OTA Flash ==="
echo "Target    : ${ESP_IP}:${ESP_PORT}"
echo "Board     : ${FQBN}"
echo "Sketch    : ${SKETCH_DIR}"
echo "Build dir : ${BUILD_DIR}"
echo "Python    : ${PYTHON}"
echo ""

mkdir -p "$BUILD_DIR"

# ── Compilar ───────────────────────────────────────────────────────────────────
echo "Compilando firmware..."
"$ARDUINO_CLI" compile \
    --fqbn "$FQBN" \
    --libraries "$LIB_DIR" \
    --output-dir "$BUILD_DIR" \
    "$SKETCH_DIR"

if [ ! -f "$BIN" ]; then
    echo "ERROR: No se generó el .bin en ${BUILD_DIR}"
    exit 1
fi

BIN_SIZE=$(du -h "$BIN" | cut -f1)
echo ""
echo "Firmware compilado: ${BIN} (${BIN_SIZE})"
echo ""

# ── Verificar conectividad ─────────────────────────────────────────────────────
echo -n "Comprobando conexión con ${ESP_IP}... "
if ping -n 1 "$ESP_IP" > /dev/null 2>&1 || ping -c 1 -W 2 "$ESP_IP" > /dev/null 2>&1; then
    echo "OK"
else
    echo "FALLO"
    echo "El ESP32 no responde en ${ESP_IP}. Comprueba que está encendido y en la red."
    exit 1
fi

# ── Flash OTA ──────────────────────────────────────────────────────────────────
echo ""
echo "Iniciando carga OTA..."
"$PYTHON" "$ESPOTA" -i "$ESP_IP" -p "$ESP_PORT" -f "$BIN"

echo ""
echo "Listo. El ESP32 se reiniciará automáticamente."
