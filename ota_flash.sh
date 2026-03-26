#!/bin/bash
# ota_flash.sh — Compila el sketch con arduino-cli y flashea el ESP32 por OTA
#
# Uso:
#   ./ota_flash.sh              → usa IP por defecto (192.168.1.13)
#   ./ota_flash.sh 192.168.1.X  → IP personalizada

set -e

# ── Configuración ─────────────────────────────────────────────────────────────
ESP_IP="${1:-192.168.1.9}"
ESP_PORT=3232
FQBN="esp32:esp32:lilygo_t_display"
SKETCH_DIR="$(cd "$(dirname "$0")/ESP_monitor_server" && pwd)"
BUILD_DIR="/tmp/esp_build"
SKETCH="ESP_monitor_server"
BIN="${BUILD_DIR}/${SKETCH}.ino.bin"
ESPOTA="$HOME/.arduino15/packages/esp32/hardware/esp32/$(ls "$HOME/.arduino15/packages/esp32/hardware/esp32/" | sort -V | tail -1)/tools/espota.py"

echo "=== MeteoStation OTA Flash ==="
echo "Target  : ${ESP_IP}:${ESP_PORT}"
echo "Board   : ${FQBN}"
echo "Sketch  : ${SKETCH_DIR}"
echo ""

# ── Compilar ──────────────────────────────────────────────────────────────────
echo "Compilando con arduino-cli..."
arduino-cli compile \
    --fqbn "$FQBN" \
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
if ping -c 1 -W 2 "$ESP_IP" > /dev/null 2>&1; then
    echo "OK"
else
    echo "FALLO"
    echo "El ESP32 no responde. Comprueba que está encendido y en la red."
    exit 1
fi

# ── Flash OTA ─────────────────────────────────────────────────────────────────
echo ""
echo "Iniciando carga OTA..."
python3 "$ESPOTA" -i "$ESP_IP" -p "$ESP_PORT" -f "$BIN"

echo ""
echo "Listo. El ESP32 se reiniciará automáticamente."
