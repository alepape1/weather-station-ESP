#!/bin/bash
# ota_flash.sh — Flashea el ESP32 por OTA usando el .bin compilado en Windows
# No requiere instalar Arduino en WSL. Solo Python3 (ya incluido en WSL).
#
# Uso:
#   ./ota_flash.sh              → usa IP por defecto (192.168.1.13)
#   ./ota_flash.sh 192.168.1.X  → IP personalizada

set -e

# ── Configuración ─────────────────────────────────────────────────────────────
ESP_IP="${1:-192.168.1.13}"
ESP_PORT=3232
SKETCH="ESP_monitor_server"
BOARD_DIR="esp32.esp32.lilygo_t_display"
WINDOWS_USER="Perfilador ResCoast"

WIN_SKETCH_PATH="/mnt/c/Users/${WINDOWS_USER}/Homeserver_Nextcloud/Documents/Documentos Alejandro/Mis repositorios/Mis repos favoritos/weather-station-ESP/ESP_monitor_server"
WIN_BIN="${WIN_SKETCH_PATH}/build/${BOARD_DIR}/${SKETCH}.ino.bin"

ARDUINO15="/mnt/c/Users/${WINDOWS_USER}/AppData/Local/Arduino15"

# ── Buscar espota.py en la instalación de Windows ─────────────────────────────
echo "=== MeteoStation OTA Flash ==="
echo "Target: ${ESP_IP}:${ESP_PORT}"
echo ""

ESPOTA=$(find "${ARDUINO15}/packages/esp32/hardware/esp32" -name "espota.py" 2>/dev/null | sort -V | tail -1)

if [ -z "$ESPOTA" ]; then
    echo "ERROR: espota.py no encontrado en ${ARDUINO15}"
    echo "Asegúrate de tener el core de ESP32 instalado en Arduino IDE de Windows."
    exit 1
fi
echo "espota.py : ${ESPOTA}"

# ── Verificar que el .bin existe ───────────────────────────────────────────────
if [ ! -f "$WIN_BIN" ]; then
    echo ""
    echo "ERROR: No se encontró el .bin compilado:"
    echo "  ${WIN_BIN}"
    echo ""
    echo "Compila primero desde Arduino IDE en Windows:"
    echo "  Sketch → Exportar binario compilado  (Ctrl+Alt+S)"
    exit 1
fi

BIN_SIZE=$(du -h "$WIN_BIN" | cut -f1)
echo "Firmware  : ${WIN_BIN}"
echo "Tamaño    : ${BIN_SIZE}"
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
python3 "$ESPOTA" -i "$ESP_IP" -p "$ESP_PORT" -f "$WIN_BIN"

echo ""
echo "Listo. El ESP32 se reiniciará automáticamente."
