#!/bin/bash
# ota_flash.sh — Compila el sketch con arduino-cli y flashea los ESP32 por OTA
#
# Uso:
#   ./ota_flash.sh                    → compila + flashea METEO (192.168.1.9)
#   ./ota_flash.sh --meteo            → compila + flashea LilyGo TTGO (PROFILE_METEO)
#   ./ota_flash.sh --irrigation       → compila + flashea 4-Relay Board (PROFILE_IRRIGATION)
#   ./ota_flash.sh --all              → compila + flashea ambos dispositivos
#   ./ota_flash.sh --upload-only      → solo flashea el .bin ya compilado (METEO)
#   ./ota_flash.sh --upload-only --irrigation → idem para IRRIGATION
#   Cualquier argumento IP sobreescribe la IP por defecto del perfil seleccionado.

set -e

# ── Configuración de perfiles ──────────────────────────────────────────────────
# METEO    → LilyGo TTGO T-Display (pantalla ST7789, 1 relay)
# IRRIGATION → ESP32 4-Relay Board (ESP32-WROOM-32E, 4 relays, sin pantalla)
METEO_IP="192.168.1.9"
METEO_FQBN="esp32:esp32:lilygo_t_display"
METEO_PROFILE=1

IRRIGATION_IP="192.168.1.11"
IRRIGATION_FQBN="esp32:esp32:esp32"
IRRIGATION_PROFILE=2

ESP_PORT=3232
SKETCH_DIR="$(cd "$(dirname "$0")/ESP_monitor_server" && pwd)"
SKETCH="ESP_monitor_server"

# ── Parseo de argumentos ───────────────────────────────────────────────────────
COMPILE=true
MODE="meteo"          # meteo | irrigation | all
CUSTOM_IP=""

for arg in "$@"; do
    case "$arg" in
        --upload-only)   COMPILE=false ;;
        --meteo)         MODE="meteo" ;;
        --irrigation)    MODE="irrigation" ;;
        --all)           MODE="all" ;;
        [0-9]*.[0-9]*.[0-9]*.[0-9]*)  CUSTOM_IP="$arg" ;;
    esac
done

# ── Detección de OS y rutas ────────────────────────────────────────────────────
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || -n "$WINDIR" ]]; then
    BASE_BUILD_DIR="$LOCALAPPDATA/Temp/esp_ota_build"
    ARDUINO_CLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
    PYTHON="/c/Users/Perfilador ResCoast/AppData/Local/Programs/Python/Python312/python.exe"
    ARDUINO15="$LOCALAPPDATA/Arduino15"
else
    BASE_BUILD_DIR="/tmp/esp_ota_build"
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
    if command -v python3 &>/dev/null; then
        PYTHON="python3"
    else
        PYTHON="python"
    fi
    ARDUINO15="$HOME/.arduino15"
fi

# espota.py — detecta la versión del core ESP32 instalada automáticamente
ESP32_VER=$(ls "$ARDUINO15/packages/esp32/hardware/esp32/" 2>/dev/null | sort -V | tail -1)
if [ -z "$ESP32_VER" ]; then
    echo "ERROR: Core esp32 no encontrado en $ARDUINO15"
    echo "Instálalo con: arduino-cli core install esp32:esp32"
    exit 1
fi
ESPOTA="$ARDUINO15/packages/esp32/hardware/esp32/${ESP32_VER}/tools/espota.py"

# ── Funciones ──────────────────────────────────────────────────────────────────

check_ping() {
    local ip="$1"
    if ping -c 1 -W 2 "$ip" > /dev/null 2>&1 || ping -n 1 -w 2000 "$ip" > /dev/null 2>&1; then
        return 0
    fi
    return 1
}

compile_profile() {
    local profile_name="$1"   # "METEO" o "IRRIGATION"
    local fqbn="$2"
    local profile_num="$3"
    local build_dir="${BASE_BUILD_DIR}_${profile_name,,}"

    echo ""
    echo "┌─ Compilando ${profile_name} ──────────────────────────────────────"
    echo "│  FQBN           : ${fqbn}"
    echo "│  DEVICE_PROFILE : ${profile_num}"
    echo "│  Build dir      : ${build_dir}"
    echo "└───────────────────────────────────────────────────────────────────"
    mkdir -p "$build_dir"
    "$ARDUINO_CLI" compile \
        --fqbn "$fqbn" \
        --build-path "$build_dir" \
        --build-property "compiler.cpp.extra_flags=-DDEVICE_PROFILE=${profile_num}" \
        "$SKETCH_DIR"
    echo "Compilación ${profile_name} completada."
}

flash_device() {
    local profile_name="$1"
    local ip="$2"
    local build_dir="${BASE_BUILD_DIR}_${profile_name,,}"
    local bin="${build_dir}/${SKETCH}.ino.bin"

    if [ ! -f "$bin" ]; then
        echo "ERROR: No se encontró el .bin para ${profile_name} en ${build_dir}"
        echo "Compila primero sin --upload-only"
        exit 1
    fi

    local bin_size
    bin_size=$(du -h "$bin" | cut -f1)

    echo ""
    echo "┌─ Flasheando ${profile_name} → ${ip}:${ESP_PORT} ─────────────────"
    echo "│  Firmware : ${bin} (${bin_size})"
    echo "└───────────────────────────────────────────────────────────────────"

    echo -n "Comprobando conexión con ${ip}... "
    if check_ping "$ip"; then
        echo "OK"
    else
        echo "FALLO"
        echo "El ESP32 (${profile_name}) no responde en ${ip}."
        echo "Comprueba que está encendido y conectado a la red."
        exit 1
    fi

    echo "Iniciando carga OTA..."
    "$PYTHON" "$ESPOTA" -i "$ip" -p "$ESP_PORT" -f "$bin"
    echo "${profile_name} flasheado. El ESP32 se reiniciará automáticamente."
}

# ── Ejecución ──────────────────────────────────────────────────────────────────
echo "=== MeteoStation OTA Flash ==="
echo "Modo    : ${MODE}"
echo "Sketch  : ${SKETCH_DIR}"
echo "Core    : esp32 ${ESP32_VER}"

case "$MODE" in

    meteo)
        IP="${CUSTOM_IP:-$METEO_IP}"
        if [ "$COMPILE" = true ]; then
            compile_profile "METEO" "$METEO_FQBN" "$METEO_PROFILE"
        else
            echo "Compilación omitida (--upload-only)"
        fi
        flash_device "METEO" "$IP"
        ;;

    irrigation)
        IP="${CUSTOM_IP:-$IRRIGATION_IP}"
        if [ "$COMPILE" = true ]; then
            compile_profile "IRRIGATION" "$IRRIGATION_FQBN" "$IRRIGATION_PROFILE"
        else
            echo "Compilación omitida (--upload-only)"
        fi
        flash_device "IRRIGATION" "$IP"
        ;;

    all)
        if [ -n "$CUSTOM_IP" ]; then
            echo "AVISO: IP personalizada ignorada en modo --all. Usa --meteo o --irrigation."
        fi
        if [ "$COMPILE" = true ]; then
            compile_profile "METEO"      "$METEO_FQBN"      "$METEO_PROFILE"
            compile_profile "IRRIGATION" "$IRRIGATION_FQBN" "$IRRIGATION_PROFILE"
        else
            echo "Compilación omitida (--upload-only)"
        fi
        flash_device "METEO"      "$METEO_IP"
        flash_device "IRRIGATION" "$IRRIGATION_IP"
        echo ""
        echo "=== Ambos dispositivos actualizados ==="
        ;;
esac

echo ""
echo "Listo."
