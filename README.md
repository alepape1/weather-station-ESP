# MeteoStation — Firmware ESP v3

Firmware dual-plataforma para la estación meteorológica casera.
Compatible con **ESP32** (LilyGo TTGO T-Display, con pantalla TFT 240×135) y **ESP8266** (sin pantalla).

El ESP recoge datos de temperatura, humedad, presión, viento y luz; los muestra en pantalla (ESP32) y los envía cada 20 segundos al servidor Flask por HTTP.

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Hardware](#hardware)
- [Sensores I2C detectados en hardware real](#sensores-i2c-detectados-en-hardware-real)
- [Librerías necesarias](#librerías-necesarias)
- [Configuración antes de compilar](#configuración-antes-de-compilar)
- [Flashear por OTA desde WSL](#flashear-por-ota-desde-wsl)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Simulación de sensores](#simulación-de-sensores)
- [Algoritmos](#algoritmos)
- [Formato de datos enviados al servidor](#formato-de-datos-enviados-al-servidor)
- [Pantalla TFT](#pantalla-tft)
- [Problemas conocidos](#problemas-conocidos)

---

## Hardware

| Componente | Modelo | Notas |
|------------|--------|-------|
| Microcontrolador | LilyGo TTGO T-Display (ESP32-D0WDQ6) | 240 MHz, 16 MB flash, pantalla integrada ST7789 240×135 |
| Temperatura exterior | Adafruit MCP9808 | Alta precisión ±0.25°C, I2C 0x19 |
| Temperatura + humedad | HTU2x (HTU21D / HTU20D) | Sensor principal, I2C 0x40 |
| Temperatura + humedad (2) | DHT11 | Sensor secundario, pin digital |
| Barómetro | SparkFun MicroPressure | I2C 0x18, rango 0–25 PSI |
| Luz ambiente | APDS-9930 / TSL2584 (clon) | Autodetección, I2C 0x39 |
| Anemómetro | Analógico 0–3.3 V | 0–30 m/s, GPIO37 (ESP32) / A0 (ESP8266) |
| Veleta | Analógica 0–3.3 V | 0–360°, GPIO36 (solo ESP32) |
| Pantalla | ST7789 240×135 integrada | Gestionada por TFT_eSPI |
| LED indicador | GPIO 2 onboard | Parpadea al enviar datos |

---

## Sensores I2C detectados en hardware real

Resultado del escáner I2C al arrancar con el hardware actual:

```
Dispositivo en 0x18  → SparkFun MicroPressure
Dispositivo en 0x19  → Adafruit MCP9808
Dispositivo en 0x39  → APDS-9930 clon (ID@0x12=0x98, fallback a registros APDS)
Dispositivo en 0x40  → HTU2x (HTU21D)
```

---

## Librerías necesarias

Instalar desde el gestor de librerías de Arduino IDE (o con arduino-cli):

| Librería | Autor | Uso |
|----------|-------|-----|
| `TFT_eSPI` | Bodmer | Pantalla ST7789 (solo ESP32) |
| `Adafruit MCP9808 Library` | Adafruit | Temperatura exterior |
| `SparkFun MicroPressure Library` | SparkFun | Barómetro |
| `DHTesp` | beegee-tokyo | DHT11 (sensor secundario) |

> HTU2x y el sensor de luz se implementan directamente sobre I2C sin librería externa.

### Configuración TFT_eSPI (ESP32)

Editar `User_Setup_Select.h` de la librería para activar la configuración del TTGO T-Display:

```cpp
// Comentar la línea por defecto:
//#include <User_Setup.h>

// Descomentar la del T-Display:
#include <User_Setups/Setup25_TTGO_T_Display.h>
```

---

## Configuración antes de compilar

Las credenciales de red se guardan en `secrets.h` (excluido del repositorio por `.gitignore`). Crear el archivo en la misma carpeta que el `.ino`:

```cpp
// secrets.h — NO subir al repositorio
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"
#define SERVER_IP     "192.168.1.X"   // IP del PC/RPi donde corre Flask
#define SERVER_PORT   7000

// Opcional: proteger OTA con contraseña
// #define OTA_PASSWORD  "mi_password_ota"
```

> Si Flask corre en WSL, usar la **IP WiFi de Windows** (no la de WSL). Con `networkingMode=mirrored` en `.wslconfig` la IP de WSL y Windows son la misma.

---

## Flashear por OTA desde WSL

El firmware incluye ArduinoOTA. Una vez flasheado por USB la primera vez, los siguientes updates se pueden hacer por WiFi.

### Requisitos previos

1. Regla de firewall en Windows (una sola vez):
```cmd
netsh advfirewall firewall add rule name="ESP32 OTA" dir=in action=allow protocol=TCP localport=10000-60000
```

2. WSL con `networkingMode=mirrored` en `C:\Users\<usuario>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```

### Compilar y flashear

```bash
# Compilar (arduino-cli en WSL)
arduino-cli compile --fqbn esp32:esp32:lilygo_t_display ESP_monitor_server/

# Flashear por OTA
python3 ~/.arduino15/packages/esp32/hardware/esp32/3.3.7/tools/espota.py \
  -i 192.168.1.13 -p 3232 \
  -I $(ip route get 192.168.1.13 | grep src | awk '{print $5}') \
  -r \
  -f ~/.cache/arduino/sketches/*/ESP_monitor_server.ino.bin
```

> También existe el script `ota_flash.sh` en la raíz del repo, que usa el binario compilado desde Arduino IDE en Windows.

---

## Arquitectura del firmware

El `loop()` tiene **tres bloques independientes** con `millis()`:

```
loop()
 ├─ ArduinoOTA.handle()            ← siempre primero
 ├─ Cada 100ms  → Leer ADC anemómetro/veleta, acumular vector de viento
 ├─ Cada 1s     → Leer MCP9808, HTU2x, barómetro, luz. Actualizar pantalla/serial
 └─ Cada 20s    → Calcular promedio vectorial, enviar HTTP POST al servidor
```

Al arrancar (`setup()`):
1. Escanear bus I2C y detectar sensores
2. Conectar WiFi
3. Iniciar ArduinoOTA
4. Imprimir info del hardware por serie
5. Enviar POST a `/api/device_info` con datos estáticos del chip

---

## Simulación de sensores

Si un sensor no responde en el `setup()` o falla durante el funcionamiento, el firmware **continúa en modo simulación** para ese sensor con valores que derivan suavemente:

| Sensor | Rango simulado | Drift/segundo |
|--------|---------------|---------------|
| MCP9808 (T.Ext) | −10 a 45 °C | ±0.05 °C |
| HTU2x (T.Int)   | −10 a 45 °C | ±0.05 °C |
| HTU2x (Humedad) | 20 a 95 %   | ±0.20 % |
| Barómetro       | 95 a 110 kPa | ±0.02 kPa |
| Luz             | 0 a 2000 lux | ±5 lux |

---

## Algoritmos

### Velocidad del viento — Media móvil (buffer circular de 10 muestras)

```cpp
float adcToWindSpeed(float adc) {
    float v = adc * (ADC_VOLTAGE_REF / ADC_RANGE);
    return (v / ADC_VOLTAGE_REF) * 30.0f; // 0–30 m/s
}
```

### Dirección del viento — Promedio vectorial

Media aritmética de ángulos es incorrecta cerca del norte (350° y 10° → 180° en vez de 0°). El firmware acumula vectores unitarios en cartesianas y calcula `atan2`:

```cpp
// Cada 100ms
windSumX += cos(deg * PI / 180);
windSumY += sin(deg * PI / 180);

// Cada 20s
float dir = atan2(windSumY, windSumX) * 180 / PI;
if (dir < 0) dir += 360;
```

### Sensor de luz — Autodetección TSL2584 / APDS-9930

El chip en 0x39 se identifica leyendo el registro de ID:
- `ID@0x12 & 0xF0 == 0x30` → APDS-9930 (registros 0x14–0x17)
- `ID@0x0A & 0xF0 == 0xA0` → TSL2584 (registros 0x0C–0x0F)
- Desconocido → fallback a APDS-9930

---

## Formato de datos enviados al servidor

### CSV periódico — `POST /send_message` (cada 20s)

14 valores separados por coma:

| Pos | Campo | Sensor | Unidad |
|-----|-------|--------|--------|
| 0 | `temperature` | MCP9808 | °C |
| 1 | `pressure` | MicroPressure | kPa* |
| 2 | `temperature_bar` | HTU2x | °C |
| 3 | `humidity` | HTU2x | % |
| 4 | `windSpeed` | Anemómetro | m/s |
| 5 | `windDirection` | Veleta | ° |
| 6 | `windSpeedFiltered` | Anemómetro | m/s |
| 7 | `windDirectionFiltered` | Veleta | ° |
| 8 | `light` | APDS-9930 | lux |
| 9 | `dht_temperature` | DHT11 | °C |
| 10 | `dht_humidity` | DHT11 | % |
| 11 | `rssi` | WiFi | dBm |
| 12 | `free_heap` | ESP32 | bytes |
| 13 | `uptime_s` | ESP32 | segundos |

> *La presión se envía en kPa (~101.3). Pendiente corregir a hPa con `readPressure(PA)/100.0`.

Ejemplo:
```
25.31,101.14,23.01,69.33,0.00,0.00,0.00,0.00,0.48,25.00,15.00,-62,142256,120
```

### Info estática del dispositivo — `POST /api/device_info` (al arrancar)

JSON enviado una vez tras conectar WiFi:

```json
{
  "chip_model": "ESP32-D0WDQ6",
  "chip_revision": 101,
  "cpu_freq_mhz": 240,
  "flash_size_mb": 16,
  "sdk_version": "v5.5.2-729-g87912cd291",
  "mac_address": "88:13:BF:FD:A2:38",
  "ip_address": "192.168.1.13"
}
```

---

## Pantalla TFT

Resolución: **240×135 px** (ST7789, LilyGo TTGO T-Display). Doble buffer con `TFT_eSprite` → `pushSprite()` atómico, sin parpadeo.

La pantalla se actualiza cada segundo:

```
┌────────────────────────────────────────┐
│ METEOSTATION   ☀0lx  WiFi  ●          │  ← cabecera (●verde=OK /rojo=error)
├──────────┬──────────┬──────────────────┤
│ T.EXT    │ T.INT    │ HUMEDAD          │
│ 25.3 C   │ 23.0 C   │ 69.3 %          │
│ [OK]     │ [OK]     │ [OK]            │
├──────────┼──────────┼──────────────────┤
│ PRESION  │ VIENTO   │ DIRECC.          │
│ 101.1KPa │ 0.0 m/s  │ 0 deg  N        │
│ [OK]     │          │                  │
└────────────────────────────────────────┘
```

Badge `[OK]` verde = sensor real. Badge `[SIM]` naranja = sensor simulado.

---

## Problemas conocidos

| Problema | Estado | Solución |
|----------|--------|----------|
| Presión en kPa en vez de hPa | Pendiente | Cambiar `readPressure(KPA)` → `readPressure(PA) / 100.0` |
| DHT11 lecturas inestables | Conocido | Valorar reemplazar por DHT22 o SHT31 |
