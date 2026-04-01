# MeteoStation — Firmware ESP v4

Firmware para la estación meteorológica doméstica Aquantia. Compatible con **dos perfiles de ESP32** y preparado para un tercer nodo ESP8266 independiente.

El ESP recoge datos de sensores y los envía cada 20 segundos al servidor Flask por **HTTPS** a [meteo.aquantialab.com](https://meteo.aquantialab.com). Incluye control de **relay(s) para electroválvula(s)** con polling cada 2s desde el dashboard.

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Dispositivos y perfiles](#dispositivos-y-perfiles)
- [Hardware](#hardware)
- [Sensores I2C detectados en hardware real](#sensores-i2c-detectados-en-hardware-real)
- [Librerías necesarias](#librerías-necesarias)
- [Configuración antes de compilar](#configuración-antes-de-compilar)
- [Flashear por OTA](#flashear-por-ota)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Relay electroválvula(s)](#relay-electroválvulas)
- [Filtros y estabilización de sensores](#filtros-y-estabilización-de-sensores)
- [Ahorro energético](#ahorro-energético)
- [Simulación de sensores](#simulación-de-sensores)
- [Algoritmos](#algoritmos)
- [Formato de datos enviados al servidor](#formato-de-datos-enviados-al-servidor)
- [Pantalla TFT](#pantalla-tft)
- [Problemas conocidos](#problemas-conocidos)

---

## Dispositivos y perfiles

El firmware compila un binario distinto para cada dispositivo usando `DEVICE_PROFILE`:

| Perfil | Dispositivo | IP | Relays | Pantalla | Suelo |
|--------|------------|-----|--------|----------|-------|
| `PROFILE_METEO` (1) | LilyGo TTGO T-Display | 192.168.1.9 | 1 × GPIO26 | ST7789 240×135 | YL-69 GPIO33 |
| `PROFILE_IRRIGATION` (2) | ESP32 4-Relay Board | 192.168.1.11 | 4 × GPIO32/33/25/26 | No | No |

El `DEVICE_PROFILE` se pasa en tiempo de compilación — el script `ota_flash.sh` lo gestiona automáticamente. No hay que editar el código fuente entre compilaciones.

---

## Hardware

### PROFILE_METEO — LilyGo TTGO T-Display

| Componente | Modelo | Notas |
|------------|--------|-------|
| Microcontrolador | LilyGo TTGO T-Display (ESP32-D0WD-V3) | 240 MHz, 16 MB flash, pantalla integrada ST7789 240×135 |
| Temperatura exterior | Adafruit MCP9808 | Alta precisión ±0.25°C, I2C 0x19 |
| Temperatura + humedad | HTU2x (HTU21D / HTU20D) | Sensor principal, I2C 0x40 |
| Temperatura + humedad (2) | DHT11 | Sensor secundario, GPIO15 |
| Barómetro | SparkFun MicroPressure | I2C 0x18, rango 0–25 PSI |
| Luz ambiente | APDS-9930 / TSL2584 (clon) | Autodetección, I2C 0x39 |
| Anemómetro | Analógico 0–3.3 V | 0–30 m/s, GPIO37 |
| Veleta | Analógica 0–3.3 V | 0–360°, GPIO36 |
| Humedad suelo | YL-69 | ADC GPIO33, filtro 10 muestras |
| Pantalla | ST7789 240×135 integrada | Gestionada por TFT_eSPI |
| LED indicador | GPIO2 onboard | Parpadea al enviar datos |
| Relay | JQC-3FF-S-Z (activo-LOW) | Electroválvula; GPIO26 |
| Botones | BOOT (GPIO0) + GPIO35 | Encienden pantalla y resetean timer de apagado |

### PROFILE_IRRIGATION — ESP32 4-Relay Board

| Componente | Modelo | Notas |
|------------|--------|-------|
| Microcontrolador | ESP32-WROOM-32E (ESP32-D0WD-V3) | 240 MHz, sin pantalla |
| Relay 1 | JQC-3FF-S-Z (activo-LOW) | GPIO32 |
| Relay 2 | JQC-3FF-S-Z (activo-LOW) | GPIO33 |
| Relay 3 | JQC-3FF-S-Z (activo-LOW) | GPIO25 |
| Relay 4 | JQC-3FF-S-Z (activo-LOW) | GPIO26 |
| LED estado | Onboard | GPIO23 |
| Alimentación | 7–30 VDC / 120–220 VAC / 5 VDC | Conectores separados |

> Referencia de placa: [ESP32 Relay x4 — ESPHome devices](https://devices.esphome.io/devices/ESP32-Relay-x4/)

---

## Sensores I2C detectados en hardware real

Resultado del escáner I2C al arrancar (PROFILE_METEO):

```
Dispositivo en 0x18  → SparkFun MicroPressure
Dispositivo en 0x19  → Adafruit MCP9808
Dispositivo en 0x39  → APDS-9930 clon (ID@0x12=0x98, fallback a registros APDS)
Dispositivo en 0x40  → HTU2x (HTU21D)
```

---

## Librerías necesarias

Instalar desde el gestor de librerías de Arduino IDE:

| Librería | Autor | Uso |
|----------|-------|-----|
| `TFT_eSPI` | Bodmer | Pantalla ST7789 (solo PROFILE_METEO) |
| `Adafruit MCP9808 Library` | Adafruit | Temperatura exterior |
| `SparkFun MicroPressure Library` | SparkFun | Barómetro |
| `DHTesp` | beegee-tokyo | DHT11 |
| `ArduinoJson` | Benoit Blanchon | Payload `/api/device_info` |

> HTU2x y el sensor de luz se implementan directamente sobre I2C sin librería externa.

### Configuración TFT_eSPI

Editar `User_Setup_Select.h` de la librería para activar la configuración del TTGO T-Display:

```cpp
//#include <User_Setup.h>                         // comentar línea por defecto
#include <User_Setups/Setup25_TTGO_T_Display.h>  // activar T-Display
```

---

## Configuración antes de compilar

Crear `secrets.h` en la misma carpeta que el `.ino` (excluido del repo por `.gitignore`):

```cpp
// secrets.h — NO subir al repositorio
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"
#define SERVER_IP     "meteo.aquantialab.com"
#define SERVER_PORT   443

// Perfil de dispositivo (el script ota_flash.sh lo pasa automáticamente)
// Si compilas desde Arduino IDE manualmente, descomenta el perfil correcto:
// #define DEVICE_PROFILE 1   // PROFILE_METEO    → LilyGo TTGO
// #define DEVICE_PROFILE 2   // PROFILE_IRRIGATION → 4-Relay Board

// Opcional: proteger OTA con contraseña
// #define OTA_PASSWORD "mi_password_ota"
```

> Para desarrollo local, cambiar `SERVER_IP` a la IP del PC/RPi donde corre Flask y `SERVER_PORT` a `7000`.

---

## Flashear por OTA

El firmware incluye ArduinoOTA. Una vez flasheado por USB la primera vez, los siguientes updates se hacen por WiFi con el script `ota_flash.sh`.

### Requisitos previos (Windows)

Regla de firewall (una sola vez):
```cmd
netsh advfirewall firewall add rule name="ESP32 OTA" dir=in action=allow protocol=TCP localport=10000-60000
```

### Uso del script

```bash
# Flashear solo el LilyGo TTGO (PROFILE_METEO, 192.168.1.9)
./ota_flash.sh --meteo

# Flashear solo la 4-Relay Board (PROFILE_IRRIGATION, 192.168.1.11)
./ota_flash.sh --irrigation

# Compilar y flashear ambos dispositivos seguidos
./ota_flash.sh --all

# Solo upload sin recompilar (usa el .bin ya compilado)
./ota_flash.sh --upload-only --meteo
./ota_flash.sh --upload-only --irrigation

# IP personalizada (sobreescribe la IP por defecto del perfil)
./ota_flash.sh --meteo 192.168.1.X

# Sin argumentos = --meteo (compatibilidad con versiones anteriores)
./ota_flash.sh
```

### Qué hace el script internamente

1. Detecta el SO (Windows/Linux) y localiza `arduino-cli` y `espota.py` automáticamente
2. Compila el sketch pasando `-DDEVICE_PROFILE=N` como flag de compilación — **no hay que editar `secrets.h`**
3. Cada perfil usa su propio directorio de build (`esp_ota_build_meteo` / `esp_ota_build_irrigation`) para no mezclar binarios
4. Verifica conectividad por ping antes de intentar el upload
5. Flashea con `espota.py` al puerto OTA 3232

### FQBNs utilizados

| Perfil | FQBN |
|--------|------|
| METEO | `esp32:esp32:lilygo_t_display` |
| IRRIGATION | `esp32:esp32:esp32` |

---

## Arquitectura del firmware

### ESP32 — FreeRTOS dual-core

```
Core 1 — loop()
 ├─ Cada 100ms  → Leer ADC anemómetro/veleta, acumular vector de viento
 └─ Cada 1s     → Leer I2C (MCP9808, HTU2x, barómetro, luz, YL-69)
                  Actualizar pantalla TFT (PROFILE_METEO)

Core 0 — networkTask()  [prioridad 2]
 ├─ Cada ~10ms  → ArduinoOTA.handle()  ← alta frecuencia, nunca bloqueado
 ├─ Cada 2s     → GET /api/relay/command → actuar relay(s) si cambia el estado
 ├─ Al arrancar → POST /api/device_info (una sola vez)
 └─ Cada 20s    → Snapshot de datos con mutex + POST /send_message
```

La comunicación entre cores usa:
- **`dataMutex`** (FreeRTOS): protege el snapshot de datos al construir el payload HTTP
- **`windMux`** (sección crítica): protege los acumuladores vectoriales del viento durante el reset

### Flag `isUpdatingOTA`

Cuando arranca una actualización OTA, se activa `isUpdatingOTA`:
- La `networkTask` deja de hacer peticiones HTTP y solo llama a `ArduinoOTA.handle()`
- Los relays se ponen en OFF (HIGH) automáticamente por seguridad
- Los sensores y la pantalla siguen funcionando en Core 1 (se puede mostrar progreso)

### ESP8266

Sin FreeRTOS. Toda la lógica permanece en `loop()` con los mismos timers de `millis()`. El OTA se llama al inicio de cada iteración del loop.

### Al arrancar (`setup()`)

1. Reducir CPU a 160 MHz
2. Escanear bus I2C y detectar sensores
3. Inicializar relays en **OFF** (`HIGH` para relays activo-LOW)
4. Si HTU2x detectado: calentamiento 3s con calefactor interno
5. Conectar WiFi
6. Iniciar ArduinoOTA con callbacks detallados
7. Crear `networkTask` en Core 0 (solo ESP32)
8. Activar Modem Sleep

---

## Relay electroválvula(s)

Los relays **JQC-3FF-S-Z** son **activo-LOW**:

| GPIO | Estado | Relay | Válvula |
|------|--------|-------|---------|
| HIGH | arranque / seguro | OFF | Cerrada |
| LOW  | activado | ON | Abierta |

El campo `relay_active` en el CSV es un **bitmask**:
- `0` → todos los relays OFF
- `1` → relay 0 ON (bit 0)
- `3` → relays 0 y 1 ON (bits 0 y 1)
- `15` → los 4 relays ON (PROFILE_IRRIGATION)

El dashboard envía el bitmask deseado vía `POST /api/relay`. El ESP lo consulta cada 2s en `GET /api/relay/command` y aplica el cambio inmediatamente. Confirma el estado real con `POST /api/relay/ack`.

---

## Filtros y estabilización de sensores

### Anemómetro — media móvil (buffer circular 10 muestras)

```cpp
float filteredADC(int newVal) {
    anemometerValues[aneIdx] = newVal;
    aneIdx = (aneIdx + 1) % FILTER_SIZE;
    long s = 0;
    for (int i = 0; i < FILTER_SIZE; i++) s += anemometerValues[i];
    return (float)s / FILTER_SIZE;
}
```

### YL-69 (humedad suelo) — media móvil (buffer circular 10 muestras)

Mismo algoritmo que el anemómetro. El buffer se precalienta con la lectura inicial en `setup()` para evitar transitorios al arrancar. Solo disponible en `PROFILE_METEO` (GPIO33).

### Dirección del viento — promedio vectorial

Media aritmética de ángulos es incorrecta cerca del norte (350° y 10° → 180° en vez de 0°). El firmware acumula vectores unitarios y calcula `atan2`:

```cpp
// Cada 100ms (Core 1, protegido con sección crítica)
windSumX += cos(deg * PI / 180);
windSumY += sin(deg * PI / 180);

// Cada 20s (Core 0, bajo sección crítica)
float dir = atan2(windSumY, windSumX) * 180 / PI;
if (dir < 0) dir += 360;
```

---

## Ahorro energético

| Medida | Configuración | Ahorro estimado |
|--------|---------------|-----------------|
| CPU a 160 MHz | `setCpuFrequencyMhz(160)` en `setup()` | ~20% vs 240 MHz |
| Modem Sleep | `WiFi.setSleep(true)` tras conectar | ~15–20 mA entre transmisiones |
| Pantalla con timeout | Se apaga tras 60s sin actividad | Variable según brillo |

Los **botones** (GPIO0/BOOT y GPIO35) reactivan la pantalla y reinician el timer de 60s.

---

## Simulación de sensores

Si un sensor no responde al arrancar o falla durante el funcionamiento, el firmware continúa en **modo simulación** para ese sensor con valores que derivan suavemente:

| Sensor | Rango simulado | Drift/lectura |
|--------|---------------|---------------|
| MCP9808 (T.Ext) | −10 a 45 °C | ±0.05 °C |
| HTU2x (T.Int)   | −10 a 45 °C | ±0.05 °C |
| HTU2x (Humedad) | 20 a 95 %   | ±0.20 % |
| Barómetro       | 95 a 110 kPa | ±0.02 kPa |
| Luz             | 0 a 2000 lux | ±5 lux |

El badge en pantalla indica `[OK]` (dato real) o `[SIM]` (simulado).

---

## Algoritmos

### Sensor de luz — Autodetección TSL2584 / APDS-9930

El chip en 0x39 se identifica al arrancar:
- `ID@0x12 & 0xF0 == 0x30` → APDS-9930 (registros 0x14–0x17)
- `ID@0x0A & 0xF0 == 0xA0` → TSL2584 (registros 0x0C–0x0F)
- Desconocido → fallback a APDS-9930

### Pipeline — Simulador de caudalímetro y sensor de presión

El firmware genera valores realistas de presión (bar) y caudal (L/min) según el escenario configurado desde el dashboard (`normal` / `leak` / `burst`). El ruido es determinista (ondas sinusoidales del timestamp) para poder comparar con el simulador Python del backend.

---

## Formato de datos enviados al servidor

### CSV periódico — `POST /send_message` (cada 20s)

16 valores separados por coma:

| Pos | Campo | Sensor | Unidad |
|-----|-------|--------|--------|
| 0 | `temperature` | MCP9808 | °C |
| 1 | `pressure` | MicroPressure | kPa |
| 2 | `temperature_bar` | HTU2x | °C |
| 3 | `humidity` | HTU2x | % |
| 4 | `windSpeed` | Anemómetro | m/s |
| 5 | `windDirection` | Veleta | ° |
| 6 | `windSpeedFiltered` | Anemómetro (filtrado) | m/s |
| 7 | `windDirectionFiltered` | Veleta (promedio vectorial) | ° |
| 8 | `light` | APDS-9930 / TSL2584 | lux |
| 9 | `dht_temperature` | DHT11 | °C |
| 10 | `dht_humidity` | DHT11 | % |
| 11 | `rssi` | WiFi | dBm |
| 12 | `free_heap` | ESP32 | bytes |
| 13 | `uptime_s` | ESP32 | s |
| 14 | `relay_active` | Relay(s) | bitmask |
| 15 | `soil_moisture` | YL-69 (filtrado) | % |

> `relay_active` es un bitmask: bit 0 = relay 0, bit 1 = relay 1, etc.
> `soil_moisture` solo tiene valor real en PROFILE_METEO; en PROFILE_IRRIGATION es 0.

### Info estática del dispositivo — `POST /api/device_info` (al arrancar)

JSON generado con ArduinoJson:

```json
{
  "chip_model": "ESP32-D0WD-V3",
  "chip_revision": 3,
  "cpu_freq_mhz": 160,
  "flash_size_mb": 16,
  "sdk_version": "v5.5.2-729-g87912cd291",
  "mac_address": "88:13:BF:FD:A2:38",
  "ip_address": "192.168.1.9",
  "relay_count": 1
}
```

El campo `relay_count` (1 o 4 según el perfil) determina cuántos botones de válvula muestra el dashboard automáticamente.

---

## Pantalla TFT

Solo disponible en `PROFILE_METEO`. Resolución **240×135 px** (ST7789). Doble buffer con `TFT_eSprite` → sin parpadeo.

```
┌────────────────────────────────────────┐
│ METEOSTATION   ☀0lx  WiFi  ●          │  ← cabecera (● verde=OK / rojo=error)
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

---

## Problemas conocidos

| Problema | Estado | Solución |
|----------|--------|----------|
| Presión en kPa en vez de hPa | Pendiente | Cambiar `readPressure(KPA)` → `readPressure(PA) / 100.0` |
| DHT11 lecturas inestables | Conocido | Valorar reemplazar por DHT22 o SHT31 |
| ArduinoJson requiere instalación manual | Pendiente | Instalar "ArduinoJson" de Benoit Blanchon desde el Library Manager |
