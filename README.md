# MeteoStation — Firmware ESP32

Firmware para la estación meteorológica doméstica Aquantia. Compatible con **dos perfiles de ESP32** y un nodo ESP8266 independiente.

El firmware soporta **dos modos de comunicación** seleccionables en tiempo de compilación con `#define USE_MQTT` en `secrets.h`:

| Modo | Transporte | Relay control | Activación |
|------|-----------|--------------|------------|
| **HTTP legacy** | HTTPS/CSV → `POST /send_message` | Polling `GET /api/relay/command` cada 2s | Por defecto (sin `USE_MQTT`) |
| **MQTT** | MQTT/TLS/JSON → broker Mosquitto | Push inmediato vía callback | `#define USE_MQTT` en `secrets.h` |

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Dispositivos y perfiles](#dispositivos-y-perfiles)
- [Hardware](#hardware)
- [Librerías necesarias](#librerías-necesarias)
- [Configuración antes de compilar](#configuración-antes-de-compilar)
- [Flashear por OTA](#flashear-por-ota)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Modo HTTP legacy](#modo-http-legacy)
- [Modo MQTT](#modo-mqtt)
- [Relay electroválvula(s)](#relay-electroválvulas)
- [Filtros y estabilización de sensores](#filtros-y-estabilización-de-sensores)
- [Simulación de sensores](#simulación-de-sensores)
- [Pantalla TFT](#pantalla-tft)
- [Ahorro energético](#ahorro-energético)
- [Algoritmos](#algoritmos)
- [Problemas conocidos](#problemas-conocidos)

---

## Dispositivos y perfiles

El firmware compila un binario distinto para cada dispositivo usando `DEVICE_PROFILE`:

| Perfil | Dispositivo | IP | Relays | Pantalla | Suelo |
|--------|------------|-----|--------|----------|-------|
| `PROFILE_METEO` (1) | LilyGo TTGO T-Display | 192.168.1.9 | 1 × GPIO26 | ST7789 240×135 | YL-69 GPIO33 |
| `PROFILE_IRRIGATION` (2) | ESP32 4-Relay Board | 192.168.1.11 | 4 × GPIO32/33/25/26 | No | No |

El `DEVICE_PROFILE` se pasa en tiempo de compilación. El script `ota_flash.sh` lo gestiona automáticamente; no hay que editar el código fuente entre compilaciones.

---

## Hardware

### PROFILE_METEO — LilyGo TTGO T-Display

| Componente | Modelo | Conexión |
|------------|--------|----------|
| Microcontrolador | LilyGo TTGO T-Display (ESP32-D0WD-V3) | — |
| Temperatura exterior | Adafruit MCP9808 | I2C 0x19 |
| Temperatura + humedad | HTU2x (HTU21D / HTU20D) | I2C 0x40 |
| Temperatura + humedad (2) | DHT11 | GPIO15 |
| Barómetro | SparkFun MicroPressure | I2C 0x18 |
| Luz ambiente | APDS-9930 / TSL2584 (clon, autodetección) | I2C 0x39 |
| Anemómetro | Analógico 0–3.3 V (0–30 m/s) | GPIO37 |
| Veleta | Analógica 0–3.3 V (0–360°) | GPIO36 |
| Humedad suelo | YL-69 + divisor de tensión | ADC GPIO33 |
| Pantalla | ST7789 240×135 integrada | SPI (TFT_eSPI) |
| LED indicador | Onboard | GPIO2 |
| Relay | JQC-3FF-S-Z (activo-LOW) | GPIO26 |
| Botones | BOOT / GPIO35 | GPIO0 / GPIO35 |

### PROFILE_IRRIGATION — ESP32 4-Relay Board

| Componente | Modelo | Conexión |
|------------|--------|----------|
| Microcontrolador | ESP32-WROOM-32E (ESP32-D0WD-V3) | — |
| Relay 1–4 | JQC-3FF-S-Z (activo-LOW) | GPIO32/33/25/26 |
| LED estado | Onboard | GPIO23 |

> Referencia de placa: [ESP32 Relay x4 — ESPHome devices](https://devices.esphome.io/devices/ESP32-Relay-x4/)

### Sensores I2C detectados en hardware real (PROFILE_METEO)

```
0x18 → SparkFun MicroPressure
0x19 → Adafruit MCP9808
0x39 → APDS-9930 clon (ID@0x12=0x98)
0x40 → HTU2x (HTU21D)
```

---

## Librerías necesarias

Instalar desde el gestor de librerías de Arduino IDE:

| Librería | Autor | Modo | Uso |
|----------|-------|------|-----|
| `TFT_eSPI` | Bodmer | Ambos | Pantalla ST7789 (solo PROFILE_METEO) |
| `Adafruit MCP9808 Library` | Adafruit | Ambos | Sensor de temperatura exterior |
| `SparkFun MicroPressure Library` | SparkFun | Ambos | Barómetro |
| `DHTesp` | beegee-tokyo | Ambos | DHT11 |
| `ArduinoJson` | Benoit Blanchon | Ambos | Payloads JSON (device_info, MQTT) |
| `PubSubClient` | knolleary | Solo MQTT | Cliente MQTT para ESP32 |

> HTU2x y el sensor de luz se implementan directamente sobre I2C sin librería externa.

### Configuración TFT_eSPI (PROFILE_METEO)

Editar `User_Setup_Select.h` de la librería para activar la configuración del TTGO T-Display:

```cpp
//#include <User_Setup.h>                         // comentar línea por defecto
#include <User_Setups/Setup25_TTGO_T_Display.h>  // activar T-Display
```

---

## Configuración antes de compilar

Crear `ESP_monitor_server/secrets.h` (excluido del repo por `.gitignore`):

```cpp
// secrets.h — NO subir al repositorio

// ── WiFi y servidor (siempre necesarios) ─────────────────────────────
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"
#define SERVER_IP     "meteo.aquantialab.com"   // solo en modo HTTP
#define SERVER_PORT   443                        // solo en modo HTTP

// ── Perfil de dispositivo ─────────────────────────────────────────────
// El script ota_flash.sh lo inyecta automáticamente.
// Si compilas desde Arduino IDE, descomenta el perfil correcto:
// #define DEVICE_PROFILE 1   // PROFILE_METEO    → LilyGo TTGO
// #define DEVICE_PROFILE 2   // PROFILE_IRRIGATION → 4-Relay Board

// ── OTA (opcional) ───────────────────────────────────────────────────
// #define OTA_PASSWORD "mi_password_ota"

// ── Modo MQTT ────────────────────────────────────────────────────────
// Descomentar USE_MQTT para activar comunicación MQTT en lugar de HTTP.
// Requiere la librería PubSubClient instalada en Arduino IDE.
//
// #define USE_MQTT
// #define MQTT_SERVER  "meteo.aquantialab.com"
// #define MQTT_PORT    8883
// #define FINCA_ID     "aquantia_prototype_1"    // namespace del dispositivo
// #define MQTT_USER    "aquantia_prototype_1"    // username = finca_id (ACL)
// #define MQTT_PASS    "contraseña_del_dispositivo"
```

---

## Flashear por OTA

El firmware incluye ArduinoOTA. La primera vez se flashea por USB; los siguientes updates se hacen por WiFi con `ota_flash.sh`.

### Requisitos previos (Windows, una sola vez)

```cmd
netsh advfirewall firewall add rule name="ESP32 OTA" dir=in action=allow protocol=TCP localport=10000-60000
```

### Uso del script

```bash
# Flashear LilyGo TTGO (PROFILE_METEO → 192.168.1.9)
./ota_flash.sh --meteo

# Flashear ESP32 4-Relay Board (PROFILE_IRRIGATION → 192.168.1.11)
./ota_flash.sh --irrigation

# Compilar y flashear ambos
./ota_flash.sh --all

# Solo upload sin recompilar (usa el .bin ya compilado)
./ota_flash.sh --upload-only --meteo
./ota_flash.sh --upload-only --irrigation

# IP personalizada
./ota_flash.sh --meteo 192.168.1.X
```

### Qué hace el script internamente

1. Detecta el SO (Windows/Linux) y localiza `arduino-cli` y `espota.py` automáticamente
2. Compila el sketch pasando `-DDEVICE_PROFILE=N` como flag de compilación — sin tocar el código fuente
3. Cada perfil usa su propio directorio de build para no mezclar binarios
4. Verifica conectividad por ping antes de intentar el upload
5. Flashea con `espota.py` al puerto OTA 3232

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
 ├─ Cada 1s     → Leer I2C (MCP9808, HTU2x, barómetro, luz)
 │               Leer ADC YL-69 (PROFILE_METEO)
 │               Actualizar pantalla TFT (PROFILE_METEO)
 └─ Siempre     → Gestionar botones y timeout de pantalla

Core 0 — networkTask()  [prioridad 2]
 ├─ Cada ~10ms  → ArduinoOTA.handle()  ← nunca bloqueado
 ├─ Al arrancar → [HTTP] POST /api/device_info
 │                [MQTT] mqttConnect() + mqttPublishRegister()
 ├─ Continuo    → [MQTT] mqttClient.loop()  ← recibe cmd commands
 ├─ Cada 2s     → [HTTP] GET /api/relay/command → actuar relay(s) + ack
 └─ Cada 20s    → Snapshot datos con mutex + envío al servidor
                  [HTTP] CSV → POST /send_message
                  [MQTT] JSON → publish aquantia/<finca_id>/telemetry
```

La comunicación entre cores usa:
- **`dataMutex`** (FreeRTOS Mutex): protege el snapshot de sensores al construir el payload
- **`windMux`** (sección crítica): protege los acumuladores vectoriales del viento durante el reset

### Stack de networkTask (FreeRTOS)

| Modo | Stack asignado | Motivo |
|------|----------------|--------|
| HTTP legacy | 8192 bytes | Suficiente para HTTPClient + TLS |
| MQTT (`USE_MQTT`) | 12288 bytes | Overhead adicional de WiFiClientSecure + PubSubClient |

### Flag `isUpdatingOTA`

Cuando arranca una actualización OTA:
- `networkTask` deja de enviar datos y solo llama a `ArduinoOTA.handle()`
- Todos los relays se ponen en OFF (HIGH) automáticamente por seguridad
- Core 1 (sensores/pantalla) sigue funcionando sin interrupciones

---

## Modo HTTP legacy

**Activado cuando `USE_MQTT` no está definido.**

### Ciclo de red

```
setup():
  └─ postDeviceInfo() → POST /api/device_info (JSON con chip info)

networkTask() loop:
  ├─ Cada 2s:
  │   └─ checkRelayCommand() → GET /api/relay/command
  │      Si bitmask cambió → actuar relays → POST /api/relay/ack
  └─ Cada 20s:
      └─ snapshot sensores (bajo dataMutex)
         → CSV de 16 campos
         → httpPost("https://SERVER_IP/send_message", csv)
```

### Formato CSV enviado (16 campos)

```
temperature,pressure,temperature_barometer,humidity,
windSpeed,windDirection,windSpeedFiltered,windDirectionFiltered,
light,dht_temperature,dht_humidity,
rssi,free_heap,uptime_s,relay_active,soil_moisture
```

Ejemplo:
```
25.31,101.14,23.01,69.33,3.50,180.00,3.32,178.50,350.40,25.00,63.00,-62,245000,12345,0,50.20
```

---

## Modo MQTT

**Activado definiendo `USE_MQTT` en `secrets.h`.**

### Ciclo de red

```
setup():
  (nada extra — todo en networkTask)

networkTask():
  ├─ Una vez (antes del for(;;)):
  │   mqttTLSClient.setCACert(MQTT_CA_CERT_PEM)   ← ISRG Root X1
  │   mqttClient.setServer(mqtt_server, mqtt_port)
  │   mqttClient.setCallback(mqttCallback)
  │   mqttClient.setBufferSize(512)
  │
  └─ loop for(;;):
      ├─ Si no conectado:
      │   └─ mqttConnect():
      │       TLS handshake → autenticar → subscribe aquantia/<finca_id>/cmd
      │       Si primera conexión → mqttPublishRegister()
      ├─ mqttClient.loop()   ← procesa mensajes entrantes (relay cmd)
      └─ Cada 20s:
          snapshot sensores → StaticJsonDocument<384> → publish telemetry
```

### TLS y certificado

La verificación TLS usa el certificado raíz **ISRG Root X1** de Let's Encrypt, almacenado en `mqtt_cert.h` como `const char MQTT_CA_CERT_PEM[] PROGMEM`. Al estar en PROGMEM, no consume RAM dinámica.

`WiFiClientSecure::setCACert()` carga el certificado y verifica que el CN del broker (`meteo.aquantialab.com`) coincida con el certificado presentado en el handshake TLS. No es necesario `setInsecure()`.

### Topics MQTT

| Topic | Dirección | Cuándo | Contenido |
|-------|-----------|--------|-----------|
| `aquantia/<finca_id>/register` | ESP → broker | Al arrancar (una vez) | JSON con MAC, IP, chip info, relay_count |
| `aquantia/<finca_id>/telemetry` | ESP → broker | Cada 20s | JSON con 17 campos de sensores |
| `aquantia/<finca_id>/cmd` | broker → ESP | Cuando el dashboard activa un relay | `{"relay": 0, "state": true}` |

### Payload de telemetría (JSON, 17 campos)

```json
{
  "temperature":           22.5,
  "pressure":              101.3,
  "temperature_barometer": 21.8,
  "humidity":              65.2,
  "windSpeed":             3.5,
  "windDirection":         180.0,
  "windSpeedFiltered":     3.3,
  "windDirectionFiltered": 178.0,
  "light":                 350.0,
  "dht_temperature":       21.6,
  "dht_humidity":          63.0,
  "rssi":                  -65,
  "free_heap":             245000,
  "uptime_s":              12345,
  "relay_active":          0,
  "soil_moisture":         50.0,
  "mac_address":           "88:13:BF:FD:A2:38"
}
```

### Callback de comandos (`mqttCallback`)

Cuando llega un mensaje en `aquantia/<finca_id>/cmd`:

```cpp
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // parsea {"relay": N, "state": true/false}
    // actúa el relay: digitalWrite(RELAY_PINS[N], state ? LOW : HIGH)
    // relay activo-LOW: LOW = ON (válvula abierta), HIGH = OFF
}
```

La latencia es la del round-trip WiFi → broker → ESP, típicamente <100ms en red local.

### Reconexión automática

Si el broker se desconecta, `networkTask` detecta `!mqttClient.connected()` en el siguiente ciclo, reintenta `mqttConnect()` y, si falla, espera 5s antes de reintentar. No bloquea el loop de sensores (Core 1).

---

## Relay electroválvula(s)

Los relays **JQC-3FF-S-Z** son **activo-LOW**:

| GPIO | Estado | Relay | Válvula |
|------|--------|-------|---------|
| HIGH | arranque / seguro | OFF | Cerrada |
| LOW  | activado | ON | Abierta |

El campo `relay_active` en el CSV y en el JSON MQTT es un **bitmask**:

| Valor | Estado |
|-------|--------|
| `0` | Todos OFF |
| `1` | Relay 0 ON (bit 0) |
| `3` | Relays 0 y 1 ON (bits 0 y 1) |
| `15` | Los 4 relays ON (PROFILE_IRRIGATION) |

### Control por modo

**HTTP legacy:**
- El ESP consulta `GET /api/relay/command` cada 2s → recibe el bitmask deseado → aplica cambios → confirma con `POST /api/relay/ack`

**MQTT:**
- El dashboard envía `POST /api/relay` → Flask publica `{"relay": N, "state": bool}` en `aquantia/<finca_id>/cmd` → ESP recibe en `mqttCallback()` → actúa inmediatamente

---

## Filtros y estabilización de sensores

### Anemómetro — media móvil (buffer circular 10 muestras)

Suaviza los picos de lectura ADC a 0–30 m/s. La velocidad filtrada se envía en `windSpeedFiltered`.

### YL-69 (humedad suelo) — media móvil (buffer circular 10 muestras)

Solo en `PROFILE_METEO` (GPIO33). El buffer se precalienta con la lectura inicial en `setup()` para evitar transitorios al arrancar.

Calibración:
```cpp
#define SOIL_RAW_DRY   3300   // ADC en tierra seca (~0%)
#define SOIL_RAW_WET   1000   // ADC en tierra saturada (~100%)
```

### Dirección del viento — promedio vectorial

La media aritmética de ángulos es incorrecta cerca del norte (350° y 10° → 180° en lugar de 0°). El firmware acumula vectores unitarios cada 100ms y calcula `atan2` cada 20s:

```cpp
// Cada 100ms (Core 1, sección crítica)
windSumX += cos(deg * PI / 180);
windSumY += sin(deg * PI / 180);

// Cada 20s (Core 0, bajo sección crítica)
float dir = atan2(windSumY, windSumX) * 180 / PI;
if (dir < 0) dir += 360;
```

---

## Simulación de sensores

Si un sensor no responde al arrancar, el firmware continúa en **modo simulación** para ese sensor con valores que derivan suavemente:

| Sensor | Rango simulado | Drift/lectura |
|--------|---------------|---------------|
| MCP9808 | −10 a 45 °C | ±0.05 °C |
| HTU2x temperatura | −10 a 45 °C | ±0.05 °C |
| HTU2x humedad | 20 a 95 % | ±0.20 % |
| DHT11 temperatura | −10 a 45 °C | ±0.05 °C |
| DHT11 humedad | 20 a 95 % | ±0.20 % |
| Barómetro | 95 a 110 kPa | ±0.02 kPa |
| Luz | 0 a 2000 lux | ±5 lux |
| Viento | 0 a 15 m/s | ±0.30 m/s |
| Dirección | 0–360° | ±2.5° |
| Humedad suelo | 0–100% | ±0.5% |

La pantalla muestra `[OK]` o `[SIM]` para cada sensor.

---

## Pantalla TFT

Solo `PROFILE_METEO`. Resolución **240×135 px** (ST7789). Doble buffer con `TFT_eSprite` → sin parpadeo.

```
┌────────────────────────────────────────┐
│ METEOSTATION   ☀350lx  WiFi  ●         │  ← ● verde=OK / rojo=error
├──────────┬──────────┬──────────────────┤
│ T.EXT    │ T.INT    │ HUMEDAD          │
│ 22.5 C   │ 21.8 C   │ 65.2 %          │
│ [OK]     │ [OK]     │ [OK]            │
├──────────┼──────────┼──────────────────┤
│ PRESION  │ VIENTO   │ DIRECC.          │
│ 101.3KPa │ 3.5 m/s  │ 180 deg S       │
│ [OK]     │          │                  │
└────────────────────────────────────────┘
```

Los **botones** (GPIO0/BOOT y GPIO35) reactivan la pantalla y reinician el timer de apagado (60s).

---

## Ahorro energético

| Medida | Configuración | Ahorro estimado |
|--------|---------------|-----------------|
| CPU a 160 MHz | `setCpuFrequencyMhz(160)` en setup | ~20% vs 240 MHz |
| Modem Sleep | `WiFi.setSleep(true)` tras conectar | ~15–20 mA entre transmisiones |
| Pantalla con timeout | Apagado tras 60s sin actividad | Variable según brillo |

---

## Algoritmos

### Sensor de luz — Autodetección TSL2584 / APDS-9930

El chip en 0x39 se identifica al arrancar:
- `ID@0x12 & 0xF0 == 0x30` → APDS-9930 (registros 0x14–0x17)
- `ID@0x0A & 0xF0 == 0xA0` → TSL2584 (registros 0x0C–0x0F)
- Desconocido → fallback APDS-9930

### Pipeline — Simulador de presión y caudal

El firmware genera valores realistas de presión (bar) y caudal (L/min) según el escenario activo (`normal` / `leak` / `burst`). El ruido es **determinista** (tres ondas sinusoidales del timestamp) para poder comparar resultados con el simulador Python del backend sin varianza aleatoria.

---

## Problemas conocidos

| Problema | Estado |
|----------|--------|
| Presión en kPa en vez de hPa | Pendiente — cambiar `readPressure(KPA)` → `readPressure(PA) / 100.0` |
| DHT11 lecturas inestables a veces | Conocido — valorar reemplazar por DHT22 o SHT31 |
