# Aquantia — Firmware ESP32

Firmware para la estación meteorológica y sistema de riego doméstico Aquantia. Compatible con **dos perfiles de hardware** seleccionables en tiempo de compilación.

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Perfiles de dispositivo](#perfiles-de-dispositivo)
- [Herramienta de flash](#herramienta-de-flash)
- [Provisioning — primer arranque](#provisioning--primer-arranque)
- [Librerías necesarias](#librerías-necesarias)
- [secrets.h — modo DEV](#secretsh--modo-dev)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Modo MQTT](#modo-mqtt)
- [Modo HTTP legacy](#modo-http-legacy)
- [Relay y electroválvulas](#relay-y-electroválvulas)
- [Pantalla TFT](#pantalla-tft)
- [Filtros y estabilización](#filtros-y-estabilización)
- [Simulación de sensores](#simulación-de-sensores)
- [Modo debug (rama test)](#modo-debug-rama-test)
- [Ahorro energético](#ahorro-energético)
- [Problemas conocidos](#problemas-conocidos)

---

## Perfiles de dispositivo

El firmware compila un binario distinto para cada dispositivo. El perfil se pasa con `-DDEVICE_PROFILE=N` en tiempo de compilación; no hay que editar el código fuente.

| Perfil | `DEVICE_PROFILE` | Hardware | Relays | Pantalla | Sensores meteo |
|--------|:----------------:|----------|:------:|:--------:|:--------------:|
| **METEO** | 1 | LilyGo TTGO T-Display | 1 × GPIO26 | ST7789 240×135 | Sí |
| **IRRIGATION** | 2 | ESP32 4-Relay Board | 4 × GPIO32/33/25/26 | No | No |

Pinout completo de cada perfil: [PINOUT.md](PINOUT.md)

---

## Herramienta de flash

La herramienta principal es **`tools/flasher_gui.py`** — una aplicación de escritorio Python/Tkinter que gestiona todo el ciclo de desarrollo:

```
tools/
├── flasher_gui.py          ← herramienta principal
├── Aquantia Flash Tool.bat ← lanzador Windows (doble clic)
└── Aquantia Flash Tool.vbs ← lanzador silencioso (sin ventana CMD)
```

### Funcionalidades

| Función | Descripción |
|---------|-------------|
| Selector de perfil | METEO / IRRIGATION |
| Selector de versión | Working copy, tags, commits históricos |
| Info de commit | Muestra hash, fecha, autor y mensaje antes de flashear |
| Estado del binario | Indica si el binario en caché es válido o hay que recompilar |
| Modo DEV / PROD | Activa/desactiva `DEV_MODE` en `secrets.h` sin editar el archivo |
| Compilar | `arduino-cli compile` con barra de progreso por hitos |
| Flash USB | `arduino-cli upload` con barra de progreso real (formato `esptool`) |
| Flash OTA | `espota.py` con barra de progreso real (formato `espota`) |
| Buscar OTA | Descubre dispositivos ArduinoOTA via mDNS (`_arduino._tcp`) |
| Factory Provision | Lee MAC → genera token → registra en backend → escribe NVS |
| **Borrar NVS** | Lee tabla de particiones real del chip y borra solo la partición NVS |

### Requisitos

```bash
pip install pyserial          # puertos serie
pip install zeroconf          # opcional — descubrimiento OTA por mDNS
pip install qrcode[pil]       # opcional — QR en factory provision
pip install bcrypt            # factory provision (hash de token)
```

Arduino IDE 2.x instalado (proporciona `arduino-cli`, `esptool` y `espota.py`).

### Borrar NVS

El botón **"🗑 Borrar NVS"** en la sección Factory Provision:

1. Lee la tabla de particiones real del chip desde `0x8000`
2. Localiza la partición NVS (type=data, subtype=nvs)
3. Muestra en el log el nombre, offset y tamaño exactos detectados
4. Pide confirmación antes de borrar
5. Ejecuta `esptool erase_region <offset> <size>`

Funciona con todos los ESP32 (S2, S3, C3, C6) y tablas de particiones personalizadas.

Tras el borrado el dispositivo arranca en **modo SoftAP** y espera que el usuario configure el WiFi.

---

## Provisioning — primer arranque

El firmware incluye un sistema de provisioning **SoftAP + NVS** que elimina la necesidad de hardcodear credenciales WiFi.

### Flujo en modo PROD

```
1. Arranque sin credenciales en NVS
   └─► TFT muestra pantalla "AQUANTIA SETUP" con el SSID y la contraseña
   └─► Serial imprime: [PROV] Conecta tu movil a: Aquantia-XXXXXX

2. Usuario conecta su móvil a la red WiFi "Aquantia-XXXXXX" (pass: aquantia1)
   └─► Abre http://192.168.4.1 en el navegador

3. Portal SoftAP muestra redes disponibles
   └─► Usuario elige su red y escribe la contraseña

4. ESP32 guarda las credenciales en NVS (partición 0x9000)
   └─► Reinicia y conecta a la red configurada

5. El dispositivo funciona con normalidad — ya no vuelve al portal
```

### SSID del punto de acceso

El SSID se forma con los últimos 6 caracteres hexadecimales de la MAC del chip, leídos de eFuse (sin necesidad de WiFi inicializado):

```
Serial = AQ-FCB467F37748
SSID   = Aquantia-F37748
```

### Modo DEV (desarrollo)

Cuando `DEV_MODE` está activo en `secrets.h`, el dispositivo **ignora el portal SoftAP** y usa directamente las credenciales de `secrets.h`. Útil para desarrollo en banco sin tener que configurar el portal cada vez.

El botón **DEV / PROD** de la Flash Tool gestiona este flag sin editar el archivo manualmente.

---

## Librerías necesarias

Instalar desde el gestor de librerías de Arduino IDE 2.x:

| Librería | Autor | Perfil | Uso |
|----------|-------|:------:|-----|
| `TFT_eSPI` | Bodmer | METEO | Pantalla ST7789 |
| `Adafruit MCP9808 Library` | Adafruit | METEO | Temperatura exterior |
| `SparkFun MicroPressure Library` | SparkFun | METEO | Barómetro |
| `DHTesp` | beegee-tokyo | METEO | DHT11 |
| `ArduinoJson` | Benoit Blanchon | Ambos | Payloads JSON |
| `PubSubClient` | knolleary | Ambos (MQTT) | Cliente MQTT |

HTU2x y el sensor de luz se implementan directamente sobre I2C sin librería externa.

### Configuración TFT_eSPI (PROFILE_METEO)

Editar `User_Setup_Select.h` de la librería:

```cpp
//#include <User_Setup.h>                         // comentar
#include <User_Setups/Setup25_TTGO_T_Display.h>  // activar T-Display
```

---

## secrets.h — modo DEV

Crear `ESP_monitor_server/secrets.h` (excluido del repo vía `.gitignore`):

```cpp
// secrets.h — NO subir al repositorio

// ── Modo DEV — activo: usa estas credenciales directamente (sin portal SoftAP)
//              inactivo: el dispositivo usa NVS / portal SoftAP en primer arranque
#define DEV_MODE

// ── Perfil de hardware (solo si no es METEO, que es el valor por defecto) ─────
// #define DEVICE_PROFILE PROFILE_IRRIGATION   // ← descomentar para placa riego

// ── WiFi (solo DEV_MODE) ──────────────────────────────────────────────────────
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"

// ── Servidor HTTP legacy (solo modo HTTP, sin USE_MQTT) ───────────────────────
#define SERVER_IP     "meteo.aquantialab.com"
#define SERVER_PORT   443

// ── MQTT ─────────────────────────────────────────────────────────────────────
#define USE_MQTT
#define MQTT_SERVER  "meteo.aquantialab.com"
#define MQTT_PORT    8883
#define FINCA_ID     "finca-dev"

// En DEV_MODE el broker acepta el usuario interno "backend":
#define MQTT_USER    "backend"
#define MQTT_PASS    "aquantia_159"   // valor de MQTT_PASSWORD en el .env del servidor
```

En modo **PROD** (sin `DEV_MODE`), las credenciales WiFi y el token MQTT vienen de la NVS escrita por la Flash Tool durante el factory provision. El `secrets.h` solo necesita las constantes de compilación (`USE_MQTT`, `MQTT_SERVER`, `MQTT_PORT`).

### Diferencia DEV / PROD en la autenticación MQTT

Este es el punto más importante. En función del modo, el firmware usa **credenciales distintas** para conectar al broker:

| | `DEV_MODE` activo | `DEV_MODE` inactivo (PROD) |
|--|--|--|
| **Usuario MQTT** | `MQTT_USER` de `secrets.h` → `"backend"` | MAC del dispositivo → `"FC:B4:67:F3:77:48"` |
| **Contraseña MQTT** | `MQTT_PASS` de `secrets.h` | Token generado en fábrica, leído de NVS |
| **WiFi** | `WIFI_SSID` / `WIFI_PASSWORD` de `secrets.h` | SSID + contraseña guardados en NVS |
| **Provisioning** | Saltado | Portal SoftAP en primer arranque |

El motivo es que en PROD el broker identifica cada dispositivo **por su MAC** y verifica el token con un hash bcrypt almacenado en la base de datos. En DEV, en cambio, no existe ese token todavía (el dispositivo no ha pasado por fábrica), así que se usa el usuario interno `backend` que el broker acepta directamente comparando la contraseña con la variable de entorno `MQTT_PASSWORD`.

> **Nota de implementación:** en el firmware, la sobreescritura `mqtt_user = MAC` está dentro de un bloque `#ifndef DEV_MODE`. Sin ese guard, el usuario de `secrets.h` quedaría siempre machacado por la MAC tras la conexión WiFi, haciendo inefectivo el `MQTT_USER` en DEV.

---

## Arquitectura del firmware

### FreeRTOS dual-core

```
Core 1 — loop()
 ├─ Cada 100ms  → Leer ADC anemómetro/veleta (METEO)
 │               Acumular vector de viento para promedio vectorial
 ├─ Cada 1s     → Leer I2C: MCP9808, HTU2x, barómetro, luz (METEO)
 │               Leer ADC YL-69 humedad suelo (METEO)
 │               Actualizar pantalla TFT (METEO)
 └─ Siempre     → Gestionar botones y timeout de pantalla (METEO)

Core 0 — networkTask()  [prioridad 2]
 ├─ Cada ~10ms  → ArduinoOTA.handle()  ← nunca bloqueado
 ├─ Al arrancar → [HTTP] POST /api/device_info
 │                [MQTT] mqttConnect() + publish register
 ├─ Continuo    → [MQTT] mqttClient.loop()  ← recibe comandos relay
 ├─ Cada 2s     → [HTTP] GET /api/relay/command → actuar relays + ack
 └─ Cada 20s    → Snapshot bajo dataMutex → envío telemetría
                  [HTTP] CSV → POST /send_message
                  [MQTT] JSON → publish aquantia/<finca_id>/telemetry
```

Sincronización entre cores:
- **`dataMutex`** (FreeRTOS Mutex): protege el snapshot de sensores al construir el payload
- **`windMux`** (sección crítica): protege los acumuladores vectoriales del viento

### Seguridad OTA

Al arrancar una actualización OTA:
- `networkTask` deja de enviar datos
- Todos los relays pasan a OFF (HIGH)
- Core 1 (sensores/pantalla) sigue funcionando

---

## Modo MQTT

Modo de comunicación principal. Activar definiendo `USE_MQTT` en `secrets.h`.

### Topics

| Topic | Dirección | Cuándo | Contenido |
|-------|-----------|--------|-----------|
| `aquantia/<finca_id>/register` | ESP → broker | Al arrancar (1 vez) | JSON con MAC, IP, chip info, relay_count |
| `aquantia/<finca_id>/telemetry` | ESP → broker | Cada 20s | JSON con 17 campos de sensores |
| `aquantia/<finca_id>/cmd` | broker → ESP | Comando relay | `{"relay": 0, "state": true}` |

### Payload telemetría (JSON)

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
  "mac_address":           "FC:B4:67:F3:77:48"
}
```

### Autenticación MQTT — flujo completo

El broker usa el plugin **mosquitto-go-auth**, que delega la validación a un webhook HTTP al backend Flask en cada intento de conexión:

```
ESP32                   Mosquitto               Backend Flask
  │                         │                        │
  │── CONNECT (user/pass) ──►│                        │
  │                         │── POST /api/mqtt/auth ─►│
  │                         │   {"username":…,        │
  │                         │    "password":…}        │
  │                         │                        │ (valida)
  │                         │◄── 200 OK ─────────────│
  │◄── CONNACK (rc=0) ──────│                        │
```

El backend acepta **dos tipos de credenciales**:

| Tipo | `username` | `password` | Cuándo se usa |
|------|-----------|-----------|---------------|
| Usuario interno | `"backend"` | Valor de `MQTT_PASSWORD` en `.env` | DEV_MODE, backend Flask propio |
| Dispositivo | MAC del chip (`"FC:B4:67:F3:77:48"`) | Token generado en fábrica (bcrypt en DB) | PROD, dispositivos reales |

Si el broker no puede alcanzar el backend, deniega todas las conexiones.

### TLS — certificado ISRG Root X1

La verificación TLS usa el certificado raíz de Let's Encrypt almacenado en `mqtt_cert.h` como `PROGMEM`. No consume RAM dinámica. El CN del broker (`meteo.aquantialab.com`) se verifica automáticamente — no se usa `setInsecure()`.

---

## Modo HTTP legacy

Modo de compatibilidad. Activo cuando `USE_MQTT` **no** está definido.

El ESP envía un CSV de 16 campos cada 20s y consulta el estado del relay cada 2s.

### Formato CSV (16 campos)

```
temperature, pressure, temperature_barometer, humidity,
windSpeed, windDirection, windSpeedFiltered, windDirectionFiltered,
light, dht_temperature, dht_humidity,
rssi, free_heap, uptime_s, relay_active, soil_moisture
```

---

## Relay y electroválvulas

Los relays **JQC-3FF-S-Z** son **activo-LOW**:

| GPIO | Estado | Relay | Válvula |
|------|--------|:-----:|---------|
| HIGH | Arranque / seguro | OFF | Cerrada |
| LOW | Activado | ON | Abierta |

El campo `relay_active` es un bitmask: bit 0 = relay 1, bit 1 = relay 2, etc.

**Control MQTT:** Flask publica `{"relay": N, "state": bool}` → `mqttCallback()` actúa el relay en <100ms.

**Control HTTP:** polling `GET /api/relay/command` cada 2s → latencia ~2s.

---

## Pantalla TFT

Solo `PROFILE_METEO`. Resolución **240×135 px** (ST7789). Doble buffer con `TFT_eSprite` → sin parpadeo.

### Pantallas

| Pantalla | Cuándo se muestra |
|----------|------------------|
| **Boot** | Primeros segundos del arranque |
| **Setup AP** | Sin credenciales WiFi — muestra SSID y contraseña del AP |
| **Datos** | Funcionamiento normal — sensores en tiempo real |

### Pantalla de datos

```
┌──────────────────────────────────────┐
│ METEOSTATION   ☀350lx  WiFi  ●       │  ← ● verde=OK / rojo=error
├──────────┬──────────┬────────────────┤
│ T.EXT    │ T.INT    │ HUMEDAD        │
│ 22.5 C   │ 21.8 C   │ 65.2 %        │
│ [OK]     │ [OK]     │ [OK]          │
├──────────┼──────────┼────────────────┤
│ PRESION  │ VIENTO   │ DIRECC.        │
│ 101.3KPa │ 3.5 m/s  │ 180 deg S     │
│ [OK]     │          │               │
└──────────────────────────────────────┘
```

Los botones (GPIO0/BOOT y GPIO35) reactivan la pantalla y reinician el timer de apagado (60s).

---

## Filtros y estabilización

### Anemómetro — media móvil circular (10 muestras, 100ms)

Suaviza picos ADC. La velocidad filtrada se publica como `windSpeedFiltered`.

### Dirección del viento — promedio vectorial

Evita el error de la media aritmética cerca del norte (350° + 10° → 0°, no 180°):

```cpp
// Cada 100ms (Core 1)
windSumX += cos(deg * PI / 180);
windSumY += sin(deg * PI / 180);

// Cada 20s (Core 0)
float dir = atan2(windSumY, windSumX) * 180 / PI;
if (dir < 0) dir += 360;
```

### YL-69 humedad suelo — media móvil circular (10 muestras)

Buffer precalentado en `setup()` para evitar transitorios al arrancar. Calibración:

```cpp
#define SOIL_RAW_DRY   3300   // ADC tierra seca   (~0%)
#define SOIL_RAW_WET   1000   // ADC tierra húmeda (~100%)
```

---

## Simulación de sensores

Si un sensor no responde al arrancar, el firmware continúa con valores simulados que derivan suavemente:

| Sensor | Rango | Drift/lectura |
|--------|-------|:-------------:|
| MCP9808 | −10 a 45 °C | ±0.05 °C |
| HTU2x temperatura | −10 a 45 °C | ±0.05 °C |
| HTU2x humedad | 20–95 % | ±0.20 % |
| DHT11 temperatura | −10 a 45 °C | ±0.05 °C |
| DHT11 humedad | 20–95 % | ±0.20 % |
| Barómetro | 95–110 kPa | ±0.02 kPa |
| Luz | 0–2000 lux | ±5 lux |
| Viento | 0–15 m/s | ±0.30 m/s |
| Dirección | 0–360° | ±2.5° |
| Humedad suelo | 0–100 % | ±0.5 % |

La pantalla TFT muestra `[OK]` o `[SIM]` por cada sensor.

---

## Modo debug (rama `test`)

La rama `test` tiene `#define DEBUG_MODE 1` activo. Cada 5s imprime por Serial:

```
====== AQUANTIA TEST REPORT ======
[PERFIL] METEO (1) | Relays: 1 | Display: SI | MQTT: SI
[TIEMPO] Uptime: 0h 01m 23s | Heap libre: 245312 B
[DEVICE] Serial: AQ-FCB467F37748
[WIFI  ] CONECTADO | IP: 192.168.1.9 | RSSI: -62 dBm
[SENSOR] MCP9808:OK  Barometro:OK  DHT11:OK  HTU2x:OK  LuzAmb:OK
[DATOS ] T_MCP:22.1 C  T_HTU:21.8 C  H_HTU:65.2%  Lux:312.0 lx
[VIENTO] Speed:3.5 m/s (filt:3.3) | Dir:180 grados (S)
[RELAYS] R1=OFF
==================================
```

Útil para verificar el funcionamiento completo sin sensores conectados (modo simulación activo para los sensores ausentes).

### Pipeline: simulación y modo hardware

La beta.2 deja preparada la transición al caudalímetro real:

- `pipeline_mode = sim | real`
- recepción inmediata de configuración por MQTT dirigida por MAC
- fallback por lectura HTTP de `/api/pipeline/config`
- si el sensor real aún no está montado, el firmware mantiene `pipeline_source = fallback` y sigue usando el simulador

---

## Ahorro energético

| Medida | Configuración | Ahorro estimado |
|--------|---------------|-----------------|
| CPU a 160 MHz | `setCpuFrequencyMhz(160)` | ~20% vs 240 MHz |
| Modem Sleep | `WiFi.setSleep(true)` tras conectar | ~15–20 mA entre transmisiones |
| Pantalla con timeout | Apagado automático tras 60s sin actividad | Variable según brillo |

---

## Problemas conocidos

| Problema | Estado |
|----------|--------|
| Presión en kPa en vez de hPa | Pendiente — cambiar `readPressure(KPA)` → `readPressure(PA) / 100.0` |
| DHT11 lecturas inestables ocasionalmente | Conocido — valorar reemplazar por DHT22 o SHT31 |

---

## Versionado y flujo de trabajo

### Estructura de ramas

```
main              ← producción (solo merges de release/* o hotfix/*)
develop           ← integración continua
feature/*         ← nueva funcionalidad (sale de develop, merge a develop)
release/vX.Y.Z    ← congelado para pruebas (sale de develop)
hotfix/*          ← parche urgente sobre main
```

**Regla principal:** nunca se trabaja directamente en `main`. Todo cambio pasa por `develop` y, cuando llega a producción, lo hace a través de una rama `release/`.

### Versionado semántico (SemVer)

El firmware sigue `MAJOR.MINOR.PATCH[-prerelease]`:

| Incremento | Cuándo |
|------------|--------|
| `PATCH` | Corrección de bug sin cambio de API/protocolo |
| `MINOR` | Nueva funcionalidad compatible con backend anterior |
| `MAJOR` | Cambio de protocolo MQTT o HTTP incompatible |

Ejemplos de ciclo: `v0.1.0-beta.1` → `v0.1.0-rc.1` → `v0.1.0` → `v0.1.1` → `v0.2.0`

### Cómo actualizar la versión del firmware

La versión se define en una sola línea al inicio de `ESP_monitor_server.ino`:

```cpp
#define FIRMWARE_VERSION "0.1.0-beta.2"
```

Este valor se envía automáticamente al backend en dos momentos:
- **Arranque**: mensaje MQTT `aquantia/<finca_id>/register` → campo `firmware_version`
- **Registro HTTP**: POST `/api/device_info` → campo `firmware_version`

El backend lo almacena en `device_info.firmware_version` y lo muestra como badge en el dashboard.

### Proceso de release

```bash
# 1. Crear rama de release desde develop
git checkout develop && git pull
git checkout -b release/v0.2.0

# 2. Actualizar FIRMWARE_VERSION en el .ino
#    Editar: #define FIRMWARE_VERSION "0.2.0-rc.1"

# 3. Actualizar CHANGELOG.md con los cambios

# 4. Commit de cierre de release
git add ESP_monitor_server/ESP_monitor_server.ino CHANGELOG.md
git commit -m "chore: bump firmware to v0.2.0-rc.1"

# 5. Etiquetar
git tag -a v0.2.0-rc.1 -m "Release candidate v0.2.0-rc.1"
git push origin release/v0.2.0 --tags

# 6. Tras validación, merge a main y develop
git checkout main && git merge --no-ff release/v0.2.0
git tag -a v0.2.0 -m "Release v0.2.0"
git push origin main --tags
git checkout develop && git merge --no-ff release/v0.2.0
git push origin develop
```

### Compatibilidad firmware ↔ backend

Ambos repositorios se versionan de forma independiente pero coordinada.
El backend mantiene en `app_settings` la clave `min_firmware_version` con la versión mínima aceptada. Cuando se introduce un cambio de protocolo incompatible, se incrementa `MAJOR` en ambos repos y se actualiza ese valor.

| Firmware | Backend app_meteo | Estado |
|----------|-------------------|--------|
| `v0.1.x` | `v0.1.x` | Compatible |
| `v0.2.0` | `v0.1.x` | Puede no funcionar — revisar CHANGELOG |

Ver también: [CHANGELOG.md](CHANGELOG.md) y el [README del backend](../app_meteo/app_meteo/README.md).
