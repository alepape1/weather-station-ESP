# Changelog — Aquantia Firmware (ESP32)

Todos los cambios notables de este proyecto se documentan en este archivo.
Formato basado en [Keep a Changelog](https://keepachangelog.com/es/1.1.0/).
Versiones siguiendo [Semantic Versioning](https://semver.org/lang/es/).

> **Compatibilidad backend:** Cada versión del firmware indica el backend compatible.
> Ver [app_meteo](https://github.com/alepape1/app_meteo) para las versiones del dashboard.

---

## [v0.1.0-beta.1] — 2026-04-13

**Backend compatible:** `v0.1.0-beta.1`

Primera versión beta pública. Firmware completo con MQTT/TLS, perfiles de dispositivo,
provisioning SoftAP, Flash Tool GUI y sistema de autenticación DEV/PROD.

### Añadido
- **MQTT sobre TLS** (puerto 8883): `PubSubClient` + `WiFiClientSecure`, certificado CA configurable
- **FreeRTOS dual-core**: Core 1 = sensores + display, Core 0 = red/MQTT (`networkTask`)
- **Perfiles de dispositivo en tiempo de compilación**:
  - `PROFILE_METEO` (defecto): MCP9808, HTU2x, MicroPressure, TSL2584/APDS-9930, DHT11, YL-69, pantalla TFT 240×135
  - `PROFILE_IRRIGATION`: 4 relays (GPIO 32/33/25/26), sin sensores meteo
- **Provisioning SoftAP**: captive portal con lista WiFi clicable, credenciales guardadas en NVS, factory reset con botón BOOT
- **DEV/PROD mode**: `#define DEV_MODE` usa `secrets.h` directamente; sin `DEV_MODE` usa MAC + token NVS para MQTT
- **Timestamp NTP**: campo `ts` (epoch Unix) en telemetría — el backend lo usa como timestamp real de la medición
- **Flash Tool GUI** (`flasher_gui.py`): flash USB, OTA, perfiles, DEV/PROD toggle, registro CSV, QR de claim, botón borrar NVS
- **Serial de dispositivo**: formato `AQ-{MAC}` (sin Flash ID), generado desde NVS
- **QR de claim**: codifica la URL completa `https://meteo.aquantialab.com/claim?serial=AQ-...`
- **Sensor YL-69**: humedad de suelo en GPIO 33, campo `soil_moisture` en payload MQTT
- **Simulador pipeline**: presión y caudal con ruido sinusoidal determinista (coincide con `pipeline_sim.py` del backend)
- **Modo simulación por sensor**: cada sensor tiene un flag `_ok`; si falla, usa valores simulados con drift lento
- **OTA seguro**: apaga todos los relays antes de iniciar flash, contraseña OTA opcional vía `secrets.h`
- **Debug Mode**: `#define DEBUG_MODE` activa reporte completo cada 5s con estado de todos los subsistemas

### Cambiado
- Perfiles `PROFILE_METEO`/`PROFILE_IRRIGATION` movidos al inicio del .ino (antes de cualquier `#if`)
- `mqtt_user`/`mqtt_pass` solo se sobreescriben con MAC+token NVS dentro de `#ifndef DEV_MODE`
- Redondeo de floats en payload MQTT: 2 decimales para temperatura/humedad/viento, 1 para dirección/luz/suelo (ahorro ~15% tamaño)
- Autodetección de sensor de luz: TSL2584 (ID `0xAx`) y APDS-9930 (ID `0x3x`) en dirección I2C `0x39`
- HTU2x implementado sin librería externa (I2C directo), con calentamiento de arranque 3s
- Pantalla TFT apagada tras 60s de inactividad, cualquier botón la enciende

### Corregido
- Guard `#ifndef DEV_MODE` para asignar `mqtt_user` = MAC (antes se sobreescribía siempre)
- Pantalla AP mostrada antes de `provisioning_start_ap()` (evita pantalla negra durante configuración)
- Progress bar del Flash Tool se movía incorrectamente
- `python.exe` en lugar de `pythonw.exe` para OTA (evita WinError 10053 en Windows)
- Tabla de particiones real leída antes de borrar NVS (evita borrar partición incorrecta)
- `windMux` declarado antes de `accumulateWindVector` (crash en compilación)

### Perfiles de hardware soportados

| Componente | PROFILE_METEO | PROFILE_IRRIGATION |
|------------|:---:|:---:|
| MCP9808 (T exterior) | ✓ | — |
| HTU2x (T+H interior) | ✓ | ✓ |
| SparkFun MicroPressure | ✓ | — |
| TSL2584 / APDS-9930 (luz) | ✓ | ✓ |
| DHT11 (T+H backup) | ✓ | — |
| YL-69 (humedad suelo) | ✓ | — |
| Anemómetro + veleta | ✓ | — |
| Pantalla TFT 240×135 | ✓ | — |
| Relays | 1 (GPIO 26) | 4 (GPIO 32/33/25/26) |

---

## [Sin versión — rama main] — 2026-03-26 a 2026-04-01

Primera versión operativa con FreeRTOS. Incluida en `v0.1.0-beta.1`.

- Refactor firmware v4: FreeRTOS dual-core
- Perfiles METEO/IRRIGATION con `DEVICE_PROFILE`
- Multi-relay (bitmask `relay_active`)
- Sensor YL-69 (campo 16 en CSV HTTP legacy)
- Simulador pipeline en firmware
- Header `X-Device-MAC` en HTTP POST
- OTA scripts (`ota_flash.sh`) compatibles Linux/Windows

---

[v0.1.0-beta.1]: https://github.com/alepape1/weather-station-ESP/releases/tag/v0.1.0-beta.1
