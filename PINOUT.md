# Aquantia — Pinout de dispositivos

Dos perfiles de hardware, un mismo firmware. El perfil se selecciona en tiempo de compilación con `DEVICE_PROFILE`.

---

## PROFILE_METEO (1) — LilyGo TTGO T-Display

**Placa:** LilyGo TTGO T-Display (ESP32-D0WDQ6, 4 MB flash, pantalla ST7789 240×135 integrada)

### GPIO del firmware

| Función | GPIO | Notas |
|---------|:----:|-------|
| I2C SDA | 21 | Bus de sensores |
| I2C SCL | 22 | Bus de sensores |
| DHT11 datos | 15 | Pull-up 4.7 kΩ a 3.3 V |
| Anemómetro (ADC) | 37 | 0–3.3 V → 0–30 m/s, ADC1_CH1 |
| Veleta dirección (ADC) | 36 | 0–3.3 V → 0–360°, ADC1_CH0 (input-only) |
| Humedad suelo YL-69 (ADC) | 33 | ADC1_CH5, divisor de tensión |
| Relay 1 (electroválvula) | 26 | Activo-LOW, JQC-3FF-S-Z |
| Botón izquierdo (BOOT) | 0 | INPUT_PULLUP, activo LOW |
| Botón derecho | 35 | INPUT, activo LOW |
| LED onboard | 2 | LED_ON=LOW, LED_OFF=HIGH |

### Pantalla TFT — SPI (ST7789 240×135)

Configurada mediante `TFT_eSPI` → `User_Setups/Setup25_TTGO_T_Display.h`. No modificar en el código del sketch.

| Señal TFT | GPIO |
|-----------|:----:|
| MOSI (SDA) | 19 |
| SCLK (SCK) | 18 |
| CS | 5 |
| DC (RS) | 16 |
| RST | 23 |
| Backlight (PWM) | 4 |

### Sensores I2C (SDA=21, SCL=22)

| Sensor | Dirección I2C | Función |
|--------|:-------------:|---------|
| Adafruit MCP9808 | 0x19 | Temperatura exterior (±0.0625 °C) |
| SparkFun MicroPressure | 0x18 | Presión barométrica |
| HTU2x (HTU21D / HTU20D) | 0x40 | Temperatura + humedad relativa |
| TSL2584 / APDS-9930 (clon) | 0x39 | Luz ambiente (lux) — autodetección |

> Sensores confirmados en hardware real con I2C scanner: `0x18, 0x19, 0x39, 0x40`

### Diagrama de conexiones externas

```
LilyGo TTGO T-Display
┌────────────────────┐
│  3V3 ──────────────┼──► VCC sensores I2C / DHT11 / divisor YL-69
│  GND ──────────────┼──► GND común
│  GPIO21 (SDA) ─────┼──► SDA → MCP9808, MicroPressure, HTU2x, TSL2584
│  GPIO22 (SCL) ─────┼──► SCL → (mismos sensores)
│  GPIO15 ───────────┼──► DHT11 DATA  (pull-up 4.7kΩ a 3.3V)
│  GPIO37 ───────────┼──► Anemómetro salida analógica 0–3.3V
│  GPIO36 ───────────┼──► Veleta salida analógica 0–3.3V
│  GPIO33 ───────────┼──► YL-69 AO (tras divisor de tensión)
│  GPIO26 ───────────┼──► IN del relay (JQC-3FF-S-Z)
│  GPIO0  ───────────┼──► Botón BOOT (ya integrado en placa)
│  GPIO35 ───────────┼──► Botón derecho externo
└────────────────────┘
```

### Calibración del sensor de suelo (YL-69)

Ajustar en `ESP_monitor_server.ino` según el sensor real:

```cpp
#define SOIL_RAW_DRY   3300   // Lectura ADC en tierra seca   (~0%)
#define SOIL_RAW_WET   1000   // Lectura ADC en tierra húmeda (~100%)
```

---

## PROFILE_IRRIGATION (2) — ESP32 4-Relay Board

**Placa:** ESP32-WROOM-32E (ESP32-D0WD-V3)
**Referencia:** [ESP32 Relay x4 — ESPHome devices](https://devices.esphome.io/devices/ESP32-Relay-x4/)

### GPIO del firmware

| Función | GPIO | Notas |
|---------|:----:|-------|
| Relay 1 (zona 1) | 32 | Activo-LOW, JQC-3FF-S-Z |
| Relay 2 (zona 2) | 33 | Activo-LOW |
| Relay 3 (zona 3) | 25 | Activo-LOW |
| Relay 4 (zona 4) | 26 | Activo-LOW |
| LED estado | 23 | Integrado en placa |
| I2C SDA | 21 | Sin sensores en v actual |
| I2C SCL | 22 | Sin sensores en v actual |

> Este perfil no tiene sensores meteorológicos ni pantalla. Solo gestiona los 4 relays por MQTT.

### Bitmask de relays

El campo `relay_active` en la telemetría y en los comandos es un bitmask de 4 bits:

| Valor | Estado |
|-------|--------|
| `0`  | Todos OFF |
| `1`  | Relay 1 ON (bit 0) |
| `2`  | Relay 2 ON (bit 1) |
| `4`  | Relay 3 ON (bit 2) |
| `8`  | Relay 4 ON (bit 3) |
| `15` | Los 4 relays ON |

### Diagrama de conexiones

```
ESP32 4-Relay Board
┌────────────────────┐
│  GPIO32 ───────────┼──► IN1 relay 1 (zona riego 1)
│  GPIO33 ───────────┼──► IN2 relay 2 (zona riego 2)
│  GPIO25 ───────────┼──► IN3 relay 3 (zona riego 3)
│  GPIO26 ───────────┼──► IN4 relay 4 (zona riego 4)
│  GPIO23 ───────────┼──► LED estado (integrado)
│  VCC / GND ────────┼──► Alimentación 5 V / GND común
└────────────────────┘

Cada relay:
  COM ──► neutro / positivo de la válvula
  NO  ──► electroválvula 24 VAC / 12 VDC
```

---

## Comportamiento del relay (ambos perfiles)

Los relays **JQC-3FF-S-Z** son **activo-LOW**:

| GPIO | Estado | Relay | Válvula |
|------|--------|:-----:|---------|
| HIGH | Arranque / seguro | OFF | Cerrada |
| LOW | Activado | ON | Abierta |

Durante una actualización OTA todos los relays pasan a OFF (HIGH) automáticamente por seguridad.

---

## Anemómetro y veleta (PROFILE_METEO)

### Anemómetro — velocidad del viento

Salida analógica 0–3.3 V proporcional a 0–30 m/s:

| Parámetro | Valor |
|-----------|-------|
| Pin | GPIO37 (ADC1_CH1) |
| Resolución ADC | 12 bit (0–4095) |
| Referencia | 3.41 V |
| Fórmula | `speed = (raw / ADC_RANGE) * ADC_VOLTAGE_REF / 3.3 * 30.0` |
| Filtro | Media móvil circular de 10 muestras, muestreo cada 100 ms |

### Veleta — dirección del viento

Salida analógica 0–3.3 V proporcional a 0–360°:

| Parámetro | Valor |
|-----------|-------|
| Pin | GPIO36 (ADC1_CH0, input-only) |
| Resolución ADC | 12 bit (0–4095) |
| Filtro | Promedio vectorial (`atan2` de suma de vectores unitarios) |

> GPIO36 es input-only en ESP32 — no conectar como salida.

---

## DHT11 — sensor secundario (PROFILE_METEO)

Temperatura y humedad de respaldo, independiente del bus I2C.

| Parámetro | Valor |
|-----------|-------|
| Pin | GPIO15 |
| Pull-up | 4.7 kΩ entre DATA y 3.3 V |
| Librería | `DHTesp` (beegee-tokyo) |

> Sensor **primario** de temperatura y humedad: HTU2x por I2C 0x40.
> DHT11 aparece como campos adicionales `dht_temperature` / `dht_humidity` en la telemetría.

---

## Alimentación

- Todos los sensores I2C y el DHT11 operan a **3.3 V**.
- El relay JQC-3FF-S-Z acepta bobina a 5 V (alimentado desde la placa, no desde el GPIO).
- Las electroválvulas conectadas a los NO del relay operan habitualmente a **24 VAC** o **12 VDC** con fuente externa.
- Las líneas I2C trabajan a 3.3 V — **no conectar a 5 V**.
