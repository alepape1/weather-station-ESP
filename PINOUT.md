# MeteoStation — Pinout completo

## ESP8266 (sin pantalla)

### Pines definidos en firmware

| Función          | GPIO (número) | Etiqueta NodeMCU |
|------------------|:-------------:|:----------------:|
| I2C SDA          | 4             | D2               |
| I2C SCL          | 5             | D1               |
| DHT11 datos      | 14            | D5               |
| Anemómetro (ADC) | A0            | A0               |
| LED onboard      | 2             | D4               |

### Sensores I2C (SDA=D2, SCL=D1)

| Sensor                       | Dirección I2C | Función                    |
|------------------------------|:-------------:|----------------------------|
| MCP9808                      | 0x19          | Temperatura exterior       |
| SparkFun MicroPressure       | 0x18 (def.)   | Presión barométrica        |
| TSL2584 / APDS-9930 (clon)   | 0x39          | Luz ambiente (lux)         |

> El sensor de viento (veleta de dirección) **no está disponible** en ESP8266
> por falta de un segundo canal ADC. La dirección se envía siempre como 0.

---

## ESP32 (con pantalla TFT 240×135)

### Pines definidos en firmware

| Función           | GPIO |
|-------------------|:----:|
| I2C SDA           | 21   |
| I2C SCL           | 22   |
| DHT11 datos       | 15   |
| Anemómetro (ADC)  | 37   |
| Veleta (ADC)      | 36   |
| LED onboard       | 2    |

### Pines pantalla TFT (SPI) — TTGO T-Display 240×135

> Estos pines se configuran en el archivo `User_Setup.h` de la librería TFT_eSPI.
> Los valores siguientes corresponden al TTGO T-Display (LilyGO).

| Función TFT | GPIO |
|-------------|:----:|
| MOSI (SDA)  | 19   |
| SCLK (SCK)  | 18   |
| CS          | 5    |
| DC (RS)     | 16   |
| RST         | 23   |
| Backlight   | 4    |

### Sensores I2C (SDA=21, SCL=22)

| Sensor                     | Dirección I2C | Función                    |
|----------------------------|:-------------:|----------------------------|
| MCP9808                    | 0x19          | Temperatura exterior       |
| SparkFun MicroPressure     | 0x18 (def.)   | Presión barométrica        |
| TSL2584 / APDS-9930 (clon) | 0x39          | Luz ambiente (lux)         |

---

## DHT11

El DHT11 usa un único pin de datos (sin I2C). Conectar:
- VCC → 3.3 V
- GND → GND
- DATA → GPIO definido arriba (D5 en ESP8266, GPIO15 en ESP32)
- Resistencia pull-up de **4.7 kΩ** entre DATA y VCC

## Anemómetro (velocidad del viento)

Salida de tensión 0–3.3 V proporcional a la velocidad (0–30 m/s):
- ESP8266: pin A0 (ADC 10 bit, ref. 3.2 V, rango 0–1023)
- ESP32:   GPIO37 (ADC 12 bit, ref. 3.41 V, rango 0–4095)

## Veleta (dirección del viento) — solo ESP32

Salida de tensión 0–3.3 V proporcional al ángulo (0–360°):
- ESP32: GPIO36 (pin input-only, ADC 12 bit)

---

## Notas de alimentación

- Todos los sensores I2C operan a **3.3 V**.
- El DHT11 acepta 3.3–5 V; usar 3.3 V cuando se conecte al ESP8266/ESP32.
- Las líneas I2C del ESP8266 son tolerantes a 3.3 V; **no conectar a 5 V**.
