# MeteoStation — Hardware BOM (Bill of Materials)

Documento para estimación de costes del proyecto de estación meteorológica casera.

---

## Descripción del proyecto

Estación meteorológica casera basada en microcontrolador ESP32 (LilyGo TTGO T-Display).
Recoge datos de temperatura exterior, temperatura interior, humedad, presión atmosférica,
velocidad y dirección del viento, y luz ambiente. Los envía cada 20 segundos por WiFi a
un servidor Flask en red local, donde se almacenan en SQLite y se visualizan en un
dashboard React en tiempo real.

Existe también una versión ESP8266 (NodeMCU) con las mismas funciones excepto veleta
de viento y pantalla TFT.

---

## Lista de componentes

### Microcontrolador

| # | Componente | Descripción técnica | Cant. |
|---|-----------|---------------------|:-----:|
| 1 | LilyGo TTGO T-Display (ESP32) | ESP32-D0WDQ6, 240 MHz, 16 MB flash, pantalla TFT ST7789 240×135 px integrada, WiFi + BT, USB-C | 1 |

> Alternativa más económica (versión sin pantalla): NodeMCU ESP8266 (funcionalidad reducida: sin veleta, sin TFT).

---

### Sensores

| # | Componente | Descripción técnica | Cant. |
|---|-----------|---------------------|:-----:|
| 2 | Adafruit MCP9808 | Sensor de temperatura de alta precisión ±0.25 °C, interfaz I2C (dirección 0x19), rango −40 a +125 °C, alimentación 2.7–5.5 V | 1 |
| 3 | HTU21D / HTU20D (HTU2x) | Sensor de temperatura + humedad relativa, I2C (dirección 0x40), precisión T ±0.3 °C / HR ±2 %, alimentación 3.3 V | 1 |
| 4 | SparkFun MicroPressure (MPR series) | Barómetro digital, I2C (dirección 0x18), rango 0–25 PSI (~0–172 kPa), alimentación 3.3 V | 1 |
| 5 | APDS-9930 / TSL2584 (clon) | Sensor de luz ambiente (lux) + proximidad, I2C (dirección 0x39), el firmware autodetecta entre APDS-9930 y TSL2584, alimentación 3.3 V | 1 |
| 6 | DHT11 | Sensor de temperatura + humedad secundario, protocolo 1-wire, precisión T ±2 °C / HR ±5 %, alimentación 3.3–5 V | 1 |
| 7 | Anemómetro analógico | Salida de tensión 0–3.3 V proporcional a velocidad 0–30 m/s, conexión a ADC (GPIO37 en ESP32 / A0 en ESP8266), requiere cable hasta unidad exterior | 1 |
| 8 | Veleta analógica | Salida de tensión 0–3.3 V proporcional a dirección 0–360°, conexión a ADC (GPIO36, solo ESP32, pin input-only), requiere cable hasta unidad exterior | 1 |

---

### Componentes pasivos y miscelánea

| # | Componente | Descripción técnica | Cant. |
|---|-----------|---------------------|:-----:|
| 9 | Resistencia pull-up 4.7 kΩ | Para línea DATA del DHT11 entre DATA y VCC (3.3 V) | 1 |
| 10 | Resistencias pull-up 4.7 kΩ (opcionales) | Para líneas SDA/SCL del bus I2C si los módulos no las incluyen | 2 |
| 11 | Protoboard / PCB de prototipo | Para montar los sensores y el cableado de forma estable | 1 |
| 12 | Cables Dupont macho-macho y macho-hembra | Para conexiones I2C, DHT11, ADC anemómetro/veleta | ~30 |
| 13 | Cable USB-C | Para alimentación y programación inicial del ESP32 | 1 |
| 14 | Fuente de alimentación 5 V / 1 A (USB) | Alimentación del ESP32 (o powerbank para exterior) | 1 |
| 15 | Caja estanca / carcasa | Para alojar el ESP32 y sensores en exterior. Tamaño orientativo: 10×7×4 cm, grado IP54 o superior | 1 |

---

## Resumen de interfaces

| Bus / Protocolo | Pines ESP32 | Dispositivos |
|-----------------|-------------|--------------|
| I2C (SDA/SCL) | GPIO 21 / GPIO 22 | MCP9808, HTU2x, SparkFun MicroPressure, APDS-9930 |
| 1-Wire (digital) | GPIO 15 | DHT11 |
| ADC 12-bit | GPIO 37 | Anemómetro |
| ADC 12-bit (input-only) | GPIO 36 | Veleta |
| SPI (TFT interno) | GPIO 18/19/5/16/23/4 | Pantalla ST7789 (integrada en TTGO T-Display) |

---

## Notas de alimentación

- Todos los sensores I2C operan a **3.3 V**. El TTGO T-Display tiene regulador 3.3 V integrado.
- El DHT11 acepta 3.3–5 V. Usar 3.3 V para compatibilidad directa con el ESP32.
- El anemómetro y la veleta analógicos deben tener salida 0–3.3 V (no conectar 5 V a los ADC del ESP32).
- Consumo estimado total en operación: ~200–300 mA a 5 V.

---

## Versión ESP8266 (alternativa de menor coste)

La misma funcionalidad puede implementarse con un **NodeMCU ESP8266** con las siguientes limitaciones:
- Sin pantalla TFT
- Sin veleta (solo un canal ADC disponible, usado por el anemómetro)
- Pines I2C: SDA=GPIO4 (D2), SCL=GPIO5 (D1)
- Pin DHT11: GPIO14 (D5)
- Pin anemómetro: A0

Componentes que **no se necesitan** en la versión ESP8266:
- LilyGo TTGO T-Display (sustituir por NodeMCU ESP8266, ~3–5 €)
- Veleta analógica

---

## Consulta de precios sugerida

Para calcular el coste total del proyecto, buscar los precios actuales de cada componente en:
- AliExpress (envío desde China, 2–4 semanas)
- Amazon España / Amazon.de (entrega rápida, precio más alto)
- Tiendas de electrónica local (BricoGeek, Mouser, Farnell para componentes de calidad)

Precios orientativos (mercado europeo, 2024):
- LilyGo TTGO T-Display: ~10–15 €
- MCP9808 (módulo Adafruit): ~6–8 €
- HTU21D (módulo): ~3–5 €
- SparkFun MicroPressure: ~10–15 €
- APDS-9930 / TSL2584 (módulo clon): ~2–4 €
- DHT11 (módulo): ~1–2 €
- Anemómetro analógico: ~5–20 € (varía mucho según calidad)
- Veleta analógica: ~5–15 €
- Componentes pasivos + cables + PCB: ~3–5 €
- Caja estanca: ~3–8 €

**Coste total estimado ESP32: ~50–100 €** (dependiendo de fuente de compra y calidad del anemómetro/veleta)
**Coste total estimado ESP8266 (sin pantalla ni veleta): ~30–60 €**
