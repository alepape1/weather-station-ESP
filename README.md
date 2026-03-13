# MeteoStation — Firmware ESP32 v3

Firmware para la estación meteorológica casera. El ESP32 recoge datos de temperatura, humedad, presión y viento, los muestra en una pantalla TFT y los envía cada 20 segundos al servidor Flask por HTTP.

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Hardware](#hardware)
- [Conexiones](#conexiones)
- [Librerías necesarias](#librerías-necesarias)
- [Configuración antes de compilar](#configuración-antes-de-compilar)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Simulación de sensores](#simulación-de-sensores)
- [Algoritmos](#algoritmos)
- [Formato de datos enviados al servidor](#formato-de-datos-enviados-al-servidor)
- [Pantalla TFT](#pantalla-tft)
- [Problemas conocidos](#problemas-conocidos)

---

## Hardware

| Componente | Modelo | Función |
|------------|--------|---------|
| Microcontrolador | LilyGo T-Display-S3 (ESP32-S3) | Procesamiento, WiFi y pantalla integrada |
| Sensor temperatura exterior | Adafruit MCP9808 | Temperatura de alta precisión (I2C) |
| Sensor temperatura + humedad | DHT11 | Temperatura interior y humedad relativa |
| Barómetro | SparkFun MicroPressure | Presión atmosférica (I2C) |
| Anemómetro | Analógico | Velocidad del viento (ADC) |
| Veleta | Analógica | Dirección del viento (ADC) |
| Pantalla | ST7789 320×170 integrada (TFT_eSPI) | Dashboard visual en tiempo real |
| LED | GPIO 2 | Indicador de envío activo |

---

## Conexiones

| Sensor | Pin ESP32 | Función |
|--------|-----------|---------|
| MCP9808 | GPIO 21 | I2C SDA |
| MCP9808 | GPIO 22 | I2C SCL |
| SparkFun MicroPressure | GPIO 21 | I2C SDA (bus compartido) |
| SparkFun MicroPressure | GPIO 22 | I2C SCL (bus compartido) |
| DHT11 | GPIO 15 | Data |
| Anemómetro | GPIO 37 | ADC (entrada analógica) |
| Veleta | GPIO 36 | ADC (entrada analógica) |
| LED indicador | GPIO 2 | Salida digital |
| Pantalla TFT | Integrada en LilyGo T-Display-S3 | SPI (configurado en `User_Setup.h` de TFT_eSPI) |

> El MCP9808 tiene dirección I2C `0x19`.
> El barómetro y el MCP9808 comparten el mismo bus I2C (pines 21 y 22).

---

## Librerías necesarias

Instalar desde el gestor de librerías de Arduino IDE:

| Librería | Autor |
|----------|-------|
| `TFT_eSPI` | Bodmer |
| `Adafruit MCP9808 Library` | Adafruit |
| `SparkFun MicroPressure Library` | SparkFun Electronics |
| `DHTesp` | beegee-tokyo |

> La librería `TFT_eSPI` requiere configurar el archivo `User_Setup.h` con los pines SPI de la pantalla ST7789 del LilyGo T-Display-S3 antes de compilar.

---

## Configuración antes de compilar

Las credenciales de red se guardan en `secrets.h` (excluido del repositorio por `.gitignore`). Crea el archivo manualmente junto al `.ino`:

```cpp
// secrets.h
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"
#define SERVER_IP     "192.168.1.X"
#define SERVER_PORT   5000
```

> `secrets.h` está en `.gitignore` — nunca se sube al repositorio.

---

## Arquitectura del firmware

El `loop()` principal tiene **tres bloques independientes** controlados por temporizadores no bloqueantes con `millis()`. Cada uno tiene su propio intervalo y ninguno depende de los demás:

```
loop()
 ├─ Cada 100ms  → Leer anemómetro y veleta, acumular vector de viento
 ├─ Cada 1s     → Leer DHT11, MCP9808, barómetro. Actualizar pantalla TFT
 └─ Cada 20s    → Calcular promedio vectorial, enviar HTTP POST al servidor
```

Esto garantiza que la pantalla siempre se actualiza y los sensores siempre se leen, independientemente de si el servidor está disponible o no.

---

## Simulación de sensores

En el `setup()` el firmware detecta cada sensor de forma independiente. Si un sensor no responde (o falla durante el funcionamiento), **el firmware continúa automáticamente en modo simulación** para ese sensor, sin detener el resto del sistema.

Los valores simulados varían lentamente (drift aleatorio acotado) para emular condiciones reales:

| Sensor | Rango simulado | Drift por segundo |
|--------|---------------|-------------------|
| MCP9808 (T.Ext) | −10 a 45 °C | ±0.05 °C |
| DHT11 (T.Int) | −10 a 45 °C | ±0.05 °C |
| DHT11 (Humedad) | 20 a 95 % | ±0.20 % |
| Barómetro | 95 a 110 KPa | ±0.02 KPa |

> El anemómetro y la veleta son ADC puro — siempre disponibles, no necesitan simulación.

### Pantalla de arranque

Al encender, la pantalla muestra el resultado de la detección antes de pasar al dashboard:

```
┌─────────────────────────────────────────┐
│  METEOSTATION  v3                       │
├─────────────────────────────────────────┤
│  MCP9808  (T.Ext)     [ REAL ]          │
│  Barometro            [  SIM ]          │
│  DHT11    (T.Int)     [ REAL ]          │
│                                         │
│         Conectando WiFi...              │
└─────────────────────────────────────────┘
```

Badge verde `REAL` = sensor detectado y funcionando.
Badge naranja `SIM` = sensor no detectado, datos simulados.

---

## Algoritmos

### Velocidad del viento — Media móvil clásica

Se mantiene un buffer circular de 10 lecturas ADC del anemómetro. La velocidad filtrada es la media de esas 10 lecturas convertida a m/s.

```cpp
#define FILTER_SIZE 10

float adcToWindSpeed(float adc) {
    float v = adc * (ADC_VOLTAGE_REF / ADC_RANGE);
    return (v / ADC_VOLTAGE_REF) * 30.0f; // 0-30 m/s
}
```

### Dirección del viento — Promedio vectorial

La dirección del viento es un ángulo circular. Una media aritmética simple produce resultados incorrectos cerca del norte (ejemplo: media de 350° y 10° sería 180° en vez de 0°).

El firmware acumula vectores unitarios en coordenadas cartesianas durante los 20 segundos del intervalo de envío y calcula el ángulo resultante con `atan2`:

```cpp
// Cada 100ms — acumular
void accumulateWindVector(float degrees) {
    float rad = degrees * PI / 180.0f;
    windSumX += cos(rad);
    windSumY += sin(rad);
    windSampleCount++;
}

// Cada 20s — calcular y resetear
float calcAndResetWindVector() {
    float deg = atan2(windSumY, windSumX) * 180.0f / PI;
    if (deg < 0) deg += 360.0f;
    windSumX = windSumY = 0; windSampleCount = 0;
    return deg;
}
```

Esto produce un promedio matemáticamente correcto para ángulos circulares.

---

## Formato de datos enviados al servidor

El ESP32 envía un HTTP POST a `/send_message` con cuerpo en texto plano, exactamente 8 valores separados por coma:

```
temperaturaMCP, presion, temperaturaDHT, humedad, velocidadViento, direccionViento, velocidadVientoFiltrada, direccionVientoPromediada
```

### Ejemplo

```
21.43,101.32,20.10,58.50,4.20,225.00,3.95,218.74
```

### Mapeo de campos

| Posición | Variable | Sensor | Unidad | Nota |
|----------|----------|--------|--------|------|
| 0 | `temperatureMCP` | MCP9808 | °C | Temperatura exterior de alta precisión |
| 1 | `pressure` | MicroPressure | KPa | Ver nota abajo |
| 2 | `temperatureDHT` | DHT11 | °C | Temperatura interior |
| 3 | `humidity` | DHT11 | % | Humedad relativa |
| 4 | `windSpeed` | Anemómetro | m/s | Lectura instantánea cruda |
| 5 | `currentWindDirDeg` | Veleta | ° | Dirección instantánea (0-315°, pasos de 45°) |
| 6 | `windSpeedFiltered` | Anemómetro | m/s | Media móvil de 10 muestras |
| 7 | `finalAvgWindDir` | Veleta | ° | Promedio vectorial de 20s |

> **Nota sobre la presión:** `barometer.readPressure(KPA)` devuelve kilopascales.
> La presión atmosférica normal (1013 hPa) aparece como ~101.3 en el servidor.
> Para corregirlo cambiar a `barometer.readPressure(PA) / 100.0` y el servidor recibirá hPa directamente.

---

## Pantalla TFT

Resolución: **320×170 px** (ST7789, LilyGo T-Display-S3). Renderizado sin parpadeo con `TFT_eSprite` (doble buffer en RAM, `pushSprite` atómico).

La pantalla se actualiza cada segundo y muestra 6 tarjetas en una cuadrícula 2×3:

```
┌──────────────────────────────────────────────────────┐
│ METEOSTATION               WiFi  ●                   │  ← cabecera (●verde=OK/rojo=error)
├────────────┬────────────┬─────────────────────────────┤
│ 🌡 T.EXT  │ 🌡 T.INT  │ 💧 HUMEDAD                 │
│  21.4 C   │  19.8 C   │  62.0 %                     │
│      [OK] │      [SIM]│        [OK]                 │
├────────────┼────────────┼─────────────────────────────┤
│ ⬤ PRESION │ ~ VIENTO  │ ⊕ DIRECC.                  │
│ 101.3 KPa │   3.9 m/s │  225 deg   SO               │
│      [SIM]│      [OK] │        [OK]                 │
└──────────────────────────────────────────────────────┘
```

Cada tarjeta muestra:
- **Icono** dibujado con primitivas TFT (termómetro, gota, manómetro, viento, brújula)
- **Valor** en fuente grande (Font 4, 26px)
- **Badge** `[OK]` en verde si el dato proviene del sensor físico, `[SIM]` en naranja si es simulado
- **Franja de color** en la parte superior de la tarjeta (verde=real, naranja=simulado)

El círculo en la esquina superior derecha de la cabecera se pone **verde** si el último envío al servidor fue exitoso (HTTP 200/201) o **rojo** si falló o no había WiFi.

---

## Problemas conocidos

| Problema | Estado | Solución |
|----------|--------|----------|
| Presión en KPa en vez de hPa | Pendiente | Cambiar `readPressure(KPA)` por `readPressure(PA) / 100.0` |
| DHT11 puede dar lecturas inestables | Conocido | El sensor puede estar dañado; valorar reemplazar por DHT22 o SHT31 |
