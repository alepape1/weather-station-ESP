# MeteoStation — Firmware ESP32

Firmware para la estación meteorológica casera. El ESP32 recoge datos de temperatura, humedad, presión y viento, los muestra en una pantalla TFT y los envía cada 20 segundos al servidor Flask por HTTP.

Repositorio del servidor y dashboard: [alepape1/app_meteo](https://github.com/alepape1/app_meteo)

---

## Índice

- [Hardware](#hardware)
- [Conexiones](#conexiones)
- [Librerías necesarias](#librerías-necesarias)
- [Configuración antes de compilar](#configuración-antes-de-compilar)
- [Arquitectura del firmware](#arquitectura-del-firmware)
- [Algoritmos](#algoritmos)
- [Formato de datos enviados al servidor](#formato-de-datos-enviados-al-servidor)
- [Pantalla TFT](#pantalla-tft)
- [Problemas conocidos](#problemas-conocidos)

---

## Hardware

| Componente | Modelo | Función |
|------------|--------|---------|
| Microcontrolador | ESP32 DevKitC | Procesamiento y WiFi |
| Sensor temperatura exterior | Adafruit MCP9808 | Temperatura de alta precisión (I2C) |
| Sensor temperatura + humedad | DHT11 | Temperatura interior y humedad relativa |
| Barómetro | SparkFun MicroPressure | Presión atmosférica (I2C) |
| Anemómetro | Analógico | Velocidad del viento (ADC) |
| Veleta | Analógica | Dirección del viento (ADC) |
| Pantalla | TFT 240×135 (TFT_eSPI) | Visualización local en tiempo real |
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
| Pantalla TFT | Según User_Setup.h | SPI (configurado en librería TFT_eSPI) |

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

> La librería `TFT_eSPI` requiere configurar el archivo `User_Setup.h` con los pines SPI de tu pantalla antes de compilar.

---

## Configuración antes de compilar

Edita estas líneas en el `.ino`:

```cpp
// Red WiFi
const char* ssid       = "TU_RED_WIFI";
const char* password   = "TU_PASSWORD";

// IP del servidor Flask (Raspberry Pi)
const char* server_ip  = "192.168.1.42";
const int   server_port = 5000;
```

> Las credenciales WiFi están actualmente en texto plano en el código.
> No subas el archivo al repositorio con tus credenciales reales o usa un archivo `secrets.h` excluido del `.gitignore`.

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

## Algoritmos

### Velocidad del viento — Media móvil clásica

Se mantiene un buffer circular de 10 lecturas ADC del anemómetro. La velocidad filtrada es la media de esas 10 lecturas convertida a m/s.

```cpp
#define FILTER_SIZE 10

float getWindSpeed(int adcValue) {
    float voltage = adcValue * (ADC_VOLTAGE_REFERENCE / ADC_RANGE);
    return (voltage / ADC_VOLTAGE_REFERENCE) * 30.0; // 0-30 m/s
}
```

### Dirección del viento — Promedio vectorial

La dirección del viento es un ángulo circular. Una media aritmética simple produce resultados incorrectos cerca del norte (ejemplo: media de 350° y 10° sería 180° en vez de 0°).

El firmware acumula vectores unitarios en coordenadas cartesianas durante los 20 segundos del intervalo de envío y calcula el ángulo resultante con `atan2`:

```cpp
// Cada 100ms — acumular
void accumulateWindVector(float degrees) {
    float rad = degrees * PI / 180.0;
    windSumX += cos(rad);
    windSumY += sin(rad);
    windSampleCount++;
}

// Cada 20s — calcular y resetear
float calculateAndResetWindVector() {
    float avgRad = atan2(windSumY, windSumX);
    float avgDeg = avgRad * 180.0 / PI;
    if (avgDeg < 0) avgDeg += 360.0;
    windSumX = windSumY = windSampleCount = 0;
    return avgDeg;
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
| 5 | `currentWindDirDegrees` | Veleta | ° | Dirección instantánea (0-315°, pasos de 45°) |
| 6 | `windSpeedFiltered` | Anemómetro | m/s | Media móvil de 10 muestras |
| 7 | `finalAverageWindDir` | Veleta | ° | Promedio vectorial de 20s |

> **Nota sobre la presión:** `barometer.readPressure(KPA)` devuelve kilopascales.
> La presión atmosférica normal (1013 hPa) aparece como ~101.3 en el servidor.
> Para corregirlo cambiar a `barometer.readPressure(PA) / 100.0` y el servidor recibirá hPa directamente.

---

## Pantalla TFT

La pantalla se actualiza cada segundo y muestra:

```
┌─────────────────────────────┐
│ METEOSTATION            [●] │  ← círculo verde=OK / rojo=sin servidor
├─────────────────────────────┤
│ T.Int:  20.5 °C             │  ← DHT11
│ Hum:    58.2 %              │  ← DHT11
│ Pres:   101.3 KPa           │  ← MicroPressure
│ Viento: 4.2ms SO            │  ← velocidad filtrada + dirección instantánea
│ T.Ext:  21.43 °C            │  ← MCP9808
└─────────────────────────────┘
```

El círculo de estado en la esquina superior derecha se pone **verde** si el último envío al servidor fue exitoso (HTTP 200/201) o **rojo** si falló o no había WiFi.

---

## Problemas conocidos

| Problema | Estado | Solución |
|----------|--------|----------|
| Presión en KPa en vez de hPa | Pendiente | Cambiar `readPressure(KPA)` por `readPressure(PA) / 100.0` |
| Credenciales WiFi en texto plano | Pendiente | Mover a `secrets.h` y añadirlo al `.gitignore` |
| DHT11 puede dar lecturas inestables | Conocido | El sensor puede estar dañado; valorar reemplazar por DHT22 o SHT31 |
