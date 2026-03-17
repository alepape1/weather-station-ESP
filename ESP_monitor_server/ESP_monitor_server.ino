// MeteoStation — Firmware v3
// Compatible con ESP32 (con pantalla ST7789 240×135) y ESP8266 (sin pantalla)
// Sensores: MCP9808, HTU2x, SparkFun MicroPressure, TSL2584/APDS, anemómetro, veleta
// Tres temporizadores independientes: 100ms viento / 1s pantalla / 20s envío

// ── Detección de plataforma ───────────────────────────────────────────────────
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
  #include <ArduinoOTA.h>
  #define I2C_SDA          4
  #define I2C_SCL          5
  #define DHTPIN          14   // D5
  #define ADC_VOLTAGE_REF  3.2f
  #define ADC_RANGE        1024.0f
  #define ANEMOMETER_PIN   A0
#else  // ESP32
  #include "WiFi.h"
  #include <HTTPClient.h>
  #include <ArduinoOTA.h>
  #define I2C_SDA          21
  #define I2C_SCL          22
  #define DHTPIN           15
  #define ADC_VOLTAGE_REF  3.41f
  #define ADC_RANGE        4096.0f
  #define ANEMOMETER_PIN   37
  #define VANE_PIN         36
  #define HAS_DISPLAY
#endif

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <SparkFun_MicroPressure.h>
#include <DHTesp.h>
#include <math.h>
#include "secrets.h"

#ifdef HAS_DISPLAY
  #include <TFT_eSPI.h>
#endif

// ── Credenciales (definidas en secrets.h) ─────────────────────────────────────
const char* ssid        = WIFI_SSID;
const char* password    = WIFI_PASSWORD;
const char* server_ip   = SERVER_IP;
const int   server_port = SERVER_PORT;

// ── Pines ─────────────────────────────────────────────────────────────────────
const int ledPin = 2;
#define LED_ON  LOW
#define LED_OFF HIGH

// ── Intervalos ─────────────────────────────────────────────────────────────────
#define WIND_MS    100
#define SCREEN_MS 1000
#define SEND_MS  20000

// ── Objetos ────────────────────────────────────────────────────────────────────
SparkFun_MicroPressure barometer;
Adafruit_MCP9808       tempsensor = Adafruit_MCP9808();
DHTesp                 dht;

#ifdef HAS_DISPLAY
  TFT_eSPI    tft = TFT_eSPI();
  TFT_eSprite spr = TFT_eSprite(&tft);
#endif

// ── Flags de sensor ────────────────────────────────────────────────────────────
bool mcp_ok = false;
bool bar_ok = false;
bool htu_ok = false;
bool dht_ok = false;
bool tsl_ok = false;

// ── Valores medidos ────────────────────────────────────────────────────────────
float  temperatureMCP    = 0;
float  temperatureDHT    = 0;   // HTU2x temperatura
float  humidity          = 0;   // HTU2x humedad
float  temperatureDHT11  = 0;   // DHT11 temperatura
float  humidityDHT11     = 0;   // DHT11 humedad
double pressure          = 0;
float  windSpeed         = 0;
float  windSpeedFiltered = 0;
float  currentWindDirDeg = 0;
float  lightLevel        = 0;

// ── Valores simulados (drift lento) ───────────────────────────────────────────
float sim_tempMCP   = 20.5f;
float sim_tempDHT   = 19.8f;
float sim_humidity  = 62.0f;
float sim_tempDHT11 = 20.1f;
float sim_humDHT11  = 60.0f;
float sim_pressure  = 101.3f;
float sim_light     = 300.0f;
float sim_windSpeed = 3.5f;
float sim_windDir   = 180.0f;

static float driftClamp(float v, float mn, float mx, float step) {
  float d = ((float)random(-100, 101) / 100.0f) * step;
  return constrain(v + d, mn, mx);
}

void updateSimulatedValues() {
  sim_tempMCP   = driftClamp(sim_tempMCP,   -10.0f,  45.0f,  0.05f);
  sim_tempDHT   = driftClamp(sim_tempDHT,   -10.0f,  45.0f,  0.05f);
  sim_humidity  = driftClamp(sim_humidity,   20.0f,  95.0f,  0.20f);
  sim_tempDHT11 = driftClamp(sim_tempDHT11, -10.0f,  45.0f,  0.05f);
  sim_humDHT11  = driftClamp(sim_humDHT11,   20.0f,  95.0f,  0.20f);
  sim_pressure  = driftClamp(sim_pressure,   95.0f, 110.0f,  0.02f);
  sim_light     = driftClamp(sim_light,       0.0f, 2000.0f, 5.0f);
  sim_windSpeed = driftClamp(sim_windSpeed,   0.0f,   15.0f, 0.3f);
  sim_windDir   = fmod(sim_windDir + ((float)random(-10, 11) / 10.0f) * 5.0f + 360.0f, 360.0f);
}

// =============================================================================
// HTU2x (HTU21D / HTU20D / SHT21) — temperatura y humedad por I2C
// Dirección fija: 0x40. Sin librería externa.
// =============================================================================
#define HTU2X_ADDR        0x40
#define HTU2X_CMD_TEMP    0xF3   // medir temperatura, no-hold master
#define HTU2X_CMD_HUM     0xF5   // medir humedad,     no-hold master
#define HTU2X_CMD_RESET   0xFE

bool htu_begin() {
  Wire.beginTransmission(HTU2X_ADDR);
  Wire.write(HTU2X_CMD_RESET);
  if (Wire.endTransmission() != 0) return false;
  delay(15);  // tiempo de reset (máx 15 ms según datasheet)
  return true;
}

// Lanza una medición y espera. Devuelve NAN si falla.
static float htu_measure(uint8_t cmd, uint16_t wait_ms) {
  Wire.beginTransmission(HTU2X_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission() != 0) return NAN;
  delay(wait_ms);
  Wire.requestFrom((uint8_t)HTU2X_ADDR, (uint8_t)3, (uint8_t)1);
  if (Wire.available() < 2) return NAN;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  if (Wire.available()) Wire.read();  // descarta CRC
  raw &= 0xFFFC;  // limpia los 2 bits de tipo de medición
  return (float)raw;
}

float htu_readTemp() {
  float raw = htu_measure(HTU2X_CMD_TEMP, 50);
  if (isnan(raw)) return NAN;
  return -46.85f + 175.72f * (raw / 65536.0f);
}

float htu_readHumidity() {
  float raw = htu_measure(HTU2X_CMD_HUM, 20);
  if (isnan(raw)) return NAN;
  return constrain(-6.0f + 125.0f * (raw / 65536.0f), 0.0f, 100.0f);
}

// =============================================================================
// Sensor de luz ambiente — autodetección TSL2584 / APDS-9930 (I2C directo)
// Ambos chips usan dirección 0x39 y protocolo CMD=0x80|reg.
// TSL2584:  datos en 0x0C-0x0F, ID en 0x0A (bits[7:4]=0xA)
// APDS-9930: datos en 0x14-0x17, ID en 0x12 (=0x39)
// =============================================================================
#define LIGHT_ADDR     0x39
#define LIGHT_CMD      0x80   // bit CMD obligatorio en el byte de registro

// Registros comunes (misma dirección en ambos chips)
#define LIGHT_CTRL     0x00   // ENABLE / POWER
#define LIGHT_TIMING   0x01   // TIMING (TSL) / ATIME (APDS)

// Registros exclusivos TSL2584
#define TSL_ID_REG     0x0A   // Part number en bits[7:4]; TSL2584 = 0xA
#define TSL_D0L        0x0C   // CH0 low  (visible + IR)
#define TSL_D1L        0x0E   // CH1 low  (IR)

// Registros exclusivos APDS-9930
#define APDS_ID_REG    0x12   // Chip ID; APDS-9930 = 0x39
#define APDS_CDATAL    0x14   // Clear channel low  (visible + IR)
#define APDS_IRDATAL   0x16   // IR channel low

static bool light_is_apds = false;  // se establece en tsl_begin()

static bool light_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LIGHT_ADDR);
  Wire.write(LIGHT_CMD | reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Lectura stop-start (más compatible con ESP8266 que repeated-start)
static uint8_t light_read8(uint8_t reg) {
  Wire.beginTransmission(LIGHT_ADDR);
  Wire.write(LIGHT_CMD | reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  Wire.requestFrom((uint8_t)LIGHT_ADDR, (uint8_t)1, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static uint16_t light_read16(uint8_t regL) {
  uint16_t lo = light_read8(regL);
  uint16_t hi = light_read8(regL + 1);
  return (hi << 8) | lo;
}

// Enciende el sensor, detecta el tipo y configura la integración.
// Devuelve true si el sensor responde en el bus.
bool tsl_begin() {
  if (!light_write(LIGHT_CTRL, 0x03)) return false;  // Power ON + ALS enable
  delay(10);

  // Autodetección: leer ambos registros de ID y mostrarlos
  uint8_t apds_id = light_read8(APDS_ID_REG);  // 0x12 → 0x39 o 0x30 para APDS-9930
  uint8_t tsl_id  = light_read8(TSL_ID_REG);   // 0x0A → 0xAx para TSL2584
  Serial.printf("[LUZ] ID@0x12=0x%02X  ID@0x0A=0x%02X\n", apds_id, tsl_id);

  // APDS-9930: ID puede ser 0x39 (rev1) o 0x30 (rev0); nibble alto = 0x3
  if ((apds_id & 0xF0) == 0x30) {
    light_is_apds = true;
    // ATIME=0xDB → 37 ciclos × 2.73 ms ≈ 101 ms por conversión
    light_write(LIGHT_TIMING, 0xDB);
    Serial.printf("[LUZ] APDS-9930 detectado (ID=0x%02X)\n", apds_id);
  } else if ((tsl_id & 0xF0) == 0xA0) {
    light_is_apds = false;
    light_write(LIGHT_TIMING, 0x02);  // 402 ms, ganancia 1×
    Serial.printf("[LUZ] TSL2584 detectado (ID=0x%02X)\n", tsl_id);
  } else {
    // ID desconocido — probar APDS-9930 como fallback (más probable en 0x39)
    light_is_apds = true;
    light_write(LIGHT_TIMING, 0xDB);
    Serial.printf("[LUZ] ID desconocido — usando registros APDS-9930 (fallback)\n");
  }

  delay(450);  // Esperar primera integración completa (cubre ambos casos)
  return true;
}

// Devuelve lux estimado a partir del canal visible+IR y el canal IR.
// Usa la fórmula TSL258x (válida también como aproximación para APDS-9930).
float tsl_readLux() {
  uint16_t ch0, ch1;
  if (light_is_apds) {
    ch0 = light_read16(APDS_CDATAL);   // clear (visible + IR)
    ch1 = light_read16(APDS_IRDATAL);  // IR
  } else {
    ch0 = light_read16(TSL_D0L);
    ch1 = light_read16(TSL_D1L);
  }

  if (ch0 == 0) return 0.0f;

  float ratio = (float)ch1 / (float)ch0;
  float lux;

  if      (ratio <= 0.52f) lux = 0.0315f * ch0 - 0.0593f * ch0 * pow(ratio, 1.4f);
  else if (ratio <= 0.65f) lux = 0.0229f * ch0 - 0.0291f * ch1;
  else if (ratio <= 0.80f) lux = 0.0157f * ch0 - 0.0180f * ch1;
  else if (ratio <= 1.30f) lux = 0.00338f * ch0 - 0.00260f * ch1;
  else                     lux = 0.0f;

  return max(0.0f, lux);
}

// ── Filtro media móvil (velocidad viento) ──────────────────────────────────────
#define FILTER_SIZE 10
int anemometerValues[FILTER_SIZE] = {};
int aneIdx = 0;

float filteredADC(int newVal) {
  anemometerValues[aneIdx] = newVal;
  aneIdx = (aneIdx + 1) % FILTER_SIZE;
  long s = 0;
  for (int i = 0; i < FILTER_SIZE; i++) s += anemometerValues[i];
  return (float)s / FILTER_SIZE;
}

float adcToWindSpeed(float adc) {
  float v = adc * (ADC_VOLTAGE_REF / ADC_RANGE);
  return (v / ADC_VOLTAGE_REF) * 30.0f;
}

#ifdef HAS_DISPLAY
float adcToWindDeg(int adc) {
  float d = (adc / ADC_RANGE) * 360.0f;
  if (d >= 337.5f || d < 22.5f) return 0.0f;
  if (d < 67.5f)                 return 45.0f;
  if (d < 112.5f)                return 90.0f;
  if (d < 157.5f)                return 135.0f;
  if (d < 202.5f)                return 180.0f;
  if (d < 247.5f)                return 225.0f;
  if (d < 292.5f)                return 270.0f;
  return 315.0f;
}
#else
float adcToWindDeg(int) { return 0.0f; }
#endif

const char* degToCompass(float d) {
  const char* dirs[] = {"N","NE","E","SE","S","SO","O","NO"};
  return dirs[((int)(d + 22.5f) / 45) % 8];
}

// ── Promedio vectorial de dirección ────────────────────────────────────────────
float windSumX = 0, windSumY = 0;
int   windSampleCount = 0;
float finalAvgWindDir = 0;

void accumulateWindVector(float deg) {
  float r = deg * PI / 180.0f;
  windSumX += cos(r);
  windSumY += sin(r);
  windSampleCount++;
}

float calcAndResetWindVector() {
  if (windSampleCount == 0) return 0;
  float deg = atan2(windSumY, windSumX) * 180.0f / PI;
  if (deg < 0) deg += 360.0f;
  windSumX = windSumY = 0;
  windSampleCount = 0;
  return deg;
}

// ── Estado ────────────────────────────────────────────────────────────────────
bool lastServerOK    = false;
unsigned long lastSendTime   = 0;
unsigned long lastScreenTime = 0;

// ── Info estática del hardware ────────────────────────────────────────────────
void printHardwareInfo() {
  Serial.println("\n=== Hardware Info ===");
#ifdef ESP8266
  Serial.printf("Chip         : ESP8266 (ID=0x%08X)\n", ESP.getChipId());
#else
  Serial.printf("Chip         : %s rev%d\n", ESP.getChipModel(), ESP.getChipRevision());
#endif
  Serial.printf("CPU          : %d MHz\n",      ESP.getCpuFreqMHz());
  Serial.printf("Flash        : %d MB\n",        ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Free Heap    : %d bytes\n",     ESP.getFreeHeap());
  Serial.printf("SDK          : %s\n",           ESP.getSdkVersion());
  Serial.printf("MAC          : %s\n",           WiFi.macAddress().c_str());
  Serial.printf("IP           : %s\n",           WiFi.localIP().toString().c_str());
  Serial.println("====================");
}

void postDeviceInfo() {
  String chipModel;
  int    chipRevision;
#ifdef ESP8266
  chipModel    = "ESP8266";
  chipRevision = 0;
#else
  chipModel    = String(ESP.getChipModel());
  chipRevision = (int)ESP.getChipRevision();
#endif

  String json = "{";
  json += "\"chip_model\":\""  + chipModel + "\",";
  json += "\"chip_revision\":" + String(chipRevision) + ",";
  json += "\"cpu_freq_mhz\":"  + String(ESP.getCpuFreqMHz()) + ",";
  json += "\"flash_size_mb\":" + String(ESP.getFlashChipSize() / (1024 * 1024)) + ",";
  json += "\"sdk_version\":\""  + String(ESP.getSdkVersion()) + "\",";
  json += "\"mac_address\":\""  + WiFi.macAddress() + "\",";
  json += "\"ip_address\":\""   + WiFi.localIP().toString() + "\"";
  json += "}";

  String url = "http://" + String(server_ip) + ":" + String(server_port) + "/api/device_info";
  HTTPClient http;
  http.setTimeout(3000);
#ifdef ESP8266
  WiFiClient wifiClient;
  http.begin(wifiClient, url);
#else
  http.begin(url);
#endif
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  http.end();
  Serial.printf("[DeviceInfo] POST → %d\n", code);
}

// ── HTTP helper ───────────────────────────────────────────────────────────────
bool httpPost(const String& url, const String& body) {
  HTTPClient http;
  http.setTimeout(3000);
#ifdef ESP8266
  WiFiClient wifiClient;
  http.begin(wifiClient, url);
#else
  http.begin(url);
#endif
  http.addHeader("Content-Type", "text/plain");
  int code = http.POST(body);
  http.end();
  return (code == 200 || code == 201);
}

// =============================================================================
// PANTALLA TFT (solo ESP32)
// =============================================================================
#ifdef HAS_DISPLAY

#define C_BG      0x0841
#define C_CARD    0x1082
#define C_HDR     0x0421
#define C_BORDER  0x2124
#define C_TEXT    0xFFFF
#define C_LABEL   0x7BEF
#define C_SIM     0xFD20
#define C_REAL    0x07E0
#define C_RED     0xF800

// Layout 240×135 — 2 filas × 3 cols + fila extra para luz
// Fila 0-1: misma cuadrícula 2×3 de siempre (tarjetas 79×57)
// La luz se muestra como barra inferior de 16px en la cabecera (valor en header)
#define HDR_H    18
#define CARD_W   79
#define CARD_H   57

static int cardX(int col) { return col * (CARD_W + 1); }
static int cardY(int row) { return HDR_H + row * (CARD_H + 2); }

void iconThermometer(int cx, int cy, uint16_t col) {
  spr.fillRoundRect(cx - 2, cy - 7, 5, 10, 2, col);
  spr.fillCircle(cx, cy + 5, 4, col);
  spr.drawFastVLine(cx, cy - 7, 8, C_CARD);
}

void iconDrop(int cx, int cy, uint16_t col) {
  spr.fillCircle(cx, cy + 4, 4, col);
  for (int i = 0; i <= 7; i++) {
    int hw = (7 - i) / 2;
    spr.drawFastHLine(cx - hw, cy - 3 + i, hw * 2 + 1, col);
  }
}

void iconGauge(int cx, int cy, uint16_t col) {
  spr.drawCircle(cx, cy, 7, col);
  spr.drawLine(cx, cy, cx + 4, cy - 4, col);
  spr.fillCircle(cx, cy, 2, col);
  spr.drawFastHLine(cx - 5, cy + 5, 11, col);
}

void iconWind(int cx, int cy, uint16_t col) {
  spr.drawFastHLine(cx - 6, cy - 3, 12, col);
  spr.drawLine(cx + 6, cy - 3, cx + 8, cy - 1, col);
  spr.drawFastHLine(cx - 6, cy,     10, col);
  spr.drawFastHLine(cx - 6, cy + 3, 12, col);
}

void iconCompass(int cx, int cy, uint16_t col) {
  spr.drawCircle(cx, cy, 7, col);
  spr.fillTriangle(cx, cy - 6, cx - 2, cy, cx + 2, cy, col);
  spr.fillTriangle(cx, cy + 6, cx - 2, cy, cx + 2, cy, C_LABEL);
}

// Icono sol simple para luz
void iconSun(int cx, int cy, uint16_t col) {
  spr.fillCircle(cx, cy, 4, col);
  for (int a = 0; a < 360; a += 45) {
    float r = a * PI / 180.0f;
    int x1 = cx + (int)(6 * cos(r));
    int y1 = cy + (int)(6 * sin(r));
    int x2 = cx + (int)(9 * cos(r));
    int y2 = cy + (int)(9 * sin(r));
    spr.drawLine(x1, y1, x2, y2, col);
  }
}

typedef void (*IconFn)(int, int, uint16_t);
IconFn iconFns[6] = {
  iconThermometer,
  iconThermometer,
  iconDrop,
  iconGauge,
  iconWind,
  iconCompass,
};

void drawCard(int col, int row,
              const char* label,
              float value, const char* unit,
              bool simulated,
              bool showCompass = false) {
  int x = cardX(col);
  int y = cardY(row);

  spr.fillRoundRect(x + 1, y + 1, CARD_W - 2, CARD_H - 2, 4, C_CARD);
  spr.drawRoundRect(x, y, CARD_W, CARD_H, 4, C_BORDER);

  uint16_t accentCol = simulated ? C_SIM : C_REAL;
  spr.fillRoundRect(x + 1, y + 1, CARD_W - 2, 3, 2, accentCol);

  int idx = row * 3 + col;
  iconFns[idx](x + 10, y + 13, accentCol);

  spr.setTextColor(C_LABEL, C_CARD);
  spr.drawString(label, x + 22, y + 5, 1);

  spr.setTextColor(C_TEXT, C_CARD);
  char buf[12];
  snprintf(buf, sizeof(buf), showCompass ? "%.0f" : "%.1f", value);
  spr.drawString(buf, x + 4, y + 18, 4);

  spr.setTextColor(C_LABEL, C_CARD);
  spr.drawString(unit, x + 4, y + CARD_H - 13, 1);

  if (showCompass) {
    spr.setTextColor(accentCol, C_CARD);
    spr.drawString(degToCompass(value), x + 42, y + CARD_H - 13, 2);
  }

  const char* badge = simulated ? "[SIM]" : "[OK]";
  spr.setTextColor(accentCol, C_CARD);
  spr.drawRightString(badge, x + CARD_W - 2, y + CARD_H - 13, 1);
}

void drawScreen() {
  spr.fillSprite(C_BG);

  // Cabecera con luz integrada a la derecha del título
  spr.fillRect(0, 0, 240, HDR_H, C_HDR);
  spr.setTextColor(C_TEXT, C_HDR);
  spr.drawString("METEOSTATION", 6, 4, 2);

  // Luz en la cabecera (icono + valor lux)
  uint16_t luxCol = tsl_ok ? C_REAL : C_SIM;
  iconSun(118, 9, luxCol);
  char luxBuf[10];
  snprintf(luxBuf, sizeof(luxBuf), "%.0flx", lightLevel);
  spr.setTextColor(luxCol, C_HDR);
  spr.drawString(luxBuf, 130, 5, 1);

  if (WiFi.status() == WL_CONNECTED) {
    spr.setTextColor(C_REAL, C_HDR);
    spr.drawString("WiFi", 168, 5, 1);
  } else {
    spr.setTextColor(C_RED, C_HDR);
    spr.drawString("NoWiFi", 157, 5, 1);
  }

  uint16_t srvCol = lastServerOK ? C_REAL : C_RED;
  spr.fillCircle(230, 9, 5, srvCol);
  spr.drawCircle(230, 9, 5, C_TEXT);

  drawCard(0, 0, "T.EXT",   temperatureMCP,     "C",    !mcp_ok);
  drawCard(1, 0, "T.INT",   temperatureDHT,     "C",    !htu_ok);
  drawCard(2, 0, "HUMEDAD", humidity,           "%",    !htu_ok);
  drawCard(0, 1, "PRESION", (float)pressure,    "KPa",  !bar_ok);
  drawCard(1, 1, "VIENTO",  windSpeedFiltered,  "m/s",  false);
  drawCard(2, 1, "DIRECC.", currentWindDirDeg,  "deg",  false, true);

  spr.pushSprite(0, 0);
}

void drawBootScreen(const char* wifiMsg) {
  spr.fillSprite(C_BG);

  spr.fillRect(0, 0, 240, HDR_H, C_HDR);
  spr.setTextColor(C_TEXT, C_HDR);
  spr.drawCentreString("METEOSTATION  v3", 120, 4, 2);

  struct { const char* lbl; bool ok; } sensors[4] = {
    { "MCP9808  (T.Ext)", mcp_ok },
    { "Barometro       ", bar_ok },
    { "HTU2x    (T+H)  ", htu_ok },
    { "LuzAmb   (0x39) ", tsl_ok },
  };

  for (int i = 0; i < 4; i++) {
    int y = 22 + i * 24;
    spr.setTextColor(C_LABEL, C_BG);
    spr.drawString(sensors[i].lbl, 8, y, 2);

    uint16_t badgeCol    = sensors[i].ok ? C_REAL : C_SIM;
    const char* badgeTxt = sensors[i].ok ? "  REAL  " : "  SIM  ";
    spr.fillRoundRect(163, y, 60, 16, 3, badgeCol);
    spr.setTextColor(TFT_BLACK, badgeCol);
    spr.drawCentreString(badgeTxt, 193, y + 4, 1);
  }

  spr.setTextColor(C_LABEL, C_BG);
  spr.drawCentreString(wifiMsg, 120, 120, 2);

  spr.pushSprite(0, 0);
}

#endif  // HAS_DISPLAY

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== MeteoStation BOOT ===");

  Serial.println("Iniciando I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C OK");

  // Escaner I2C — detecta todos los dispositivos en el bus
  Serial.println("Escaneando bus I2C...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Dispositivo en 0x%02X\n", addr);
      found++;
    }
    delay(2);
  }
  if (found == 0) Serial.println("  Ningun dispositivo encontrado");
  else Serial.printf("  Total: %d dispositivos\n", found);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LED_OFF);

#ifdef HAS_DISPLAY
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  spr.createSprite(240, 135);
  spr.setSwapBytes(true);
#endif

  mcp_ok = tempsensor.begin(0x19);
  if (mcp_ok) {
    tempsensor.setResolution(3);
    Serial.println("MCP9808 OK");
  } else {
    Serial.println("MCP9808 no detectado — modo simulacion");
    temperatureMCP = sim_tempMCP;
  }

  bar_ok = barometer.begin();
  if (bar_ok) {
    Serial.println("Barometro OK");
  } else {
    Serial.println("Barometro no detectado — modo simulacion");
    pressure = sim_pressure;
  }

  htu_ok = htu_begin();
  if (htu_ok) {
    float t = htu_readTemp();
    float h = htu_readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temperatureDHT = t;
      humidity       = h;
      Serial.println("HTU2x OK");
    } else {
      htu_ok = false;
    }
  }
  if (!htu_ok) {
    Serial.println("HTU2x no detectado — modo simulacion");
    temperatureDHT = sim_tempDHT;
    humidity       = sim_humidity;
  }

  // DHT11
  dht.setup(DHTPIN, DHTesp::DHT11);
  delay(dht.getMinimumSamplingPeriod());
  {
    TempAndHumidity th = dht.getTempAndHumidity();
    dht_ok = (dht.getStatus() == DHTesp::ERROR_NONE);
    if (dht_ok) {
      temperatureDHT11 = th.temperature;
      humidityDHT11    = th.humidity;
      Serial.println("DHT11 OK");
    } else {
      Serial.println("DHT11 no detectado — modo simulacion");
      temperatureDHT11 = sim_tempDHT11;
      humidityDHT11    = sim_humDHT11;
    }
  }

  // TSL2584 — el begin() ya espera la primera integración (450ms)
  tsl_ok = tsl_begin();
  if (tsl_ok) {
    Serial.println("Sensor luz OK");
    lightLevel = tsl_readLux();
  } else {
    Serial.println("Sensor luz no detectado — modo simulacion");
    lightLevel = sim_light;
  }

#ifdef ESP8266
  Serial.println("Plataforma: ESP8266 (sin pantalla, sin veleta)");
#else
  Serial.println("Plataforma: ESP32 (con pantalla TFT)");
#endif

#ifdef HAS_DISPLAY
  drawBootScreen("Conectando WiFi...");
#endif

  Serial.println("Conectando WiFi...");
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
#ifdef ESP8266
    yield();
#endif
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi OK: " + WiFi.localIP().toString());

    // ── OTA (Over-The-Air) ──────────────────────────────────────────────────
#ifdef ESP8266
    ArduinoOTA.setHostname("meteostation-esp8266");
#else
    ArduinoOTA.setHostname("meteostation-esp32");
#endif
    // Contraseña OTA opcional — definir OTA_PASSWORD en secrets.h para activarla
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() {
      Serial.println("\n[OTA] Inicio de actualización");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\n[OTA] Actualización completada — reiniciando");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] %u%%\r", progress * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("[OTA] Error %u\n", error);
    });
    ArduinoOTA.begin();
    Serial.println("OTA listo — hostname: " +
#ifdef ESP8266
      String("meteostation-esp8266")
#else
      String("meteostation-esp32")
#endif
    );
    // ── Fin OTA ─────────────────────────────────────────────────────────────

    printHardwareInfo();
    postDeviceInfo();

#ifdef HAS_DISPLAY
    drawBootScreen(("IP: " + WiFi.localIP().toString()).c_str());
#endif
  } else {
    Serial.println("Sin WiFi — continuando sin conexion (OTA no disponible)");
#ifdef HAS_DISPLAY
    drawBootScreen("Sin WiFi — modo offline");
#endif
  }

#ifdef HAS_DISPLAY
  delay(2500);
#endif
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  ArduinoOTA.handle();  // OTA — debe ser lo primero del loop

  unsigned long now = millis();

  // ── 1. Lectura ADC viento (cada 100ms) ──────────────────────────────────────
  static unsigned long lastWindRead = 0;
  if (now - lastWindRead >= WIND_MS) {
#ifdef HAS_DISPLAY
    int rawAne = analogRead(ANEMOMETER_PIN);
    float filtAne     = filteredADC(rawAne);
    windSpeed         = adcToWindSpeed((float)rawAne);
    windSpeedFiltered = adcToWindSpeed(filtAne);
    int rawVane       = analogRead(VANE_PIN);
    currentWindDirDeg = adcToWindDeg(rawVane);
#else
    // ESP8266 sin veleta: anemómetro y veleta simulados
    windSpeed         = sim_windSpeed;
    windSpeedFiltered = sim_windSpeed;
    currentWindDirDeg = sim_windDir;
#endif

    accumulateWindVector(currentWindDirDeg);
    lastWindRead = now;
  }

  // ── 2. Sensores I2C (MCP9808, HTU2x, barometro, luz) + pantalla (cada 1s) ───
  if (now - lastScreenTime >= SCREEN_MS) {

    // MCP9808
    if (mcp_ok) {
      tempsensor.wake();
      float t = tempsensor.readTempC();
      tempsensor.shutdown_wake(1);
      if (!isnan(t) && t > -40.0f) {
        temperatureMCP = t;
      } else {
        mcp_ok = false;
        Serial.println("MCP9808 fallo en lectura — cambiando a simulacion");
      }
    }
    if (!mcp_ok) temperatureMCP = sim_tempMCP;

    // Barómetro
    if (bar_ok) {
      double p = barometer.readPressure(KPA);
      if (p > 50.0) {
        pressure = p;
      } else {
        bar_ok = false;
        Serial.println("Barometro fallo en lectura — cambiando a simulacion");
      }
    }
    if (!bar_ok) pressure = sim_pressure;

    // HTU2x
    if (htu_ok) {
      float t = htu_readTemp();
      float h = htu_readHumidity();
      if (!isnan(t) && !isnan(h)) {
        temperatureDHT = t;
        humidity       = h;
      } else {
        htu_ok = false;
        Serial.println("HTU2x fallo en lectura — cambiando a simulacion");
      }
    }
    if (!htu_ok) {
      temperatureDHT = sim_tempDHT;
      humidity       = sim_humidity;
    }

    // DHT11
    {
      TempAndHumidity th = dht.getTempAndHumidity();
      if (dht.getStatus() == DHTesp::ERROR_NONE) {
        dht_ok           = true;
        temperatureDHT11 = th.temperature;
        humidityDHT11    = th.humidity;
      } else {
        dht_ok = false;
      }
    }
    if (!dht_ok) {
      temperatureDHT11 = sim_tempDHT11;
      humidityDHT11    = sim_humDHT11;
    }

    // TSL2584
    if (tsl_ok) {
      float lux = tsl_readLux();
      if (lux >= 0.0f) {
        lightLevel = lux;
      } else {
        tsl_ok = false;
        Serial.println("TSL2584 fallo en lectura — cambiando a simulacion");
      }
    }
    if (!tsl_ok) lightLevel = sim_light;

    updateSimulatedValues();

#ifdef HAS_DISPLAY
    drawScreen();
#else
    Serial.printf("[1s] T:%.1f Tb:%.1f H:%.1f D11T:%.1f D11H:%.1f P:%.2f W:%.2f D:%.0f Lux:%.1f\n",
      temperatureMCP, temperatureDHT, humidity,
      temperatureDHT11, humidityDHT11,
      (float)pressure, windSpeedFiltered, currentWindDirDeg, lightLevel);
#endif

    lastScreenTime = now;
  }

  // ── 3. Envío HTTP (cada 20s) ─────────────────────────────────────────────────
  if (now - lastSendTime >= SEND_MS) {
    finalAvgWindDir = calcAndResetWindVector();
    bool ok = false;

    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(ledPin, LED_ON);

      String url = "http://" + String(server_ip) + ":" + String(server_port) + "/send_message";
      // CSV: 11 valores — temperatura, presion, temp_bar, humedad,
      //                    viento, direccion, viento_filtrado, dir_filtrada, luz,
      //                    dht11_temperatura, dht11_humedad
      String msg = String(temperatureMCP, 2)    + "," +
                   String(pressure, 2)          + "," +
                   String(temperatureDHT, 2)    + "," +
                   String(humidity, 2)          + "," +
                   String(windSpeed, 2)         + "," +
                   String(currentWindDirDeg, 2) + "," +
                   String(windSpeedFiltered, 2) + "," +
                   String(finalAvgWindDir, 2)   + "," +
                   String(lightLevel, 2)        + "," +
                   String(temperatureDHT11, 2)  + "," +
                   String(humidityDHT11, 2)     + "," +
                   String(WiFi.RSSI())          + "," +
                   String((long)ESP.getFreeHeap()) + "," +
                   String((long)(millis() / 1000));

      Serial.println("TX: " + msg);
      ok = httpPost(url, msg);
      Serial.printf("HTTP %s\n", ok ? "200 OK" : "ERROR");
      digitalWrite(ledPin, LED_OFF);

    } else {
      Serial.println("WiFi desconectado — intentando reconectar");
      WiFi.reconnect();
    }

    lastServerOK = ok;
    lastSendTime = now;
  }
}
