// MeteoStation — Firmware v3
// Compatible con ESP32 (con pantalla ST7789 240×135) y ESP8266 (sin pantalla)
// Sensores: MCP9808, DHT11, SparkFun MicroPressure, TSL2584, anemómetro, veleta
// Tres temporizadores independientes: 100ms viento / 1s pantalla / 20s envío

// ── Detección de plataforma ───────────────────────────────────────────────────
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
  #define I2C_SDA          4
  #define I2C_SCL          5
  #define DHTPIN          14   // D5
  #define ADC_VOLTAGE_REF  3.2f
  #define ADC_RANGE        1024.0f
  #define ANEMOMETER_PIN   A0
#else  // ESP32
  #include "WiFi.h"
  #include <HTTPClient.h>
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
bool dht_ok = false;
bool tsl_ok = false;

// ── Valores medidos ────────────────────────────────────────────────────────────
float  temperatureMCP    = 0;
float  temperatureDHT    = 0;
float  humidity          = 0;
double pressure          = 0;
float  windSpeed         = 0;
float  windSpeedFiltered = 0;
float  currentWindDirDeg = 0;
float  lightLevel        = 0;   // lux — TSL2584

// ── Valores simulados (drift lento) ───────────────────────────────────────────
float sim_tempMCP  = 20.5f;
float sim_tempDHT  = 19.8f;
float sim_humidity = 62.0f;
float sim_pressure = 101.3f;
float sim_light    = 300.0f;

static float driftClamp(float v, float mn, float mx, float step) {
  float d = ((float)random(-100, 101) / 100.0f) * step;
  return constrain(v + d, mn, mx);
}

void updateSimulatedValues() {
  sim_tempMCP  = driftClamp(sim_tempMCP,  -10.0f,  45.0f,  0.05f);
  sim_tempDHT  = driftClamp(sim_tempDHT,  -10.0f,  45.0f,  0.05f);
  sim_humidity = driftClamp(sim_humidity,  20.0f,  95.0f,  0.20f);
  sim_pressure = driftClamp(sim_pressure,  95.0f, 110.0f,  0.02f);
  sim_light    = driftClamp(sim_light,      0.0f, 2000.0f, 5.0f);
}

// =============================================================================
// TSL2584 — Sensor de luz ambiente (I2C directo, sin librería externa)
// =============================================================================
#define TSL2584_ADDR   0x39
#define TSL2584_CMD    0x80
#define TSL2584_CTRL   0x00   // POWER
#define TSL2584_TIMING 0x01   // Integración y ganancia
#define TSL2584_D0L    0x0C   // CH0 low  (visible + IR)
#define TSL2584_D0H    0x0D   // CH0 high
#define TSL2584_D1L    0x0E   // CH1 low  (IR)
#define TSL2584_D1H    0x0F   // CH1 high

static bool tsl_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TSL2584_ADDR);
  Wire.write(TSL2584_CMD | reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static uint8_t tsl_read8(uint8_t reg) {
  Wire.beginTransmission(TSL2584_ADDR);
  Wire.write(TSL2584_CMD | reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)TSL2584_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static uint16_t tsl_read16(uint8_t regL) {
  // Dos lecturas separadas — más compatible con ESP8266
  uint16_t lo = tsl_read8(regL);
  uint16_t hi = tsl_read8(regL + 1);
  return (hi << 8) | lo;
}

// Vuelca todos los registros 0x00–0x0F por Serial (debug)
static void tsl_dumpRegs() {
  Serial.println("[TSL] Dump registros:");
  for (uint8_t r = 0x00; r <= 0x0F; r++) {
    Serial.printf("  reg 0x%02X = 0x%02X\n", r, tsl_read8(r));
  }
}

// Retorna true si el sensor responde en el bus I2C
bool tsl_begin() {
  if (!tsl_write(TSL2584_CTRL, 0x03)) return false;  // Power ON
  delay(5);
  // Integración 402ms, ganancia 1×
  tsl_write(TSL2584_TIMING, 0x02);
  delay(450);  // Esperar primera integración completa
  return true;
}

// Calcula lux a partir de CH0 (visible+IR) y CH1 (IR)
// Fórmula TSL258x para integración 402ms, ganancia 1×
float tsl_readLux() {
  uint16_t ch0 = tsl_read16(TSL2584_D0L);
  uint16_t ch1 = tsl_read16(TSL2584_D1L);

  Serial.printf("[TSL] CH0=%u CH1=%u\n", ch0, ch1);  // DEBUG — quitar cuando funcione

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
  drawCard(1, 0, "T.INT",   temperatureDHT,     "C",    !dht_ok);
  drawCard(2, 0, "HUMEDAD", humidity,           "%",    !dht_ok);
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
    { "DHT11    (T.Int)", dht_ok },
    { "TSL2584  (Luz)  ", tsl_ok },
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

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LED_OFF);

#ifdef HAS_DISPLAY
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  spr.createSprite(240, 135);
  spr.setSwapBytes(true);
#endif

  dht.setup(DHTPIN, DHTesp::DHT11);

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

  delay(dht.getMinimumSamplingPeriod());
  TempAndHumidity th = dht.getTempAndHumidity();
  dht_ok = (dht.getStatus() == DHTesp::ERROR_NONE);
  if (dht_ok) {
    Serial.println("DHT11 OK");
    temperatureDHT = th.temperature;
    humidity       = th.humidity;
  } else {
    Serial.println("DHT11 no detectado — modo simulacion");
    temperatureDHT = sim_tempDHT;
    humidity       = sim_humidity;
  }

  // TSL2584 — el begin() ya espera la primera integración (450ms)
  tsl_ok = tsl_begin();
  if (tsl_ok) {
    Serial.println("TSL2584 OK");
    tsl_dumpRegs();   // DEBUG — quitar cuando funcione
    lightLevel = tsl_readLux();
  } else {
    Serial.println("TSL2584 no detectado — modo simulacion");
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
#ifdef HAS_DISPLAY
    drawBootScreen(("IP: " + WiFi.localIP().toString()).c_str());
#endif
  } else {
    Serial.println("Sin WiFi — continuando sin conexion");
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
  unsigned long now = millis();

  // ── 1. Lectura ADC viento (cada 100ms) ──────────────────────────────────────
  static unsigned long lastWindRead = 0;
  if (now - lastWindRead >= WIND_MS) {
    int rawAne = analogRead(ANEMOMETER_PIN);

    float filtAne     = filteredADC(rawAne);
    windSpeed         = adcToWindSpeed((float)rawAne);
    windSpeedFiltered = adcToWindSpeed(filtAne);

#ifdef HAS_DISPLAY
    int rawVane       = analogRead(VANE_PIN);
    currentWindDirDeg = adcToWindDeg(rawVane);
#else
    currentWindDirDeg = 0.0f;
#endif

    accumulateWindVector(currentWindDirDeg);
    lastWindRead = now;
  }

  // ── 2. Sensores I2C + DHT + TSL2584 + pantalla (cada 1s) ────────────────────
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

    // DHT11
    TempAndHumidity th = dht.getTempAndHumidity();
    if (dht.getStatus() == DHTesp::ERROR_NONE) {
      dht_ok         = true;
      temperatureDHT = th.temperature;
      humidity       = th.humidity;
    } else {
      dht_ok         = false;
      temperatureDHT = sim_tempDHT;
      humidity       = sim_humidity;
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
    Serial.printf("[1s] T:%.1f Tb:%.1f H:%.1f P:%.2f W:%.2f D:%.0f Lux:%.1f\n",
      temperatureMCP, temperatureDHT, humidity,
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
      // CSV: 9 valores — temperatura, presion, temp_bar, humedad,
      //                   viento, direccion, viento_filtrado, dir_filtrada, luz
      String msg = String(temperatureMCP, 2)    + "," +
                   String(pressure, 2)          + "," +
                   String(temperatureDHT, 2)    + "," +
                   String(humidity, 2)          + "," +
                   String(windSpeed, 2)         + "," +
                   String(currentWindDirDeg, 2) + "," +
                   String(windSpeedFiltered, 2) + "," +
                   String(finalAvgWindDir, 2)   + "," +
                   String(lightLevel, 2);

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
