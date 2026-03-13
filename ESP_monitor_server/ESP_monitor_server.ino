// MeteoStation — Firmware v3
// LilyGo T-Display-S3 (ST7789, 320×170)
// Simulación automática por sensor con badge REAL/SIM en pantalla
// Tres temporizadores independientes: 100ms viento / 1s pantalla / 20s envío

#define ESP32

#include "WiFi.h"
#include <HTTPClient.h>
#include "secrets.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <SparkFun_MicroPressure.h>
#include <DHTesp.h>
#include <math.h>

// ── Pines ──────────────────────────────────────────────────────────────────────
#define I2C_SDA          21
#define I2C_SCL          22
#define DHTPIN           15
#define ADC_VOLTAGE_REF  3.41f
#define ADC_RANGE        4096.0f

const int anemometerPin = 37;
const int vanePin       = 36;
const int ledPin        = 2;

// ── Intervalos ─────────────────────────────────────────────────────────────────
#define WIND_MS    100
#define SCREEN_MS 1000
#define SEND_MS  20000

// ── Objetos ────────────────────────────────────────────────────────────────────
SparkFun_MicroPressure barometer;
Adafruit_MCP9808       tempsensor = Adafruit_MCP9808();
DHTesp                 dht;
TFT_eSPI               tft = TFT_eSPI();
TFT_eSprite            spr = TFT_eSprite(&tft);

// ── Flags de sensor ────────────────────────────────────────────────────────────
bool mcp_ok = false;
bool bar_ok = false;
bool dht_ok = false;

// ── Valores medidos ────────────────────────────────────────────────────────────
float  temperatureMCP    = 0;
float  temperatureDHT    = 0;
float  humidity          = 0;
double pressure          = 0;
float  windSpeed         = 0;
float  windSpeedFiltered = 0;
float  currentWindDirDeg = 0;

// ── Valores simulados (drift lento) ───────────────────────────────────────────
float sim_tempMCP  = 20.5f;
float sim_tempDHT  = 19.8f;
float sim_humidity = 62.0f;
float sim_pressure = 101.3f;

static float driftClamp(float v, float mn, float mx, float step) {
  float d = ((float)random(-100, 101) / 100.0f) * step;
  return constrain(v + d, mn, mx);
}

void updateSimulatedValues() {
  sim_tempMCP  = driftClamp(sim_tempMCP,  -10.0f, 45.0f, 0.05f);
  sim_tempDHT  = driftClamp(sim_tempDHT,  -10.0f, 45.0f, 0.05f);
  sim_humidity = driftClamp(sim_humidity,  20.0f, 95.0f, 0.20f);
  sim_pressure = driftClamp(sim_pressure,  95.0f,110.0f, 0.02f);
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

// ── Paleta de colores ─────────────────────────────────────────────────────────
// RGB565
#define C_BG      0x0841   // fondo oscuro azul marino
#define C_CARD    0x1082   // tarjeta ligeramente más claro
#define C_HDR     0x0421   // barra de cabecera muy oscura
#define C_BORDER  0x2124   // borde de tarjeta
#define C_TEXT    0xFFFF   // blanco
#define C_LABEL   0x7BEF   // gris claro
#define C_SIM     0xFD20   // naranja (simulado)
#define C_REAL    0x07E0   // verde (real)
#define C_RED     0xF800

// ── Layout (320×170) ──────────────────────────────────────────────────────────
// Cabecera: y=0..21 (22px)
// 2 filas × 3 columnas
// CARD_W=106, CARD_H=73, gap horizontal=1, gap vertical=2
//   col x: 0, 107, 214
//   row y: 22, 97
#define HDR_H    22
#define CARD_W  106
#define CARD_H   73

static int cardX(int col) { return col * (CARD_W + 1); }
static int cardY(int row) { return HDR_H + row * (CARD_H + 2); }

// ── Iconos (dibujados sobre el sprite) ───────────────────────────────────────
// Todos los iconos caben en un cuadrado de ~14×14px
// cx, cy = centro del icono

void iconThermometer(int cx, int cy, uint16_t col) {
  spr.fillRoundRect(cx - 2, cy - 7, 5, 10, 2, col);
  spr.fillCircle(cx, cy + 5, 4, col);
  // línea interior (para dar forma de termómetro)
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
  spr.drawLine(cx, cy, cx + 4, cy - 4, col);  // aguja
  spr.fillCircle(cx, cy, 2, col);
  spr.drawFastHLine(cx - 5, cy + 5, 11, col); // base
}

void iconWind(int cx, int cy, uint16_t col) {
  spr.drawFastHLine(cx - 6, cy - 3, 12, col);
  spr.drawLine(cx + 6, cy - 3, cx + 8, cy - 1, col);
  spr.drawFastHLine(cx - 6, cy,     10, col);
  spr.drawFastHLine(cx - 6, cy + 3, 12, col);
}

void iconCompass(int cx, int cy, uint16_t col) {
  spr.drawCircle(cx, cy, 7, col);
  // N arriba, S abajo en pequeño
  spr.fillTriangle(cx, cy - 6, cx - 2, cy, cx + 2, cy, col);         // norte
  spr.fillTriangle(cx, cy + 6, cx - 2, cy, cx + 2, cy, C_LABEL);     // sur
}

typedef void (*IconFn)(int, int, uint16_t);
IconFn iconFns[6] = {
  iconThermometer, // T.EXT  (0,0)
  iconThermometer, // T.INT  (1,0)
  iconDrop,        // HUMID  (2,0)
  iconGauge,       // PRES   (0,1)
  iconWind,        // VIENTO (1,1)
  iconCompass,     // DIR    (2,1)
};

// ── Dibujar tarjeta ───────────────────────────────────────────────────────────
void drawCard(int col, int row,
              const char* label,
              float value, const char* unit,
              bool simulated,
              bool showCompass = false) {

  int x = cardX(col);
  int y = cardY(row);

  // Fondo + borde
  spr.fillRoundRect(x + 1, y + 1, CARD_W - 2, CARD_H - 2, 4, C_CARD);
  spr.drawRoundRect(x, y, CARD_W, CARD_H, 4, C_BORDER);

  // Franja de color en la parte superior de la tarjeta
  uint16_t accentCol = simulated ? C_SIM : C_REAL;
  spr.fillRoundRect(x + 1, y + 1, CARD_W - 2, 3, 2, accentCol);

  // Icono
  int idx = row * 3 + col;
  iconFns[idx](x + 12, y + 17, accentCol);

  // Etiqueta
  spr.setTextColor(C_LABEL, C_CARD);
  spr.drawString(label, x + 27, y + 11, 1);

  // Valor principal (Font 4 = 26px)
  spr.setTextColor(C_TEXT, C_CARD);
  char buf[12];
  snprintf(buf, sizeof(buf), showCompass ? "%.0f" : "%.1f", value);
  spr.drawString(buf, x + 6, y + 27, 4);

  // Unidad
  spr.setTextColor(C_LABEL, C_CARD);
  spr.drawString(unit, x + 6, y + 55, 1);

  // Abreviatura de dirección brújula
  if (showCompass) {
    spr.setTextColor(accentCol, C_CARD);
    spr.drawString(degToCompass(value), x + 60, y + 55, 2);
  }

  // Badge REAL / SIM (esquina inferior derecha)
  const char* badge = simulated ? "[SIM]" : "[OK]";
  spr.setTextColor(accentCol, C_CARD);
  spr.drawRightString(badge, x + CARD_W - 4, y + CARD_H - 12, 1);
}

// ── Dibujar pantalla principal ────────────────────────────────────────────────
void drawScreen() {
  spr.fillSprite(C_BG);

  // ── Cabecera ──
  spr.fillRect(0, 0, 320, HDR_H, C_HDR);

  spr.setTextColor(C_TEXT, C_HDR);
  spr.drawString("METEOSTATION", 8, 5, 2);

  // WiFi
  if (WiFi.status() == WL_CONNECTED) {
    spr.setTextColor(C_REAL, C_HDR);
    spr.drawString("WiFi", 220, 6, 1);
  } else {
    spr.setTextColor(C_RED, C_HDR);
    spr.drawString("NoWiFi", 210, 6, 1);
  }

  // Círculo de estado del servidor
  uint16_t srvCol = lastServerOK ? C_REAL : C_RED;
  spr.fillCircle(308, 11, 6, srvCol);
  spr.drawCircle(308, 11, 6, C_TEXT);

  // ── 6 tarjetas ──
  //  Fila 0
  drawCard(0, 0, "T.EXT",   temperatureMCP,     "C",    !mcp_ok);
  drawCard(1, 0, "T.INT",   temperatureDHT,     "C",    !dht_ok);
  drawCard(2, 0, "HUMEDAD", humidity,           "%",    !dht_ok);
  // Fila 1
  drawCard(0, 1, "PRESION", (float)pressure,    "KPa",  !bar_ok);
  drawCard(1, 1, "VIENTO",  windSpeedFiltered,  "m/s",  false);
  drawCard(2, 1, "DIRECC.", currentWindDirDeg,  "deg",  false, true);

  spr.pushSprite(0, 0);
}

// ── Pantalla de arranque ───────────────────────────────────────────────────────
void drawBootScreen(const char* wifiMsg) {
  spr.fillSprite(C_BG);

  // Cabecera
  spr.fillRect(0, 0, 320, HDR_H, C_HDR);
  spr.setTextColor(C_TEXT, C_HDR);
  spr.drawCentreString("METEOSTATION  v3", 160, 5, 2);

  // Estado de cada sensor
  struct { const char* lbl; bool ok; } sensors[3] = {
    { "MCP9808  (T.Ext)", mcp_ok },
    { "Barometro       ", bar_ok },
    { "DHT11    (T.Int)", dht_ok },
  };

  for (int i = 0; i < 3; i++) {
    int y = 32 + i * 26;
    spr.setTextColor(C_LABEL, C_BG);
    spr.drawString(sensors[i].lbl, 20, y, 2);

    uint16_t badgeCol  = sensors[i].ok ? C_REAL : C_SIM;
    const char* badgeTxt = sensors[i].ok ? "  REAL  " : "  SIM  ";
    spr.fillRoundRect(220, y, 70, 16, 3, badgeCol);
    spr.setTextColor(TFT_BLACK, badgeCol);
    spr.drawCentreString(badgeTxt, 255, y + 4, 1);
  }

  spr.setTextColor(C_LABEL, C_BG);
  spr.drawCentreString(wifiMsg, 160, 126, 2);

  spr.pushSprite(0, 0);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Pantalla
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  spr.createSprite(320, 170);
  spr.setSwapBytes(true);

  // DHT — necesita tiempo antes de la primera lectura
  dht.setup(DHTPIN, DHTesp::DHT11);

  // Detectar MCP9808
  mcp_ok = tempsensor.begin(0x19);
  if (mcp_ok) {
    tempsensor.setResolution(3);
    Serial.println("MCP9808 OK");
  } else {
    Serial.println("MCP9808 no detectado — modo simulacion");
    temperatureMCP = sim_tempMCP;
  }

  // Detectar barómetro
  bar_ok = barometer.begin();
  if (bar_ok) {
    Serial.println("Barometro OK");
  } else {
    Serial.println("Barometro no detectado — modo simulacion");
    pressure = sim_pressure;
  }

  // Validar DHT11 con primera lectura
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

  // Pantalla de boot con estado de sensores
  drawBootScreen("Conectando WiFi...");

  // WiFi (máximo 20 intentos = 10s)
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi OK: " + WiFi.localIP().toString());
    drawBootScreen(("IP: " + WiFi.localIP().toString()).c_str());
  } else {
    Serial.println("Sin WiFi — continuando sin conexion");
    drawBootScreen("Sin WiFi — modo offline");
  }

  delay(2500);  // mostrar boot screen 2.5s antes de pasar al dashboard
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── 1. Lectura ADC viento (cada 100ms) ──────────────────────────────────────
  static unsigned long lastWindRead = 0;
  if (now - lastWindRead >= WIND_MS) {
    int rawAne  = analogRead(anemometerPin);
    int rawVane = analogRead(vanePin);

    float filtAne     = filteredADC(rawAne);
    windSpeed         = adcToWindSpeed((float)rawAne);
    windSpeedFiltered = adcToWindSpeed(filtAne);
    currentWindDirDeg = adcToWindDeg(rawVane);
    accumulateWindVector(currentWindDirDeg);

    lastWindRead = now;
  }

  // ── 2. Sensores I2C + DHT + pantalla (cada 1s) ──────────────────────────────
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
      dht_ok = false;
      temperatureDHT = sim_tempDHT;
      humidity       = sim_humidity;
    }

    // Avanzar valores simulados suavemente
    updateSimulatedValues();

    drawScreen();
    lastScreenTime = now;
  }

  // ── 3. Envío HTTP (cada 20s) ─────────────────────────────────────────────────
  if (now - lastSendTime >= SEND_MS) {
    finalAvgWindDir = calcAndResetWindVector();
    bool ok = false;

    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(ledPin, LOW);

      HTTPClient http;
      http.setTimeout(3000);
      String url = "http://" + String(server_ip) + ":" + String(server_port) + "/send_message";
      http.begin(url);
      http.addHeader("Content-Type", "text/plain");

      String msg = String(temperatureMCP, 2)    + "," +
                   String(pressure, 2)          + "," +
                   String(temperatureDHT, 2)    + "," +
                   String(humidity, 2)          + "," +
                   String(windSpeed, 2)         + "," +
                   String(currentWindDirDeg, 2) + "," +
                   String(windSpeedFiltered, 2) + "," +
                   String(finalAvgWindDir, 2);

      Serial.println("TX: " + msg);
      int code = http.POST(msg);
      ok = (code == 200 || code == 201);
      Serial.printf("HTTP %d\n", code);
      http.end();
      digitalWrite(ledPin, HIGH);

    } else {
      Serial.println("WiFi desconectado — intentando reconectar");
      WiFi.reconnect();
    }

    lastServerOK = ok;
    lastSendTime = now;
  }
}
