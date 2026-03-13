#define ESP32

#include "WiFi.h"
#include <HTTPClient.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <SparkFun_MicroPressure.h>
#include <DHTesp.h>
#include <math.h> // Necesario para sin, cos, atan2

#define TFT_GREY 0x5AEB

// --- CONFIGURACIÓN DE PINES ---
#define I2C_SDA 21
#define I2C_SCL 22
#define DHTPIN 15
#define ADC_VOLTAGE_REFERENCE 3.41
#define ADC_RANGE 4096.0

const int anemometerPin = 37;
const int vanePin = 36;
const int ledPin = 2;

// --- CONFIGURACIÓN DE RED ---
const char* ssid = "Promicon_Fibra-ea7i";
const char* password = "d6dlda8k";
const char* server_ip = "192.168.1.42";
const int server_port = 5000;

// --- OBJETOS ---
SparkFun_MicroPressure barometer;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
DHTesp dht;
TFT_eSPI tft = TFT_eSPI();

// --- VARIABLES GLOBALES ---
#define FILTER_SIZE 10
#define SEND_DATA_INTERVAL 20000
#define SCREEN_UPDATE_INTERVAL 1000

unsigned long lastSendTime = 0;
unsigned long lastScreenTime = 0;
bool lastServerStatus = false;

// Variables de sensores instantáneas
float windSpeed = 0;
float windSpeedFiltered = 0;
double pressure = 0;
float temperatureMCP = 0;
float temperatureDHT = 0;
float humidity = 0;

// Variables para la dirección del viento
float currentWindDirDegrees = 0; // Para mostrar en pantalla (instantáneo)
String currentWindDirString = "";

// --- VARIABLES PARA EL PROMEDIO VECTORIAL (VELETA) ---
float windSumX = 0;
float windSumY = 0;
int windSampleCount = 0;
float finalAverageWindDir = 0; // Este es el valor PROMEDIO que enviaremos al server

// Filtros (Solo mantenemos el del anemómetro)
int anemometerValues[FILTER_SIZE];
int currentIndexAnemometer = 0;


// --- FUNCIONES AUXILIARES ---

float getWindSpeed(int adcValue) {
  float voltage = adcValue * (ADC_VOLTAGE_REFERENCE / ADC_RANGE);
  return ((voltage / ADC_VOLTAGE_REFERENCE) * 30.0);
}

// Convierte ADC a Grados (0, 45, 90...)
float getWindDirectionDegrees(int adcValue) {
  float degrees = (adcValue / ADC_RANGE) * 360.0;
  if (degrees >= 337.5 || degrees < 22.5) return 0.0;
  if (degrees < 67.5) return 45.0;
  if (degrees < 112.5) return 90.0;
  if (degrees < 157.5) return 135.0;
  if (degrees < 202.5) return 180.0;
  if (degrees < 247.5) return 225.0;
  if (degrees < 292.5) return 270.0;
  if (degrees < 337.5) return 315.0;
  return 0.0;
}

String getWindDirectionString(float degrees) {
  if (degrees >= 360) degrees = 0;
  int directionCase = round(degrees / 45.0);
  switch (directionCase) {
    case 0: return "N";
    case 1: return "NE";
    case 2: return "E";
    case 3: return "SE";
    case 4: return "S";
    case 5: return "SO";
    case 6: return "O";
    case 7: return "NO";
    case 8: return "N";
    default: return "N";
  }
}

// Filtro media móvil (solo para velocidad)
float getFilteredValue(int newValue, int* storage, int &index) {
  storage[index] = newValue;
  index = (index + 1) % FILTER_SIZE;
  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += storage[i];
  }
  return (float)sum / FILTER_SIZE;
}

// Acumula vectores de viento para el promedio vectorial (llamar cada 100ms)
void accumulateWindVector(float degrees) {
    float rad = degrees * PI / 180.0;
    windSumX += cos(rad);
    windSumY += sin(rad);
    windSampleCount++;
}

// Calcula el promedio vectorial y resetea los acumuladores (llamar antes de enviar)
float calculateAndResetWindVector() {
    if (windSampleCount == 0) return 0;

    float avgRad = atan2(windSumY, windSumX);
    float avgDeg = avgRad * 180.0 / PI;

    if (avgDeg < 0) avgDeg += 360.0;

    windSumX = 0;
    windSumY = 0;
    windSampleCount = 0;

    return avgDeg;
}

// --- DIBUJO TFT ---
void drawServerIcon(bool connected) {
  int x = 228; int y = 10; int r = 5;
  uint16_t color = connected ? TFT_GREEN : TFT_RED;
  tft.fillCircle(x, y, r, color);
  tft.drawCircle(x, y, r, TFT_WHITE);
}

void setupTFT() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString("METEOSTATION", 110, 2, 2);
  tft.drawLine(0, 20, 240, 20, TFT_GREY);

  tft.setTextColor(TFT_SILVER, TFT_BLACK);
  tft.drawString("T.Int:", 5, 25, 2);
  tft.drawString("Hum:",   5, 45, 2);
  tft.drawString("Pres:",  5, 65, 2);
  tft.drawString("Viento:",5, 85, 2);
  tft.drawString("T.Ext:", 5, 105, 2);
  drawServerIcon(false);
}

void updateScreen() {
  int x_pos_val = 80;
  tft.setTextPadding(60);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawFloat(temperatureDHT, 1, x_pos_val, 25, 2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawFloat(humidity, 1, x_pos_val, 45, 2);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawFloat(pressure, 1, x_pos_val, 65, 2);

  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  String windStr = String(windSpeedFiltered, 1) + "ms " + currentWindDirString;
  tft.drawString(windStr, x_pos_val, 85, 2);

  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawFloat(temperatureMCP, 2, x_pos_val, 105, 2);
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(ledPin, OUTPUT);

  setupTFT();

  dht.setup(DHTPIN, DHTesp::DHT11);

  if (!tempsensor.begin(0x19)) {
    Serial.println("MCP9808 Error");
    tft.drawString("Err MCP", 10, 115, 2);
  } else {
    tempsensor.setResolution(3);
  }

  if (!barometer.begin()) {
    Serial.println("Barometro Error");
    tft.drawString("Err Bar", 60, 115, 2);
  }

  tft.drawString("Conectando WiFi...", 5, 115, 2);
  WiFi.begin(ssid, password);

  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED && tryCount < 20) {
    delay(500);
    Serial.print(".");
    tryCount++;
  }

  tft.fillRect(0, 115, 240, 20, TFT_BLACK);

  if (WiFi.status() == WL_CONNECTED) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("IP:" + WiFi.localIP().toString(), 5, 115, 2);
      delay(2000);
      tft.fillRect(0, 115, 240, 20, TFT_BLACK);
  } else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("No WiFi", 5, 115, 2);
  }
}

// --- LOOP ---
void loop() {
  unsigned long currentTime = millis();

  // 1. LECTURA DE SENSORES Y ACUMULACIÓN (cada 100ms)
  static unsigned long lastSensorRead = 0;
  if (currentTime - lastSensorRead > 100) {
    int rawAnemometer = analogRead(anemometerPin);
    int rawVane = analogRead(vanePin);

    float filteredAnemometer = getFilteredValue(rawAnemometer, anemometerValues, currentIndexAnemometer);
    windSpeed = getWindSpeed(rawAnemometer);
    windSpeedFiltered = getWindSpeed(filteredAnemometer);

    currentWindDirDegrees = getWindDirectionDegrees(rawVane);
    currentWindDirString = getWindDirectionString(currentWindDirDegrees);

    accumulateWindVector(currentWindDirDegrees);

    lastSensorRead = currentTime;
  }

  // 2. ACTUALIZAR PANTALLA (cada 1s)
  if (currentTime - lastScreenTime >= SCREEN_UPDATE_INTERVAL) {
    temperatureDHT = dht.getTemperature();
    humidity = dht.getHumidity();

    tempsensor.wake();
    temperatureMCP = tempsensor.readTempC();
    tempsensor.shutdown_wake(1);

    pressure = barometer.readPressure(KPA);

    updateScreen();
    lastScreenTime = currentTime;
  }

  // 3. ENVIAR AL SERVIDOR (cada 20s)
  if (currentTime - lastSendTime >= SEND_DATA_INTERVAL) {

    finalAverageWindDir = calculateAndResetWindVector();

    bool envioExitoso = false;

    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(ledPin, LOW);

      HTTPClient http;
      http.setTimeout(3000);

      String url = "http://" + String(server_ip) + ":" + String(server_port) + "/send_message";
      http.begin(url);
      http.addHeader("Content-Type", "text/plain");

      String mensaje = String(temperatureMCP) + "," +
                       String(pressure) + "," +
                       String(temperatureDHT) + "," +
                       String(humidity) + "," +
                       String(windSpeed) + "," +
                       String(currentWindDirDegrees) + "," +
                       String(windSpeedFiltered) + "," +
                       String(finalAverageWindDir);

      Serial.println("Enviando: " + mensaje);

      int httpCode = http.POST(mensaje);

      if (httpCode == 200 || httpCode == 201) {
        Serial.printf("OK Code: %d\n", httpCode);
        envioExitoso = true;
      } else {
        Serial.printf("Error Code: %d\n", httpCode);
        envioExitoso = false;
      }

      http.end();
      digitalWrite(ledPin, HIGH);

    } else {
      Serial.println("WiFi Desconectado - reconectando...");
      envioExitoso = false;
      WiFi.reconnect();
    }

    drawServerIcon(envioExitoso);
    lastSendTime = currentTime;
  }
}
