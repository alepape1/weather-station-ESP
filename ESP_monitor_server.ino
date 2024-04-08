#define ESP32


#if defined(ESP8266)

  #define HARDWARE "ESP8266"
  #include "ESP8266WiFi.h"
  #include <ESP8266HTTPClient.h>
  #define I2C_SDA D2
  #define I2C_SCL D1
  #define DHTPIN 15
  #define ADC_VOLTAGE_REFERENCE 5
  #define ADC_RANGE 4095.0

#elif defined(ESP32)

  // #define HARDWARE "ESP32"
  #define I2C_SDA 21
  #define I2C_SCL 22
  #include "WiFi.h"
  #include <HTTPClient.h>
  #define DHTPIN 15
  #include <TFT_eSPI.h>
  #include <SPI.h>
  #define ADC_VOLTAGE_REFERENCE 3.41
  #define ADC_RANGE 4096.0


#define TFT_GREY 0x5AEB  // New colour
#define SEND_DATA_TIME 20000


#endif

#define FILTER_SIZE 10  // Adjust this value for desired smoothing effect

#include <math.h>
#include <SparkFun_MicroPressure.h>
#include <DHTesp.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>


SparkFun_MicroPressure barometer;  // Use default values with reset and EOC pins unusedDHTesp dht;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
WiFiClient client;
DHTesp dht;

const char* ssid = "Promicon_Fibra-ea7i";
const char* password = "d6dlda8k";
const char* server_ip = "192.168.1.32";  // Dirección IP del servidor Flask
const int server_port = 5000;            // Puerto del servidor Flask

const int ledPin = 2;  // Pin GPIO del LED

float windSpeed = 0;
double pressure = 0;
float temperature = 0;
float humidity = 0;
double temperatureDHT = 0;
float heatIndex = 0;

//Filter values and variables for anemometer
int anemometerValues[FILTER_SIZE];  // Array to store past readings
int currentIndexAnemometer = 0;
float filteredAnemometerValue = 0.0;

//Filter values and variables for anemometer
int vaneValues[FILTER_SIZE];  // Array to store past readings
int currentIndexVane = 0;
float filteredVaneValue = 0.0;

float adcVoltageReference = ADC_VOLTAGE_REFERENCE;
int anemometerPin = 37;
int vanePin = 36;  // select the input pin for the potentiometer
// int ledPin = 13;      // select the pin for the LED
int vaneValue = 0;  // variable to store the value coming from the sensor
int anemometerValue = 0; //variable to store the value coming from the sensor
float degrees = 0;
float adcRange = ADC_RANGE;


unsigned long lastTime = 0;
unsigned long interval = SEND_DATA_TIME; // Intervalo en milisegundos 



TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h


// Función para convertir el voltaje del anemómetro a velocidad del viento
float getWindSpeed(int adcValue) {

  float voltage = adcValue* (adcVoltageReference / adcRange);
  // Calibración del sensor (ajustar según la hoja de datos del anemómetro)
  return ((voltage / adcVoltageReference) * 30.0);
}

// Función para convertir el valor ADC de la veleta a grados
float getWindDirectionDegrees(int adcValue) {
  // Rango de valores ADC
  
  // Convertir ADC a grados
  float degrees = (adcValue / adcRange) * 360.0;
  // Norte = 0 grados
  if (degrees >= 337.5 || degrees < 22.5) {
    return 0.0;
  } else if (degrees >= 22.5 && degrees < 67.5) {
    return 45.0;
  } else if (degrees >= 67.5 && degrees < 112.5) {
    return 90.0;
  } else if (degrees >= 112.5 && degrees < 157.5) {
    return 135.0;
  } else if (degrees >= 157.5 && degrees < 202.5) {
    return 180.0;
  } else if (degrees >= 202.5 && degrees < 247.5) {
    return 225.0;
  } else if (degrees >= 247.5 && degrees < 292.5) {
    return 270.0;
  } else if (degrees >= 292.5 && degrees < 337.5) {
    return 315.0;
  } else{
    return 0.0;
  }
}

// Función para convertir grados a dirección del viento
String getWindDirectionString(float degrees) {

  int directionCase = round(degrees / 45.0);  // Round degrees to nearest integer

  switch (directionCase) {
    case 0:
      return "Norte";
    case 1:
      return "Noreste";
    case 2:
      return "Este";
    case 3:
      return "Sureste";
    case 4:
      return "Sur";
    case 5:
      return "Suroeste";
    case 6:
      return "Oeste";
    case 7:
      return "Noroeste";
    default:
      return "Desconocido";
  }
}

// Function to calculate the moving average of anemometer readings
float getFilteredAnemometerValue(int newAnemometerValue) {
  // Update filter buffer
  anemometerValues[currentIndexAnemometer] = newAnemometerValue;
  currentIndexAnemometer = (currentIndexAnemometer + 1) % FILTER_SIZE;

  // Calculate filtered value
  float filteredAnemometerValue = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    filteredAnemometerValue += anemometerValues[i];
  }
  filteredAnemometerValue /= FILTER_SIZE;

  return filteredAnemometerValue;
}

// Function to calculate the moving average of vane readings
float getFilteredVaneValue(int newVaneValue) {
  // Update filter buffer
  vaneValues[currentIndexVane] = newVaneValue;
  currentIndexVane = (currentIndexVane + 1) % FILTER_SIZE;

  // Calculate filtered value
  float filteredVaneValue = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    filteredVaneValue += vaneValues[i];
  }
  filteredVaneValue /= FILTER_SIZE;

  return filteredVaneValue;
}

void setup() {

  tft.init();
  tft.setRotation(1);

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  // Inicializar DHT
  dht.setup(DHTPIN, DHTesp::DHT11);

  Serial.println("\nInitializing...");

  // Fill screen with grey so we can see the effect of printing with and without
  // a background colour defined
  tft.fillScreen(TFT_GREY);
  tft.println("Pape Wheater station");
  delay(2000);

  if (!tempsensor.begin(0x19)) {
    Serial.println("Failed to find Adafruit MCP9808 chip");
    tft.println("MCP9808 Failed!");
    while (1) { delay(10); }
  } else {
    Serial.println("MCP9808 Found!");
    tft.println("MCP9808 Found!");
    delay(2000);
    tempsensor.setResolution(3);
    Serial.print("Resolution temperature sensor:");
    Serial.println(tempsensor.getResolution());
    
  }

  delay(1000);

  if (!barometer.begin()) {  // barometer.connect starts wire and attempts to connect to sensor
    Serial.println(F("Error connecting barometer sensor..."));
    delay(500);
  }


  Serial.println(F("Connected to Sensors"));
  tft.println("Sensor connected");
  delay(5);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  tft.print("Connecting to WiFi: ");
  tft.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.print(".");
  }

  Serial.println("\n");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(ledPin, OUTPUT);  // Configurar el pin del LED como salida
  
}

void loop() {

  unsigned long currentTime = millis();


  if (client.connect(server_ip, server_port)) {

    

    Serial.println("Connected to server");
    digitalWrite(ledPin, LOW);  // Encender el LED

    HTTPClient http;
    http.begin(client, server_ip, server_port, "/send_message");  // Especifica la ruta
    http.addHeader("Content-Type", "text/plain");

    // Leer datos del sensor DHT11
    temperatureDHT = dht.getTemperature();
    humidity = dht.getHumidity();
    heatIndex = dht.computeHeatIndex(temperatureDHT, humidity);

    // read the value from the sensor:
    vaneValue = analogRead(vanePin);

    anemometerValue = analogRead(anemometerPin);
  
    windSpeed = getWindSpeed(anemometerValue);
    delay(500);
    degrees = getWindDirectionDegrees(vaneValue);
    // Get filtered anemometer value
    float filteredAnemometerValue = getFilteredAnemometerValue(anemometerValue);
    float windSpeedFiltered = getWindSpeed(filteredAnemometerValue);
    // barometer.checkUpdates();
    float filteredVaneValue = getFilteredVaneValue(vaneValue);
    float vaneValueFiltered = getWindDirectionDegrees(filteredVaneValue);

    tempsensor.wake();  // wake up, ready to read!
    delay(5);
    temperature = tempsensor.readTempC();    // Returns temperature in C
    pressure = barometer.readPressure(KPA);  // Returns pressure in Pascals
    tempsensor.shutdown_wake(1);
    
    Serial.print(F("Temperature B.: "));
    Serial.print(temperatureDHT);
    Serial.print(",");
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(F("Heat index: "));
    Serial.print(heatIndex);
    Serial.print(",");
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(F("Pressure: "));
    Serial.println(pressure);

    Serial.print("ADC:");
    Serial.print(vaneValue);
    Serial.print("\t");
    Serial.print("Grados:");
    Serial.print(degrees);
    Serial.print("\t");
    Serial.print(vaneValueFiltered);


    Serial.print("\t");
    Serial.print(getWindDirectionString(degrees));
    Serial.print("\t");

    Serial.print("ADC:");
    Serial.print(anemometerValue);
    Serial.print("\t");
  

    Serial.print(windSpeed);
    Serial.print("m/sg");
    Serial.print("\t");


    Serial.print(windSpeedFiltered);   
    Serial.println("m/sg");

    delay(500);
    // Concatena los valores en un string

    if (currentTime - lastTime >= interval) {

      String mensaje = String(temperature) + "," + String(pressure) + "," + String(temperatureDHT) + "," + String(humidity) + "," + String(windSpeed) + ","+ String(degrees) + "," + String(windSpeedFiltered) + "," + String(vaneValueFiltered);
      int httpCode = http.POST(mensaje);

      if (httpCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpCode);
      } else {
        Serial.println("Error al enviar mensaje!");
      }

      http.end();
      client.stop();
      digitalWrite(ledPin, HIGH);  // Apagar el LED
      lastTime = currentTime;

    }

  } else {
    Serial.println("Connection to server failed");
  }


  // // Fill screen with grey so we can see the effect of printing with and without
  // // a background colour defined
  tft.fillScreen(TFT_WHITE);

  // // Set "cursor" at top left corner of display (0,0) and select font 2
  // // (cursor will move to next line automatically during printing with 'tft.println'
  // //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(0, 0, 2);
  // // Set the font colour to be white with a black background, set text size multiplier to 1
  // tft.setTextColor(TFT_BLACK);
  // tft.setTextSize(1);
  // tft.setTextFont(3);

  // // We can now plot text on screen using the "print" class
  // tft.println("Smart Grow Data Monitor");

  // // Set the font colour to be red with black background, set to font 4

  // tft.setTextFont(1);
  // tft.println(humidity);


  // Set the font colour to be blue with no background, set to font 2
  // Set the font colour to be yellow with no background, set to font 7
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextFont(2);
  tft.print("Temp = ");
  tft.println(temperature);

  tft.setTextColor(TFT_RED);
  tft.setTextFont(2);
  tft.print("Temp 2 = ");
  tft.println(temperatureDHT);  // Should print DEADBEEF

  tft.setTextColor(TFT_BLUE);
  tft.setTextFont(2);
  tft.print("Humidity = ");
  tft.println(humidity);  // Print floating point number

  // Set the font colour to be green with black background, set to font 2
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.print("Preassure = ");
  tft.println(pressure);  // Print as integer value in binary

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.print("Preassure = ");
  tft.println(pressure);  // Print as integer value in binary

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.print("Vane ADC = ");
  tft.println(vaneValue);  // Print as integer value in binary

  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextFont(2);
  tft.print("Anemometer ADC = ");
  tft.println(anemometerValue);  // Should print DEADBEEF

  // while(1) yield(); // We must yield() to stop a watchdog timeout.
}
