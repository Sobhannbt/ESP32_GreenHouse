#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Wi-Fi credentials
const char* wifi_ssid = "your-SSID"; // Replace with your Wi-Fi SSID
const char* wifi_password = "your-PASSWORD"; // Replace with your Wi-Fi password

// MQTT broker settings
const char* mqtt_server = "broker.hivemq.com"; // Replace with your MQTT broker
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Client_001"; // Unique ID for each of 120 devices
const char* mqtt_topic = "Device001/data"; // Unique topic for each device

// BME280 settings
Adafruit_BME280 bme; // I2C interface (SDA: GPIO21, SCL: GPIO22)

// Soil moisture sensor settings
const int soilMoisturePin = 36; // ADC1_0 (GPIO36) for capacitive soil moisture sensor
const int dryValue = 4095; // ADC value for dry soil (adjust based on calibration)
const int wetValue = 1500;  // ADC value for wet soil (adjust based on calibration)

// OLED settings (1-inch, 128x64, SSD1306)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin not used
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button settings
#define BUTTON_PIN GPIO_NUM_0 // GPIO0 for button (supports wake-up from deep sleep)
#define DISPLAY_ADDRESS 0x3C

// Deep sleep settings
#define us_TO_s_FACTOR 1000000  // Conversion factor for microseconds to seconds
#define SLEEP_TIME_HOUR 3600    // Sleep for 1 hour (in seconds)
#define DISPLAY_TIME 60         // Display data for 1 minute (in seconds)

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

////////////////////// Private Function //////////////////////
void connectWiFi();
void connectMQTT();
void displaySensorData();
//////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize BME280
  if (!bme.begin(0x76)) { // Default I2C address for BME280 (0x76 or 0x77)
    Serial.println("Could not find BME280 sensor!");
    while (1);
  }

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS)) { // I2C address 0x3C for SSD1306
    Serial.println("Could not find OLED display!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Check wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {     // Button pressed: Display sensor data for 1 minute   
    displaySensorData();
    delay(DISPLAY_TIME * 1000); // Show data for 1 minute
    display.clearDisplay();
    display.display(); // Update to clear screen
    display.ssd1306_command(SSD1306_DISPLAYOFF); // Power off OLED
  } else {
    // Timer wake-up: Send data via MQTT
    connectWiFi();
    client.setServer(mqtt_server, mqtt_port);
    connectMQTT();
    
    // Read sensor data
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    int soilRaw = analogRead(soilMoisturePin);
    int soilMoisturePercent = map(soilRaw, dryValue, wetValue, 0, 100);
    soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);

    // Create JSON payload
    StaticJsonDocument<200> doc;
    
    doc["device"] = mqtt_client_id;
    doc["temperature"] = temperature;
    doc["soil_moisture"] = soilMoisturePercent;
    doc["humidity"] = humidity;
    
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    // Publish data to MQTT
    if (client.publish(mqtt_topic, jsonBuffer)) {
      Serial.println("Data published successfully");
    } else {
      Serial.println("Failed to publish data");
    }

    // Disconnect MQTT and Wi-Fi
    client.disconnect();
    WiFi.disconnect();
  }

  // Configure button wake-up (falling edge, when button is pressed to GND)
  esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0); // 0 = LOW (button press)
  
  // Configure timer wake-up for hourly MQTT
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_HOUR * us_TO_s_FACTOR);

  // Enter deep sleep
  Serial.println("Entering deep sleep...");
  esp_deep_sleep_start();
}

void loop() {
  // Empty, as deep sleep is used
}

void connectWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(wifi_ssid, wifi_password);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 10) {
    delay(1000);
    Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
  } else {
    Serial.println("Wi-Fi connection failed!");
    esp_deep_sleep_start();
  }
}

void connectMQTT() {
  int retries = 0;
  while (!client.connected() && retries < 5) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(1000);
      retries++;
    }
  }
  if (!client.connected()) {
    Serial.println("MQTT connection failed!");
    esp_deep_sleep_start();
  }
}

void displaySensorData() {
  // Read sensor data
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  int soilRaw = analogRead(soilMoisturePin);
  int soilMoisturePercent = map(soilRaw, dryValue, wetValue, 0, 100);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);

  // Display data on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature);
  display.println(" C");
  display.print("Humidity: ");
  display.print(humidity);
  display.println(" %");
  display.print("Soil Moisture: ");
  display.print(soilMoisturePercent);
  display.println(" %");
  display.display();
}