#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>

// SENSOR
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ACTUATORS
#define FAN_PIN 5
#define HEATER_PIN 18
#define HUMIDIFIER_PIN 19

// STATUS LEDs (Green = good, Red = problem)
#define GREEN_LED 13
#define RED_LED   12

// WIFI
#define WIFI_SSID "AASTU staff"
#define WIFI_PASSWORD "st@77#AASTU2021"

// FIREBASE
#define DATABASE_URL "https://cockroach-farming-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "rr2RvuHjsaBvTRi5ZdbaIeI0DzAPcll29rI7n0J8"

// Variables for thresholds (can be changed from Firebase)
float tempHigh = 30.0;
float tempLow = 20.0;
float humHigh = 70.0;
float humLow = 60.0;

// Hysteresis to prevent rapid on/off
const float TEMP_HYST = 1.0;

// Control states
bool autoMode = true;
bool fanState = false;
bool heaterState = false;
bool humidifierState = false;
bool manualFan = false;
bool manualHeater = false;
bool manualHumidifier = false;

// Sensor readings
float temperature = 0;
float humidity = 0;

// PWM for fan (channel 0)
const int FAN_PWM_CHANNEL = 0;
const int PWM_FREQ = 25000;    // 25kHz - silent, standard for PC fans
const int PWM_RES = 8;

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 10000;  // Update every 10 seconds

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.println(WiFi.localIP());
}

void setupFirebase() {
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void setup() {
  Serial.begin(115200);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // digitalWrite(FAN_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  
  // Setup PWM for fan speed (0-255)
  ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RES);  // Channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);

  dht.begin();
  connectWiFi();
  setupFirebase();

  Serial.println("System Ready");
}

void readSensor() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT read error!");
    ledcWrite(FAN_PWM_CHANNEL, 0); // Turn off fan on error

    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    return;
  }
}

void fetchSettingsFromFirebase() {
  if (Firebase.ready()) {
    if (Firebase.RTDB.getBool(&fbdo, "/settings/autoMode"))
      autoMode = fbdo.boolData();

    if (Firebase.RTDB.getFloat(&fbdo, "/settings/tempHigh"))
      tempHigh = fbdo.floatData();

    if (Firebase.RTDB.getFloat(&fbdo, "/settings/tempLow"))
      tempLow = fbdo.floatData();

    if (Firebase.RTDB.getFloat(&fbdo, "/settings/humHigh"))
      humHigh = fbdo.floatData();

    if (Firebase.RTDB.getBool(&fbdo, "/manual/fan"))
      manualFan = fbdo.boolData();

    if (Firebase.RTDB.getBool(&fbdo, "/manual/heater"))
      manualHeater = fbdo.boolData();
    if (Firebase.RTDB.getBool(&fbdo, "/manual/humidifier"))
      manualHumidifier = fbdo.boolData();
  }
}

void automaticControl(int &fanSpeed) {
  // Fan ON if too hot or too humid
  if (temperature > tempHigh || humidity > humHigh) {
    float maxExcess = max(temperature - tempHigh, humidity - humHigh);
    fanSpeed = map(constrain(maxExcess, 0, 20), 0, 20, 128, 255); // Scale speed
    fanState = true;
  } else if (temperature < tempHigh - TEMP_HYST && humidity < humHigh) {
    fanSpeed = 0;
    fanState = false;
  }

  // Heater ON if too cold
  if (temperature < tempLow) {
    heaterState = true;
  } else if (temperature > tempLow + TEMP_HYST) {
    heaterState = false;
  }
  // Humidifier for low humidity
  if (humidity < humLow) humidifierState = true;
  else if (humidity > humLow + 5) humidifierState = false;
}

void applyControl() {
  int fanSpeed = 0;
  if (autoMode) {
    automaticControl(fanSpeed);
  } else {
    fanState = manualFan;
    fanSpeed = manualFan ? 100 : 0;
    heaterState = manualHeater;
    humidifierState = manualHumidifier;
  }

  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);

  // digitalWrite(FAN_PIN, fanState);
  digitalWrite(HEATER_PIN, heaterState);
  digitalWrite(HUMIDIFIER_PIN, humidifierState);
}

void sendDataToFirebase() {
  if (!Firebase.ready()) return;

  long ts = millis();

  // Sensor data
  Firebase.RTDB.setFloat(&fbdo, "/sensor/temperature", temperature);
  Firebase.RTDB.setFloat(&fbdo, "/sensor/humidity", humidity);
  Firebase.RTDB.setInt(&fbdo, "/sensor/lastUpdated", ts);

  // Chamber data
  String chamberStatus = "normal";
  if (heaterState) chamberStatus = "critical";
  else if (fanState) chamberStatus = "warning";

  Firebase.RTDB.setString(&fbdo, "/chambers/chamber_1/status", chamberStatus);
  Firebase.RTDB.setFloat(&fbdo, "/chambers/chamber_1/temperature", temperature);
  Firebase.RTDB.setFloat(&fbdo, "/chambers/chamber_1/humidity", humidity);
  Firebase.RTDB.setInt(&fbdo, "/chambers/chamber_1/lastUpdated", ts);

  // Control outputs
  Firebase.RTDB.setBool(&fbdo, "/control/fan", fanState);
  Firebase.RTDB.setBool(&fbdo, "/control/heater", heaterState);
  Firebase.RTDB.setInt(&fbdo, "/control/lastUpdated", ts);

  // Status
  Firebase.RTDB.setBool(&fbdo, "/status/fan", fanState);
  Firebase.RTDB.setBool(&fbdo, "/status/heater", heaterState);
  Firebase.RTDB.setString(&fbdo, "/status/system", "operational");
  Firebase.RTDB.setInt(&fbdo, "/status/lastUpdated", ts);

  // System info
  Firebase.RTDB.setInt(&fbdo, "/system/rssi", WiFi.RSSI());
  Firebase.RTDB.setInt(&fbdo, "/system/uptime", millis() / 1000);

  // LED status
    bool inRange = (temperature >= tempLow && temperature <= tempHigh && 
                    humidity >= humLow && humidity <= humHigh);
    digitalWrite(GREEN_LED, inRange ? HIGH : LOW);
    digitalWrite(RED_LED, inRange ? LOW : HIGH);
}

void loop() {
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    readSensor();

    if (Firebase.ready()) {
      fetchSettingsFromFirebase();
      applyControl();
      sendDataToFirebase();
    }

    Serial.printf("Temp: %.2f°C | Hum: %.2f%% | Fan: %d | Heater: %d | Humidifier: %d | Mode: %s\n",
                  temperature, humidity, fanState, heaterState, humidifierState, autoMode ? "AUTO" : "MANUAL");
  }
}