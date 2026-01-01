#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>

// SENSOR
#define DHTPIN 4
#define DHTTYPE DHT11 // Changed to DHT11
DHT dht(DHTPIN, DHTTYPE);

// ACTUATORS
#define FAN_PIN 5
#define HEATER_PIN 18
#define HUMIDIFIER_PIN 19

// STATUS LEDs (Green = good, Red = problem)
#define GREEN_LED 13
#define RED_LED 12

// WIFI
#define WIFI_SSID "Balerion"
#define WIFI_PASSWORD "00000000"

// FIREBASE - FILL THESE IN!
#define DATABASE_URL "https://cockroach-farming-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "rr2RvuHjsaBvTRi5ZdbaIeI0DzAPcll29rI7n0J8"

// Variables for thresholds (can be changed from Firebase)
float tempHigh = 30.0;
float tempLow = 20.0;
float humHigh = 70.0;
float humLow = 60.0;

// Hysteresis
const float TEMP_HYST = 1.0;
const float HUM_HYST = 5.0; // Added for humidifier consistency

// Control states
bool autoMode = true;
bool fanState = false;
bool heaterState = false;
bool humidifierState = false;
bool manualFan = false;
bool manualHeater = false;
bool manualHumidifier = false;

// Sensor readings (initialized to invalid)
float temperature = NAN;
float humidity = NAN;

// PWM for fan
const int FAN_PWM_CHANNEL = 0;
const int PWM_FREQ = 25000;
const int PWM_RES = 8;

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 30000; // 30 seconds - safer for DHT11

// Alert System
bool alertActive = false;
String alertMessage = "";
unsigned long alertStartTime = 0;
const unsigned long ALERT_DELAY = 60000;
void connectWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.println(WiFi.localIP());
}

void setupFirebase()
{
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void setup()
{
  Serial.begin(115200);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  Serial.println("Testing heater relay...");
  digitalWrite(HEATER_PIN, LOW);
  delay(3000);
  digitalWrite(HEATER_PIN, HIGH);
  delay(3000);
  digitalWrite(HEATER_PIN, LOW);
  // digitalWrite(HEATER_PIN, heaterState);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  // PWM setup for fan
  ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);

  dht.begin();

  connectWiFi();
  setupFirebase();

  Serial.println("System Ready - Heater Auto Control + Alerts + Logging Active");
}

bool readSensor()
{
  // Retry up to 3 times for reliable reading
  for (int i = 0; i < 3; i++)
  {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (!isnan(humidity) && !isnan(temperature))
    {
      return true; // Success
    }
    delay(500); // Short delay before retry
  }

  Serial.println("DHT11 read error after retries!");
  temperature = NAN;
  humidity = NAN;
  return false;
}

void fetchSettingsFromFirebase()
{
  if (!Firebase.ready())
    return;

  if (Firebase.RTDB.getBool(&fbdo, "/settings/autoMode"))
    autoMode = fbdo.boolData();
  if (Firebase.RTDB.getFloat(&fbdo, "/settings/tempHigh"))
    tempHigh = fbdo.floatData();
  if (Firebase.RTDB.getFloat(&fbdo, "/settings/tempLow"))
    tempLow = fbdo.floatData();
  if (Firebase.RTDB.getFloat(&fbdo, "/settings/humHigh"))
    humHigh = fbdo.floatData();
  if (Firebase.RTDB.getFloat(&fbdo, "/settings/humLow"))
    humLow = fbdo.floatData(); // Add this if you have humLow in Firebase

  if (Firebase.RTDB.getBool(&fbdo, "/manual/fan"))
    manualFan = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, "/manual/heater"))
    manualHeater = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, "/manual/humidifier"))
    manualHumidifier = fbdo.boolData();
}

void automaticControl(int &fanSpeed)
{
  fanSpeed = 0;
  fanState = false;

  // Fan for high temp or high humidity
  if (temperature > tempHigh || humidity > humHigh)
  {
    float maxExcess = max(temperature - tempHigh, humidity - humHigh);
    fanSpeed = map(constrain(maxExcess, 0, 20), 0, 15, 20, 25);
    fanState = true;
  }
  else if (temperature < tempHigh - TEMP_HYST && humidity < humHigh - HUM_HYST)
  {
    fanSpeed = 0;
    fanState = false;
  }

  // Heater for low temp
  if (temperature < tempLow)
  {
    heaterState = true;
  }
  else if (temperature > tempLow + TEMP_HYST)
  {
    heaterState = false;
  }

  // Humidifier for low humidity
  if (humidity < humLow)
  {
    humidifierState = true;
  }
  else if (humidity > humLow + HUM_HYST)
  {
    humidifierState = false;
  }
}

void checkAndTriggerAlert()
{
  bool outOfRange = (temperature < tempLow || temperature > tempHigh ||
                     humidity < humLow || humidity > humHigh);

  if (outOfRange && !alertActive)
  {
    if (alertStartTime == 0)
    {
      alertStartTime = millis();
    }
    else if (millis() - alertStartTime >= ALERT_DELAY)
    {
      alertActive = true;
      if (temperature < tempLow)
        alertMessage = "Too cold! Heater needed";
      else if (temperature > tempHigh)
        alertMessage = "Too hot! Cooling needed";
      else if (humidity < humLow)
        alertMessage = "Too dry!";
      else
        alertMessage = "Too humid!";

      Serial.println("*** ALERT *** " + alertMessage);

      if (Firebase.ready())
      {
        Firebase.RTDB.setBool(&fbdo, "/alert/active", true);
        Firebase.RTDB.setString(&fbdo, "/alert/message", alertMessage);
        Firebase.RTDB.setInt(&fbdo, "/alert/timestamp", millis());
      }
    }
  }
  else if (!outOfRange && alertActive)
  {
    // Clear alert
    alertActive = false;
    alertMessage = "";
    alertStartTime = 0;
    Serial.println("Alert cleared - conditions normal");

    if (Firebase.ready())
    {
      Firebase.RTDB.setBool(&fbdo, "/alert/active", false);
      Firebase.RTDB.setString(&fbdo, "/alert/message", "normal");
    }
  }
}

void applyControl()
{
  int fanSpeed = 0;

  if (autoMode)
  {
    automaticControl(fanSpeed);
  }
  else
  {
    fanState = manualFan;
    fanSpeed = manualFan ? 50 : 0; // Full speed in manual for simplicity
    heaterState = manualHeater;
    humidifierState = manualHumidifier;
  }

  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
  digitalWrite(HEATER_PIN, HIGH);
  digitalWrite(HUMIDIFIER_PIN, humidifierState);
}

void updateLEDs(bool sensorOk)
{
  bool inRange = sensorOk && (temperature >= tempLow && temperature <= tempHigh &&
                              humidity >= humLow && humidity <= humHigh);

  if (alertActive)
  {
    // Blink red LED during alert
    digitalWrite(RED_LED, (millis() / 200) % 2 ? HIGH : LOW);
    digitalWrite(GREEN_LED, LOW);
  }
  else
  {
    digitalWrite(GREEN_LED, inRange ? HIGH : LOW);
    digitalWrite(RED_LED, inRange ? LOW : HIGH);
  }

  if (!sensorOk)
  {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  }
}

void sendDataToFirebase()
{
  if (!Firebase.ready())
    return;

  long ts = millis();

  // Only send if readings are valid
  if (!isnan(temperature) && !isnan(humidity))
  {
    Firebase.RTDB.setFloat(&fbdo, "/sensor/temperature", temperature);
    Firebase.RTDB.setFloat(&fbdo, "/sensor/humidity", humidity);
    Firebase.RTDB.setInt(&fbdo, "/sensor/lastUpdated", ts);

    String chamberStatus = (heaterState) ? "critical" : (fanState ? "warning" : "normal");
    Firebase.RTDB.setString(&fbdo, "/chambers/chamber_1/status", chamberStatus);
    Firebase.RTDB.setFloat(&fbdo, "/chambers/chamber_1/temperature", temperature);
    Firebase.RTDB.setFloat(&fbdo, "/chambers/chamber_1/humidity", humidity);
    Firebase.RTDB.setInt(&fbdo, "/chambers/chamber_1/lastUpdated", ts);

    Firebase.RTDB.setBool(&fbdo, "/control/fan", fanState);
    Firebase.RTDB.setBool(&fbdo, "/control/heater", heaterState);
    Firebase.RTDB.setInt(&fbdo, "/control/lastUpdated", ts);

    Firebase.RTDB.setBool(&fbdo, "/status/fan", fanState);
    Firebase.RTDB.setBool(&fbdo, "/status/heater", heaterState);
    Firebase.RTDB.setString(&fbdo, "/status/system", "operational");
    Firebase.RTDB.setInt(&fbdo, "/status/lastUpdated", ts);
  }

  Firebase.RTDB.setInt(&fbdo, "/system/rssi", WiFi.RSSI());
  Firebase.RTDB.setInt(&fbdo, "/system/uptime", millis() / 1000);
}

void loop()
{
  if (millis() - lastUpdate >= updateInterval)
  {
    lastUpdate = millis();

    bool sensorOk = readSensor();

    if (Firebase.ready())
    {
      fetchSettingsFromFirebase();
      if (sensorOk)
      {
        applyControl();
        checkAndTriggerAlert(); // Check for alert conditions
      } // Only control if sensor is good
      sendDataToFirebase();
    }

    updateLEDs(sensorOk);

    if (sensorOk)
    {
      Serial.printf("Temp: %.2f°C | Hum: %.2f%% | Fan: %s (%d) | Heater: %s | Humidifier: %s | Mode: %s\n",
                    temperature, humidity,
                    fanState ? "ON" : "OFF", ledcRead(FAN_PWM_CHANNEL),
                    heaterState ? "ON" : "OFF",
                    humidifierState ? "ON" : "OFF",
                    autoMode ? "AUTO" : "MANUAL");
      alertActive ? " | *** ALERT ACTIVE ***" : "";
    }
    else
    {
      Serial.println("Sensor error - no valid readings");
    }
  }
}