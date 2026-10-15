#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const int MOIST_PIN = 34;
const int RAIN_PIN = 36;
const int OVERRIDE_PIN = 19;
const int DHT_PIN = 32;

// Water level sensor
const int WATER_LEVEL_PIN = 35;

// State LEDs
const int IRRIGATE_LED = 16;
const int HOLD_LED = 17;
const int RAIN_LED = 4;
const int MANUAL_LED = 2;

// Tank empty buzzer
const int BUZZER_PIN = 15;

#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

const int PUMP_PIN = 23;
const int VALVE_PIN = 5;

// Active LOW relay
const int RELAY_ON = LOW;
const int RELAY_OFF = HIGH;

// Valve settling time before pump starts/stops
const unsigned long SETTLE_MS = 1500;

// Hysteresis margin
const int HYSTERESIS_MARGIN = 8;

// Firmware absolute maximum runtime
const int MAX_RUNTIME_MINUTES = 20;

// SOIL MOISTURE CALIBRATION
const int RAW_DRY = 4095;
const int RAW_WET = 2700;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// KEYPAD
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {13, 14, 27, 26};
byte colPins[COLS] = {25, 33, 18};

Keypad keypad = Keypad(
  makeKeymap(keys),
  rowPins,
  colPins,
  ROWS,
  COLS
);

// OVERRIDE
volatile bool overrideFlag = false;
volatile unsigned long lastIsr = 0;
volatile unsigned long overrideTime = 0;

bool manualOverride = false;

// Hysteresis state
bool moistureIrrigation = false;

// SETTINGS
struct Settings {
  int runMinutes;
  int setpointPct;
  int startHour;
};

Settings cfg = {5, 40, 6};

Preferences prefs;

// Wi-Fi
const char* WIFI_SSID = "DRDOOM";
const char* WIFI_PASSWORD = "i-am-doom";

// Network Time
const char* NTP_SERVER = "pool.ntp.org";

const long GMT_OFFSET_SEC = 8 * 3600;
const int DAYLIGHT_OFFSET_SEC = 0;

// Open-Meteo, Manila
const char* WEATHER_URL =
  "https://api.open-meteo.com/v1/forecast?"
  "latitude=14.6042&"
  "longitude=120.9822&"
  "hourly=temperature_2m,relative_humidity_2m,"
  "rain,soil_moisture_0_to_1cm,precipitation_probability";

// Weather rain hold
const int RAIN_HOLD_PCT = 1;

// Forecast becomes stale after 3 hours.
const unsigned long FORECAST_MAX_AGE =
  3UL * 60UL * 60UL * 1000UL;

// Fetch new forecast every 10 minutes.
const unsigned long FORECAST_FETCH_INTERVAL =
  10UL * 60UL * 1000UL;

int rainProbability = 0;

float forecastTemperature = 0;
float forecastHumidity = 0;
float forecastRain = 0;
float forecastSoilMoisture = 0;

String forecastTime = "";

unsigned long forecastFetchedAt = 0;
unsigned long lastForecastFetch = 0;

bool forecastAvailable = false;
bool forecastStaleDueToWiFi = false;
bool wifiWasConnected = false;

// MENU VARIABLES
int menuItem = 0;

bool menuActive = false;
bool enteringValue = false;

String inputValue = "";

// FARM STATES
enum FarmState {
  S_HOLD,
  S_IRRIGATE,
  S_RAIN,
  S_MANUAL
};


// LAB 6 PART A: scheduled tasks and cached sensor values
unsigned long tSense = 0;
unsigned long tUi = 0;
unsigned long tLog = 0;

const unsigned long SENSE_INTERVAL = 1000UL;
const unsigned long UI_INTERVAL = 200UL;
const unsigned long LOG_INTERVAL = 2000UL;

float soilMoisturePct = 0.0;
float airTemperature = NAN;
float airHumidity = NAN;
bool localRainDetected = false;
bool localTankEmpty = false;
FarmState currentState = S_HOLD;
FarmState displayedState = S_HOLD;
bool stateDisplayInitialized = false;


// LAB 6 PART B: Bluetooth Low Energy (BLE) service and characteristics.
#define BLE_SERVICE_UUID  "a1b2c3d4-0001-4000-8000-00805f9b34fb"
#define BLE_STATE_UUID    "a1b2c3d4-0002-4000-8000-00805f9b34fb" // Read + Notify
#define BLE_VALUES_UUID   "a1b2c3d4-0003-4000-8000-00805f9b34fb" // Read + Notify
#define BLE_SETPOINT_UUID "a1b2c3d4-0004-4000-8000-00805f9b34fb" // Read + Write
#define BLE_COMMAND_UUID  "a1b2c3d4-0005-4000-8000-00805f9b34fb" // Write

BLEServer* bleServer = nullptr;
BLEService* bleService = nullptr;
BLECharacteristic* chState = nullptr;
BLECharacteristic* chValues = nullptr;
BLECharacteristic* chSetpoint = nullptr;
BLECharacteristic* chCommand = nullptr;
bool bleClientConnected = false;

// LAB 6 PART C: Tracking variables for change detection
String lastNotifiedValues = "";
FarmState lastNotifiedState = S_HOLD;

class SmartFarmServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    Serial.println("BLE client connected.");
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    Serial.println("BLE client disconnected; restarting advertising.");
    BLEDevice::startAdvertising();
  }
};

const char* farmStateName(FarmState state) {
  switch (state) {
    case S_IRRIGATE: return "IRRIGATE";
    case S_RAIN:     return "RAIN_DETECTED";
    case S_MANUAL:   return "MANUAL_OVERRIDE";
    case S_HOLD:
    default:         return "HOLD";
  }
}

void startBLEService() {
  BLEDevice::init("Group6-SmartFarm-Node");

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new SmartFarmServerCallbacks());

  bleService = bleServer->createService(BLE_SERVICE_UUID);

  chState = bleService->createCharacteristic(
    BLE_STATE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  chState->addDescriptor(new BLE2902());
  chState->setValue(farmStateName(currentState));

  chValues = bleService->createCharacteristic(
    BLE_VALUES_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  chValues->addDescriptor(new BLE2902());

  char initialValues[100];
  snprintf(
    initialValues,
    sizeof(initialValues),
    "soil=%.1f;temp=%.1f;humidity=%.1f;rain=%d;tankEmpty=%d",
    soilMoisturePct,
    airTemperature,
    airHumidity,
    localRainDetected ? 1 : 0,
    localTankEmpty ? 1 : 0
  );
  chValues->setValue(initialValues);

  chSetpoint = bleService->createCharacteristic(
    BLE_SETPOINT_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  chSetpoint->addDescriptor(new BLE2902());
  chSetpoint->setValue(String(cfg.setpointPct).c_str());

  chCommand = bleService->createCharacteristic(
    BLE_COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  chCommand->setValue("");

  bleService->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE service started.");
  Serial.println("BLE device name: SmartFarm-Node");
  Serial.println("BLE characteristics: STATE, VALUES, SETPOINT, COMMAND");
}

// LAB 6 PART C: Notification function
void notifyBLEClient() {
  if (!bleClientConnected) return;

  // Format the current sensor values as a short delimited string
  char buf[100];
  snprintf(
    buf,
    sizeof(buf),
    "soil=%.1f;temp=%.1f;humidity=%.1f;rain=%d;tankEmpty=%d",
    soilMoisturePct,
    airTemperature,
    airHumidity,
    localRainDetected ? 1 : 0,
    localTankEmpty ? 1 : 0
  );

  String currentValues = String(buf);
  
  // Notify the connected client whenever a value changes, rather than on a fixed timer alone
  if (currentValues != lastNotifiedValues) {
    chValues->setValue(buf);
    chValues->notify();
    lastNotifiedValues = currentValues;
  }

  // Notify state changes
  if (currentState != lastNotifiedState) {
    chState->setValue(farmStateName(currentState));
    chState->notify();
    lastNotifiedState = currentState;
  }
}

// Non-blocking keypad feedback messages
bool menuMessageActive = false;
unsigned long menuMessageUntil = 0;

// ACTUATOR STATES
enum ActuatorPhase {
  PH_IDLE,
  PH_OPENING,
  PH_RUNNING,
  PH_CLOSING
};

ActuatorPhase actuatorPhase = PH_IDLE;
unsigned long phaseStartTime = 0;

// TIMED IRRIGATION
unsigned long irrigationStartTime = 0;
unsigned long irrigationDuration = 0;

bool irrigationActive = false;

unsigned long lastDisplayedSecond = 999999;

// OVERRIDE INTERRUPT
void IRAM_ATTR onOverride() {

  unsigned long now = millis();
  if (now - lastIsr < 200) {
    return;
  }

  lastIsr = now;

  overrideFlag = true;
  overrideTime = now;
}


// SOIL MOISTURE
int readMoistureRaw(int n = 10) {

  long sum = 0;

  for (int i = 0; i < n; i++) {

    sum += analogRead(MOIST_PIN);
  }

  return sum / n;
}

float readMoisturePct() {

  int raw = readMoistureRaw();

  float pct =
    100.0 * (RAW_DRY - raw) /
    (float)(RAW_DRY - RAW_WET);

  return constrain(pct, 0.0, 100.0);
}

// RAIN SENSOR
bool isRaining() {

  return digitalRead(RAIN_PIN) == LOW;
}

// WATER LEVEL
bool tankEmpty() {

  return digitalRead(WATER_LEVEL_PIN) == LOW;
}


// Sample each local sensor once per scheduled sensing cycle.
void readAllSensors() {
  soilMoisturePct = readMoisturePct();
  localRainDetected = isRaining();
  localTankEmpty = tankEmpty();

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // Keep the last valid reading if the DHT22 returns an invalid sample.
  if (!isnan(t)) airTemperature = t;
  if (!isnan(h)) airHumidity = h;
}

// UPDATE STATE LEDs
void updateStateLEDs(FarmState state) {

  digitalWrite(IRRIGATE_LED, LOW);
  digitalWrite(HOLD_LED, LOW);
  digitalWrite(RAIN_LED, LOW);
  digitalWrite(MANUAL_LED, LOW);

  if (state == S_IRRIGATE) {

    digitalWrite(IRRIGATE_LED, HIGH);
  }

  else if (state == S_HOLD) {

    digitalWrite(HOLD_LED, HIGH);
  }

  else if (state == S_RAIN) {

    digitalWrite(RAIN_LED, HIGH);
  }

  else if (state == S_MANUAL) {

    digitalWrite(MANUAL_LED, HIGH);
  }
}

// UPDATE TANK BUZZER
void updateTankBuzzer() {

  if (localTankEmpty) {

    digitalWrite(BUZZER_PIN, HIGH);

  }

  else {

    digitalWrite(BUZZER_PIN, LOW);
  }
}

// DECIDE FARM STATE
FarmState decide() {

  float moisture = soilMoisturePct;

  if (manualOverride) {

    return S_MANUAL;
  }

  if (localRainDetected) {

    return S_RAIN;
  }

  if (weatherRainHold()) {

    return S_RAIN;
  }

  if (moistureIrrigation) {

    if (moisture >= cfg.setpointPct + HYSTERESIS_MARGIN) {

      moistureIrrigation = false;
    }
  }

  else {

    if (moisture < cfg.setpointPct) {

      moistureIrrigation = true;
    }
  }

  if (moistureIrrigation) {

    return S_IRRIGATE;
  }

  return S_HOLD;
}

// DISPLAY FARM STATE
void displayState(FarmState state) {

  lcd.clear();

  if (state == S_MANUAL) {
    lcd.setCursor(0, 0);
    lcd.print("State: MANUAL");
    lcd.setCursor(0, 1);
    lcd.print("By: Override");
  }

  else if (state == S_RAIN) {
    lcd.setCursor(0, 0);
    lcd.print("State: RAIN");
    lcd.setCursor(0, 1);
    lcd.print("By: Rain Sensor");
  }

  else if (state == S_IRRIGATE) {
    lcd.setCursor(0, 0);
    lcd.print("State: IRRIGATE");
    lcd.setCursor(0, 1);
    lcd.print("By: Soil Moist");
  }

  else {
    lcd.setCursor(0, 0);
    lcd.print("State: HOLD");
    lcd.setCursor(0, 1);
    lcd.print("By: Moisture OK");
  }
}


// SAVE SETTINGS
void saveSettings() {

  prefs.begin("irrig", false);

  prefs.putInt("run", cfg.runMinutes);
  prefs.putInt("sp", cfg.setpointPct);
  prefs.putInt("hr", cfg.startHour);

  prefs.end();
}

// LOAD SETTINGS
void loadSettings() {

  prefs.begin("irrig", true);

  cfg.runMinutes = prefs.getInt("run", 5);
  cfg.setpointPct = prefs.getInt("sp", 40);
  cfg.startHour = prefs.getInt("hr", 6);

  prefs.end();
}

// APPLY SETTING
bool applySetting(int which, int value) {

  switch (which) {

    case 0:

      if (value < 1 || value > 30) {
        return false;
      }

      cfg.runMinutes = value;

      break;

    case 1:

      if (value < 10 || value > 90) {
        return false;
      }

      cfg.setpointPct = value;

      break;

    case 2:

      if (value < 0 || value > 23) {
        return false;
      }

      cfg.startHour = value;

      break;
  }

  saveSettings();

  if (which == 1 && chSetpoint != nullptr) {
    chSetpoint->setValue(String(cfg.setpointPct).c_str());
  }

  return true;
}

// START WI-FI WITHOUT HOLDING UP LOCAL CONTROLLER STARTUP
void connectWiFi() {
  Serial.println();
  Serial.println("Starting Wi-Fi connection in background...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// CONFIGURE NETWORK TIME WITHOUT WAITING FOR SYNCHRONIZATION
void setupTime() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    Serial.println("Network time synchronization requested.");
  }
}

// FETCH WEATHER FORECAST
bool fetchForecast() {

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("Wi-Fi disconnected. Forecast not updated.");

    return false;
  }

  Serial.println();
  Serial.println("Requesting weather forecast...");

  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(3000);
  http.begin(WEATHER_URL);

  int httpCode = http.GET();

  Serial.print("HTTP Response Code: ");
  Serial.println(httpCode);

  if (httpCode <= 0) {

    Serial.print("HTTP request failed: ");
    Serial.println(http.errorToString(httpCode));

    http.end();

    return false;
  }

  String payload = http.getString();

  Serial.print("Response size: ");
  Serial.print(payload.length());
  Serial.println(" bytes");

  DynamicJsonDocument doc(30000);

  DeserializationError error =
    deserializeJson(doc, payload);

  if (error) {

    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());

    http.end();

    return false;
  }

  Serial.println("JSON parsing successful!");

  JsonObject hourly = doc["hourly"];

  forecastTime =
    hourly["time"][0].as<String>();

  forecastTemperature =
    hourly["temperature_2m"][0];

  forecastHumidity =
    hourly["relative_humidity_2m"][0];

  forecastRain =
    hourly["rain"][0];

  forecastSoilMoisture =
    hourly["soil_moisture_0_to_1cm"][0];

  rainProbability =
    hourly["precipitation_probability"][0];

  forecastFetchedAt = millis();
  lastForecastFetch = millis();
  forecastAvailable = true;
  forecastStaleDueToWiFi = false;

  Serial.println();
  Serial.println("========== FORECAST ==========");

  Serial.print("Forecast time: ");
  Serial.println(forecastTime);

  Serial.print("Temperature: ");
  Serial.print(forecastTemperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(forecastHumidity);
  Serial.println(" %");

  Serial.print("Rain: ");
  Serial.print(forecastRain);
  Serial.println(" mm");

  Serial.print("Soil moisture: ");
  Serial.println(forecastSoilMoisture);

  Serial.print("Rain probability: ");
  Serial.print(rainProbability);
  Serial.println(" %");

  Serial.println("==============================");

  http.end();

  return true;
}

// CHECK FORECAST AGE
bool forecastIsFresh() {

  if (!forecastAvailable) {
    return false;
  }

  if (forecastStaleDueToWiFi) {
    return false;
  }

  unsigned long age =
    millis() - forecastFetchedAt;

  return age <= FORECAST_MAX_AGE;
}

// HANDLE WI-FI STATUS
void serviceWiFi() {

  bool wifiConnected =
    WiFi.status() == WL_CONNECTED;

  if (wifiWasConnected && !wifiConnected) {

    forecastStaleDueToWiFi = true;

    Serial.println();
    Serial.println("Wi-Fi disconnected.");
    Serial.println("Forecast marked STALE.");
    Serial.println("Using local sensors.");
  }

  if (!wifiWasConnected && wifiConnected) {

    Serial.println();
    Serial.println("Wi-Fi reconnected.");
    Serial.println("Waiting for scheduled forecast update...");
  }

  wifiWasConnected = wifiConnected;
}

// DISPLAY WEATHER STATUS
void displayWeatherStatus() {

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Rain:");
  lcd.print(rainProbability);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Age:");

  if (forecastAvailable) {

    unsigned long ageMinutes =
      (millis() - forecastFetchedAt) / 60000UL;

    lcd.print(ageMinutes);
    lcd.print("m ");

  }

  else {

    lcd.print("-- ");
  }

  if (forecastIsFresh()) {

    lcd.print("FRESH");

  }

  else {

    lcd.print("STALE");
  }
}

// CHECK WEATHER RAIN HOLD
bool weatherRainHold() {

  if (!forecastIsFresh()) {
    return false;
  }

  return rainProbability >= RAIN_HOLD_PCT;
}

// DISPLAY MENU
void showMenu() {

  lcd.clear();

  if (menuItem == 0) {

    lcd.setCursor(0, 0);
    lcd.print("1:Pump Runtime");

    lcd.setCursor(0, 1);
    lcd.print(cfg.runMinutes);
    lcd.print(" min");
  }

  else if (menuItem == 1) {

    lcd.setCursor(0, 0);
    lcd.print("2:Moisture Set");

    lcd.setCursor(0, 1);
    lcd.print(cfg.setpointPct);
    lcd.print("%");
  }

  else if (menuItem == 2) {

    lcd.setCursor(0, 0);
    lcd.print("3:Start Hour");

    lcd.setCursor(0, 1);
    lcd.print(cfg.startHour);
    lcd.print(":00");
  }
}

// START NUMERIC ENTRY
void startEntry() {

  enteringValue = true;
  inputValue = "";

  lcd.clear();

  if (menuItem == 0) {

    lcd.print("Pump Time:");
  }

  else if (menuItem == 1) {

    lcd.print("Setpoint:");
  }

  else if (menuItem == 2) {

    lcd.print("Start Hour:");
  }

  lcd.setCursor(0, 1);
}


// HANDLE ENTRYs 
void handleEntry(char key) {

  if (key >= '0' && key <= '9') {

    if (inputValue.length() < 2) {

      inputValue += key;

      lcd.setCursor(0, 1);
      lcd.print("                ");

      lcd.setCursor(0, 1);
      lcd.print(inputValue);
    }
  }


  else if (key == '#') {

    if (inputValue.length() == 0) {
      return;
    }

    int value = inputValue.toInt();

    if (applySetting(menuItem, value)) {
      lcd.clear();
      lcd.print("Value accepted!");
      menuMessageActive = true;
      menuMessageUntil = millis() + 1000UL;
      enteringValue = false;
      inputValue = "";
    }

    else {
      lcd.clear();
      lcd.print("INVALID VALUE");
      lcd.setCursor(0, 1);

      if (menuItem == 0) lcd.print("Range: 1-30");
      else if (menuItem == 1) lcd.print("Range: 10-90");
      else if (menuItem == 2) lcd.print("Range: 0-23");

      menuMessageActive = true;
      menuMessageUntil = millis() + 2000UL;
      enteringValue = false;
      inputValue = "";
    }
  }

  else if (key == '*') {

    enteringValue = false;
    inputValue = "";

    showMenu();
  }
}

// HANDLE MENU
void handleMenu(char key) {

  if (key == '1') {

    menuItem = 0;
    startEntry();
  }

  else if (key == '2') {

    menuItem = 1;
    startEntry();
  }

  else if (key == '3') {

    menuItem = 2;
    startEntry();
  }

  else if (key == '4') {

    menuItem = 0;
    showMenu();
  }

  else if (key == '5') {

    menuItem = 1;
    showMenu();
  }

  else if (key == '6') {

    menuItem = 2;
    showMenu();
  }

  else if (key == '#') {

    menuActive = false;
    stateDisplayInitialized = false;

    lcd.clear();
  }
}

// START IRRIGATION
void startIrrigation() {

  if (irrigationActive) {
    return;
  }

  Serial.println();
  Serial.println("=== IRRIGATION START ===");

  Serial.print("Runtime setting: ");
  Serial.print(cfg.runMinutes);
  Serial.println(" minute(s)");

  // Show when the firmware safety limit overrides the keypad setting
  if (cfg.runMinutes > MAX_RUNTIME_MINUTES) {
    Serial.print("Firmware limit applied: ");
    Serial.print(MAX_RUNTIME_MINUTES);
    Serial.println(" minute(s)");
  }

  // Runtime from keypad setting
  int effectiveRuntime = cfg.runMinutes;

  // Limit runtime to the firmware maximum
  if (effectiveRuntime > MAX_RUNTIME_MINUTES) {
    effectiveRuntime = MAX_RUNTIME_MINUTES;
  }

  irrigationDuration =
    (unsigned long)effectiveRuntime * 60000UL;

  irrigationActive = true;

  lastDisplayedSecond = 999999;

  // Start by opening valve
  digitalWrite(VALVE_PIN, RELAY_ON);

  Serial.println("VALVE ON");
  Serial.println("Waiting for valve to settle...");

  phaseStartTime = millis();

  actuatorPhase = PH_OPENING;
}

// STOP IRRIGATION
void stopIrrigation() {

  if (!irrigationActive) {
    return;
  }

  Serial.println();
  Serial.println("=== IRRIGATION STOPPING ===");

  // Pump OFF first
  digitalWrite(PUMP_PIN, RELAY_OFF);

  Serial.println("PUMP OFF");

  phaseStartTime = millis();

  actuatorPhase = PH_CLOSING;
}

// SERVICE ACTUATORS
void serviceActuators() {

  unsigned long now = millis();

  // VALVE OPENING
  if (actuatorPhase == PH_OPENING) {

    if (now - phaseStartTime >= SETTLE_MS) {

      // Start pump after valve settles
      digitalWrite(PUMP_PIN, RELAY_ON);

      Serial.println("PUMP ON");
      Serial.println("Timed irrigation running...");

      irrigationStartTime = now;

      actuatorPhase = PH_RUNNING;
    }
  }

  // PUMP RUNNING

  else if (actuatorPhase == PH_RUNNING) {

    // Check runtime
    if (now - irrigationStartTime >= irrigationDuration) {

      Serial.println("Runtime reached zero.");

      stopIrrigation();
    }

    // Display countdown
    else {

      unsigned long elapsed =
        now - irrigationStartTime;

      unsigned long remaining =
        irrigationDuration - elapsed;

      unsigned long totalSeconds =
        remaining / 1000UL;

      // Only update LCD when the second changes
      if (totalSeconds != lastDisplayedSecond) {

        lastDisplayedSecond = totalSeconds;

        unsigned int minutes =
          totalSeconds / 60;

        unsigned int seconds =
          totalSeconds % 60;

        lcd.setCursor(0, 0);
        lcd.print("IRRIGATION      ");

        lcd.setCursor(0, 1);

        lcd.print("Time: ");
        if (minutes < 10) {
          lcd.print("0");
        }

        lcd.print(minutes);
        lcd.print(":");

        if (seconds < 10) {
          lcd.print("0");
        }

        lcd.print(seconds);
        lcd.print("        ");
      }
    }
  }

  // VALVE CLOSING
  else if (actuatorPhase == PH_CLOSING) {

    if (now - phaseStartTime >= SETTLE_MS) {

      // Close valve after pump has stopped
      digitalWrite(VALVE_PIN, RELAY_OFF);

      Serial.println("VALVE OFF");
      Serial.println("=== IRRIGATION COMPLETE ===");

      irrigationActive = false;

      actuatorPhase = PH_IDLE;

      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print("Irrigation");
      lcd.setCursor(0, 1);
      lcd.print("Complete");
    }
  }
}

// SETUP
void setup() {
  Serial.begin(115200);

  // Set safe output states before starting sensors or network services.
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, RELAY_OFF);
  digitalWrite(VALVE_PIN, RELAY_OFF);

  pinMode(IRRIGATE_LED, OUTPUT);
  pinMode(HOLD_LED, OUTPUT);
  pinMode(RAIN_LED, OUTPUT);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(IRRIGATE_LED, LOW);
  digitalWrite(HOLD_LED, LOW);
  digitalWrite(RAIN_LED, LOW);
  digitalWrite(MANUAL_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(MOIST_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(OVERRIDE_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(OVERRIDE_PIN),
    onOverride,
    FALLING
  );

  dht.begin();
  lcd.init();
  lcd.backlight();
  loadSettings();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farm");
  lcd.setCursor(0, 1);
  lcd.print("System Ready");

  // Take an initial local reading before starting network activity.
  readAllSensors();
  currentState = decide();
  updateStateLEDs(currentState);
  updateTankBuzzer();

  // Start the BLE service after settings and initial sensor values are ready.
  startBLEService();

  // Start Wi-Fi without waiting for a connection or forecast request.
  connectWiFi();
  setupTime();
  lastForecastFetch = millis();
  tSense = millis();
  tUi = millis();
  tLog = millis();

  Serial.println("Local Smart Farm controller initialized.");
}

void loop() {
  const unsigned long now = millis();

  // Wi-Fi status is checked every loop; the controller remains local-first.
  serviceWiFi();

  // Retry network-time configuration after Wi-Fi becomes available.
  static bool timeConfigRequested = false;
  if (!timeConfigRequested && WiFi.status() == WL_CONNECTED) {
    setupTime();
    timeConfigRequested = true;
  }

  // Forecast refresh is scheduled. HTTPClient has a bounded timeout, but the
  // request can still briefly block this loop; this is documented for Part A.
  if (now - lastForecastFetch >= FORECAST_FETCH_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) {
      if (fetchForecast()) {
        Serial.println("Scheduled forecast update successful.");
      } else {
        Serial.println("Scheduled forecast update failed.");
      }
    } else {
      Serial.println("Wi-Fi disconnected. Forecast not updated.");
      forecastStaleDueToWiFi = true;
      lastForecastFetch = now;
    }
  }

  // Apply manual override requests raised by the interrupt.
  if (overrideFlag) {
    noInterrupts();
    overrideFlag = false;
    interrupts();

    manualOverride = !manualOverride;
    Serial.print("OVERRIDE SWITCH PRESSED -> ");
    Serial.println(manualOverride ? "MANUAL OVERRIDE ON" : "MANUAL OVERRIDE OFF");
  }

  // Scheduled sensor sampling and one state decision per sensing cycle.
  if (now - tSense >= SENSE_INTERVAL) {
    tSense = now;
    readAllSensors();
    currentState = decide();
    updateStateLEDs(currentState);
    updateTankBuzzer();
    
    // LAB 6 PART C: Push updates to the BLE client if values changed
    notifyBLEClient();
  }

  // Restore the menu after a timed feedback message without blocking the loop.
  if (menuMessageActive && (long)(now - menuMessageUntil) >= 0) {
    menuMessageActive = false;
    if (menuActive) showMenu();
  }

  // Keypad is serviced continuously; temporary feedback messages are preserved.
  if (!menuMessageActive) {
    char key = keypad.getKey();
    if (key) {
      if (key == '*') {
        if (!menuActive) {
          menuActive = true;
          enteringValue = false;
          showMenu();
        } else if (!enteringValue) {
          menuActive = false;
          stateDisplayInitialized = false;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Runtime: ");
          lcd.print(cfg.runMinutes);
          lcd.print(" min");
          lcd.setCursor(0, 1);
          lcd.print("Press * Menu");
        } else {
          handleEntry(key);
        }
      } else if (menuActive) {
        if (enteringValue) handleEntry(key);
        else handleMenu(key);
      }
    }
  }

  // Always service actuator timing, including while the menu is open.
  serviceActuators();

  // Avoid overwriting the menu or irrigation countdown with the state screen.
  if (!menuActive && !irrigationActive && !menuMessageActive &&
      (!stateDisplayInitialized || currentState != displayedState) &&
      now - tUi >= UI_INTERVAL) {
    tUi = now;
    displayState(currentState);
    displayedState = currentState;
    stateDisplayInitialized = true;
  }

  // Periodic diagnostics use cached readings, not extra sensor reads.
  if (now - tLog >= LOG_INTERVAL) {
    tLog = now;
    Serial.print("Moisture: ");
    Serial.print(soilMoisturePct, 1);
    Serial.println("%");

    Serial.print("Tank: ");
    Serial.println(localTankEmpty ? "EMPTY" : "OK");

    if (currentState == S_MANUAL) {
      Serial.println("Farm State: MANUAL_OVERRIDE");
      Serial.println("Deciding Input: Manual Override");
    } else if (currentState == S_RAIN) {
      Serial.println("Farm State: RAIN_DETECTED");
      Serial.println(localRainDetected ? "Deciding Input: Rain Sensor" : "Deciding Input: Weather Forecast");
    } else if (currentState == S_IRRIGATE) {
      Serial.println("Farm State: IRRIGATE");
      Serial.println("Deciding Input: Soil Moisture");
    } else {
      Serial.println("Farm State: HOLD");
      Serial.println("Deciding Input: Moisture OK");
    }

    Serial.print("Pump Runtime Setting: ");
    Serial.print(cfg.runMinutes);
    Serial.println(" min");
    Serial.print("Wi-Fi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.print("Forecast: ");
    Serial.println(forecastIsFresh() ? "FRESH" : "STALE");
    Serial.print("Rain Probability: ");
    Serial.print(rainProbability);
    Serial.println("%");
    Serial.println("-----------------------------");
  }

  // Start timed irrigation only when the current state calls for it.
  if (!menuActive && !irrigationActive && currentState == S_IRRIGATE) {
    Serial.println("Farm State: IRRIGATE; starting timed irrigation.");
    startIrrigation();
  }
}