#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>
#include <Adafruit_MLX90614.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// GPIO Pins for ESP32-C3 Super Mini
#define I2C_SDA 8
#define I2C_SCL 9
#define MAX30102_ADDRESS 0x57

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// ============ BLE CONFIGURATION ============
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "ESP32-C3-Device"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ============ SENSOR OBJECTS ============
MAX30105 particleSensor;
MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============ MAX30102 VARIABLES ============
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

volatile bool fingerDetected = false;
volatile long irValue = 0;
volatile long redValue = 0;
volatile float heartRate = 0.0;
volatile long interBeatInterval = 0;
volatile float signalQuality = 0.0;
volatile int spo2 = 0;

bool lastFingerState = false;
unsigned long lastStateChangeTime = 0;
const unsigned long DEBOUNCE_TIME = 1000;

const int SPO2_BUFFER_SIZE = 100;
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int spo2SampleCount = 0;

// ============ MPU6050 VARIABLES ============
unsigned long stepCount = 0;
float lastAccelMagnitude = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
const unsigned long MIN_STEP_INTERVAL = 200;
const float STEP_THRESHOLD = 0.8;
const float STEP_THRESHOLD_MAX = 3.0;

unsigned long totalSleepTime = 0;
unsigned long sleepStartTime = 0;
bool isSleeping = false;
const float SLEEP_THRESHOLD = 0.15;
const unsigned long SLEEP_START_DELAY = 5000;
unsigned long stillStartTime = 0;
bool isStill = false;
const int DEMO_SPEED_MULTIPLIER = 60;

String activityState = "UNKNOWN";
float currentAccelMagnitude = 0;

// ============ MLX90614 VARIABLES ============
float bodyTemperature = 0.0;
bool mlxInitialized = false;

// ============ OLED VARIABLES ============
bool oledInitialized = false;
unsigned long lastOLEDUpdate = 0;
const unsigned long OLED_UPDATE_INTERVAL = 500; // Update every 500ms

// ============ BLE DATA TRANSMISSION ============
unsigned long lastBLEUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL = 1000;

// ============ BLE SERVER CALLBACKS ============
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    pServer->updatePeerMTU(pServer->getConnId(), 517);
    Serial.println("\n✓✓✓ CLIENT CONNECTED! ✓✓✓");
    Serial.println("Your app is now connected!");
    Serial.println("Starting to send all sensor data...\n");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("\n✗✗✗ CLIENT DISCONNECTED! ✗✗✗");
    Serial.println("Connection closed.");
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n═══════════════════════════════════════");
  Serial.println("   COMBINED HEALTH MONITOR SYSTEM");
  Serial.println("═══════════════════════════════════════");
  Serial.println("MAX30102 + MPU6050 + MLX90614 + OLED + BLE");
  Serial.print("Device Name: ");
  Serial.println(DEVICE_NAME);
  Serial.println("═══════════════════════════════════════\n");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // ============ INITIALIZE OLED ============
  Serial.println("🔧 Initializing OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("❌ OLED not found at 0x3C!");
    Serial.println("   Continuing without display...");
    oledInitialized = false;
  } else {
    Serial.println("✅ OLED initialized");
    oledInitialized = true;
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 25);
    display.println(F("Health Monitor"));
    display.setCursor(30, 40);
    display.println(F("Starting..."));
    display.display();
    delay(2000);
  }
  
  // ============ INITIALIZE MAX30102 ============
  Serial.println("🔧 Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDRESS)) {
    Serial.println("❌ MAX30102 not found!");
    Serial.println("   Continuing without heart rate sensor...");
  } else {
    Serial.println("✅ MAX30102 initialized");
    byte ledBrightness = 0x1F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 400;
    int pulseWidth = 411;
    int adcRange = 4096;
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.enableDIETEMPRDY();
    
    for(byte x = 0; x < RATE_SIZE; x++) {
      rates[x] = 0;
    }
  }
  
  // ============ INITIALIZE MPU6050 ============
  Serial.println("🔧 Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not found!");
    Serial.println("   Continuing without motion sensor...");
  } else {
    Serial.println("✅ MPU6050 initialized");
    Serial.println("   Calibrating... keep device still...");
    delay(2000);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("✅ MPU6050 calibrated");
  }
  
  // ============ INITIALIZE MLX90614 ============
  Serial.println("🔧 Initializing MLX90614...");
  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 not found!");
    Serial.println("   Continuing without temperature sensor...");
    mlxInitialized = false;
  } else {
    Serial.println("✅ MLX90614 initialized");
    mlxInitialized = true;
  }
  
  // ============ INITIALIZE BLE ============
  Serial.println("\n📡 Initializing BLE...");
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(517);
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ BLE service started!");
  Serial.println("✅ Device is discoverable!\n");
  
  Serial.println("═══════════════════════════════════════");
  Serial.println("          SYSTEM READY!");
  Serial.println("═══════════════════════════════════════");
  Serial.println("👆 Place finger on MAX30102");
  Serial.println("🚶 Walk to count steps");
  Serial.println("😴 Stay still 5s for sleep tracking");
  Serial.println("⚡ DEMO: 1 sec = 1 min displayed\n");
}

void updateOLED() {
  if (!oledInitialized) return;
  
  display.clearDisplay();
  
  // Title bar
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Health Monitor"));
  
  // BLE Status
  display.setCursor(100, 0);
  display.print(deviceConnected ? F("BLE") : F("---"));
  
  // Line separator
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Heart Rate
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print(F("HR:"));
  display.setTextSize(2);
  display.setCursor(25, 13);
  if (heartRate > 0) {
    display.print((int)heartRate);
    display.setTextSize(1);
    display.print(F("bpm"));
  } else {
    display.print(F("--"));
  }
  
  // SpO2
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print(F("O2:"));
  display.setTextSize(2);
  display.setCursor(25, 29);
  if (spo2 > 0) {
    display.print(spo2);
    display.setTextSize(1);
    display.print(F("%"));
  } else {
    display.print(F("--"));
  }
  
  // Bottom section - Steps, Sleep, Temperature
  display.drawLine(0, 45, 128, 45, SSD1306_WHITE);
  
  display.setTextSize(1);
  
  // Steps
  display.setCursor(0, 48);
  display.print(F("Steps:"));
  display.setCursor(40, 48);
  display.print(stepCount);
  
  // Sleep
  display.setCursor(0, 57);
  display.print(F("Sleep:"));
  display.setCursor(40, 57);
  if (isSleeping) {
    unsigned long currentSleep = millis() - sleepStartTime;
    unsigned long totalWithCurrent = totalSleepTime + currentSleep;
    unsigned long demoTime = totalWithCurrent * DEMO_SPEED_MULTIPLIER;
    float sleepHours = demoTime / 3600000.0;
    display.print(sleepHours, 1);
    display.print(F("h"));
  } else {
    unsigned long demoTotal = totalSleepTime * DEMO_SPEED_MULTIPLIER;
    float sleepHours = demoTotal / 3600000.0;
    display.print(sleepHours, 1);
    display.print(F("h"));
  }
  
  // Temperature
  display.setCursor(85, 48);
  display.print(F("T:"));
  display.setCursor(97, 48);
  if (bodyTemperature > 0) {
    display.print(bodyTemperature, 1);
    display.print(F("C"));
  } else {
    display.print(F("--"));
  }
  
  display.display();
}

void sendAllSensorData() {
  // Calculate sleep time in demo format
  unsigned long totalWithCurrent = totalSleepTime;
  if (isSleeping) {
    totalWithCurrent += (millis() - sleepStartTime);
  }
  unsigned long demoTotal = totalWithCurrent * DEMO_SPEED_MULTIPLIER;
  float sleepHours = demoTotal / 3600000.0;
  
  // Validate temperature reading (handle NaN)
  float validTemp = bodyTemperature;
  if (isnan(validTemp) || validTemp < -40 || validTemp > 100) {
    validTemp = 0.0;
  }
  
  // Build JSON with all sensor data
  String jsonData = "{\"heartRate\":";
  jsonData += String((int)heartRate);
  jsonData += ",\"spo2\":";
  jsonData += String(spo2);
  jsonData += ",\"steps\":";
  jsonData += String(stepCount);
  jsonData += ",\"sleepHours\":";
  jsonData += String(sleepHours, 1);
  jsonData += ",\"temperature\":";
  jsonData += String(validTemp, 1);
  jsonData += "}";
  
  // Send via BLE
  pTxCharacteristic->setValue(jsonData.c_str());
  pTxCharacteristic->notify();
  
  Serial.println("📤 Sent to app: " + jsonData);
}

void loop() {
  unsigned long currentTime = millis();
  
  // ============ HANDLE BLE CONNECTION ============
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // ============ READ MAX30102 (Heart Rate & SpO2) ============
  irValue = particleSensor.getIR();
  redValue = particleSensor.getRed();
  
  bool currentFingerDetected = (irValue > 50000);
  
  if (currentFingerDetected != lastFingerState) {
    lastStateChangeTime = currentTime;
  }
  
  if (currentTime - lastStateChangeTime >= DEBOUNCE_TIME) {
    if (currentFingerDetected && !fingerDetected) {
      fingerDetected = true;
      Serial.println("🎯 FINGER DETECTED");
      lastBeat = 0;
      for(byte x = 0; x < RATE_SIZE; x++) {
        rates[x] = 0;
      }
      rateSpot = 0;
      beatAvg = 0;
      spo2SampleCount = 0;
    } else if (!currentFingerDetected && fingerDetected) {
      fingerDetected = false;
      Serial.println("👋 FINGER REMOVED");
      heartRate = 0.0;
      interBeatInterval = 0;
      signalQuality = 0.0;
      spo2 = 0;
    }
  }
  lastFingerState = currentFingerDetected;
  
  if (fingerDetected) {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      if (delta > 300 && delta < 3000) {
        interBeatInterval = delta;
        beatsPerMinute = 60000.0 / delta;
        
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) {
            beatAvg += rates[x];
          }
          beatAvg /= RATE_SIZE;
          heartRate = beatAvg;
          
          Serial.print("💓 BEAT! BPM: ");
          Serial.println(beatAvg);
        }
      }
    }
    
    // SpO2 Calculation
    if (spo2SampleCount < SPO2_BUFFER_SIZE) {
      irBuffer[spo2SampleCount] = irValue;
      redBuffer[spo2SampleCount] = redValue;
      spo2SampleCount++;
    } else {
      spo2SampleCount = 0;
      
      double avgRed = 0, avgIR = 0;
      for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        avgRed += redBuffer[i];
        avgIR += irBuffer[i];
      }
      avgRed /= SPO2_BUFFER_SIZE;
      avgIR /= SPO2_BUFFER_SIZE;
      
      double rmsRed = 0, rmsIR = 0;
      for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        rmsRed += pow(redBuffer[i] - avgRed, 2);
        rmsIR += pow(irBuffer[i] - avgIR, 2);
      }
      rmsRed = sqrt(rmsRed / SPO2_BUFFER_SIZE);
      rmsIR = sqrt(rmsIR / SPO2_BUFFER_SIZE);
      
      double R = (rmsRed / avgRed) / (rmsIR / avgIR);
      spo2 = (int)(110 - 25 * R);
      
      if (spo2 > 100) spo2 = 100;
      if (spo2 < 70) spo2 = 0;
    }
  }
  
  // ============ READ MPU6050 (Steps & Sleep) ============
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  currentAccelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  float movementMagnitude = abs(currentAccelMagnitude - 1.0);
  
  // Step Detection
  if (movementMagnitude > STEP_THRESHOLD && 
      movementMagnitude < STEP_THRESHOLD_MAX &&
      !stepDetected &&
      (currentTime - lastStepTime) > MIN_STEP_INTERVAL) {
    
    if (movementMagnitude > lastAccelMagnitude) {
      stepCount++;
      stepDetected = true;
      lastStepTime = currentTime;
      Serial.print("👣 STEP! Total: ");
      Serial.println(stepCount);
    }
  }
  
  if (movementMagnitude < STEP_THRESHOLD * 0.8) {
    stepDetected = false;
  }
  lastAccelMagnitude = movementMagnitude;
  
  // Sleep Detection
  if (movementMagnitude < SLEEP_THRESHOLD) {
    if (!isStill) {
      isStill = true;
      stillStartTime = currentTime;
      stepDetected = false;
      Serial.println("🛑 Device stopped moving...");
    }
    
    if (!isSleeping && (currentTime - stillStartTime) >= SLEEP_START_DELAY) {
      isSleeping = true;
      sleepStartTime = currentTime;
      stepCount = 0;
      Serial.println("😴 SLEEP STARTED (After 5 seconds still)");
    }
  } else {
    if (isSleeping) {
      isSleeping = false;
      isStill = false;
      unsigned long sleepDuration = currentTime - sleepStartTime;
      totalSleepTime += sleepDuration;
      Serial.println("⏰ SLEEP ENDED");
      stillStartTime = 0;
    } else if (isStill) {
      isStill = false;
      stillStartTime = 0;
    }
  }
  
  // ============ READ MLX90614 (Temperature) ============
  if (mlxInitialized) {
    bodyTemperature = mlx.readObjectTempC();
    
    if (isnan(bodyTemperature) || bodyTemperature < -40 || bodyTemperature > 100) {
      bodyTemperature = 0.0;
    }
  } else {
    bodyTemperature = 0.0;
  }
  
  // ============ UPDATE OLED DISPLAY ============
  if (currentTime - lastOLEDUpdate >= OLED_UPDATE_INTERVAL) {
    lastOLEDUpdate = currentTime;
    updateOLED();
  }
  
  // ============ SEND BLE DATA ============
  if (deviceConnected && (currentTime - lastBLEUpdate >= BLE_UPDATE_INTERVAL)) {
    lastBLEUpdate = currentTime;
    sendAllSensorData();
  }
  
  // ============ PERIODIC SERIAL DISPLAY ============
  static unsigned long lastDisplayTime = 0;
  if (currentTime - lastDisplayTime >= 5000) {
    lastDisplayTime = currentTime;
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║        ALL SENSORS STATUS             ║");
    Serial.println("╠═══════════════════════════════════════╣");
    Serial.print("║ 💓 Heart Rate: ");
    Serial.print(heartRate > 0 ? String((int)heartRate) + " BPM" : "---");
    Serial.println();
    Serial.print("║ 🩸 SpO2: ");
    Serial.print(spo2 > 0 ? String(spo2) + "%" : "---");
    Serial.println();
    Serial.print("║ 👣 Steps: ");
    Serial.println(stepCount);
    Serial.print("║ 😴 Sleep: ");
    if (isSleeping) {
      unsigned long currentSleep = currentTime - sleepStartTime;
      unsigned long demoTime = currentSleep * DEMO_SPEED_MULTIPLIER;
      Serial.print("SLEEPING (");
      Serial.print(demoTime / 3600000);
      Serial.print("h ");
      Serial.print((demoTime % 3600000) / 60000);
      Serial.println("m)");
    } else {
      Serial.println("AWAKE");
    }
    Serial.print("║ 🌡️  Temperature: ");
    if (bodyTemperature > 0) {
      Serial.print(bodyTemperature, 1);
      Serial.println("°C");
    } else {
      Serial.println("---");
    }
    Serial.print("║ 📡 BLE: ");
    Serial.println(deviceConnected ? "CONNECTED" : "DISCONNECTED");
    Serial.println("╚═══════════════════════════════════════╝\n");
  }
  
  delay(10);
}