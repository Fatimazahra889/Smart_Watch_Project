#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPIO Pins for ESP32-C3 Super Mini
#define I2C_SDA 8  // GPIO 8 for SDA
#define I2C_SCL 9  // GPIO 9 for SCL

// MAX30102 I2C Address
#define MAX30102_ADDRESS 0x57

// ============ BLE CONFIGURATION ============
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "ESP32-C3-Device"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensor Object
MAX30105 particleSensor;

// Heart Rate Detection Variables (Using Library's Algorithm)
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Global Data Variables
volatile bool fingerDetected = false;
volatile long irValue = 0;
volatile long redValue = 0;
volatile float heartRate = 0.0;
volatile long interBeatInterval = 0;
volatile float signalQuality = 0.0;
volatile int spo2 = 0;

// Finger Detection
bool lastFingerState = false;
unsigned long lastStateChangeTime = 0;
const unsigned long DEBOUNCE_TIME = 1000;

// SpO2 Calculation Variables
const int SPO2_BUFFER_SIZE = 100;
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int spo2SampleCount = 0;

// ============ BLE DATA TRANSMISSION ============
unsigned long lastBLEUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL = 1000; // Send every 1 second

// ============ BLE SERVER CALLBACKS ============
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    
    // Request larger MTU
    pServer->updatePeerMTU(pServer->getConnId(), 517);
    
    Serial.println("\n✓✓✓ CLIENT CONNECTED! ✓✓✓");
    Serial.println("Your app is now connected!");
    Serial.println("Starting to send heart rate & SpO2 data...\n");
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
  
  Serial.println("\n================================");
  Serial.println("MAX30102 BLE HR & SpO2 Sender");
  Serial.println("DIAGNOSTIC MODE");
  Serial.println("================================");
  Serial.print("Device Name: ");
  Serial.println(DEVICE_NAME);
  Serial.println("\nWaiting for connection...\n");
  
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz Fast I2C
  
  // Initialize MAX30102
  Serial.println("🔧 Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDRESS)) {
    Serial.println("❌ MAX30102 not found at address 0x57!");
    Serial.println("   Check your wiring:");
    Serial.println("   SDA -> GPIO 8");
    Serial.println("   SCL -> GPIO 9");
    Serial.println("   VIN -> 3.3V");
    Serial.println("   GND -> GND");
    while (1);
  }
  
  Serial.println("✅ MAX30102 detected at address 0x57");
  
  // Configure MAX30102 for wrist reading (optimized settings)
  byte ledBrightness = 0x1F;  // 31 - Good for wrist (0x00 to 0xFF)
  byte sampleAverage = 4;      // Average 4 samples
  byte ledMode = 2;            // Red + IR mode for SpO2
  int sampleRate = 400;        // 400 samples per second
  int pulseWidth = 411;        // 411μs pulse width
  int adcRange = 4096;         // ADC range 4096
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
  
  Serial.println("✅ MAX30102 configured successfully");
  Serial.println("\n👆 Place your finger/wrist on the sensor");
  Serial.println("⏳ Wait for stable readings...\n");
  
  // Initialize rate array
  for(byte x = 0; x < RATE_SIZE; x++) {
    rates[x] = 0;
  }
  
  // Initialize BLE
  Serial.println("📡 Initializing BLE...");
  BLEDevice::init(DEVICE_NAME);
  
  // Set MTU size
  BLEDevice::setMTU(517);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // Create RX Characteristic
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("✓ BLE service started!");
  Serial.println("✓ MTU set to 517 bytes");
  Serial.println("✓ Device is discoverable!\n");
}

void sendHeartRateAndSpO2Data() {
  // Build compact JSON
  String jsonData = "{\"heartRate\":";
  jsonData += String((int)heartRate);
  jsonData += ",\"spo2\":";
  jsonData += String(spo2);
  jsonData += ",\"steps\":0,\"sleepHours\":0,\"temperature\":0}";
  
  // Check size
  int dataSize = jsonData.length();
  Serial.print("📦 JSON size: ");
  Serial.print(dataSize);
  Serial.println(" bytes");
  
  if (dataSize > 512) {
    Serial.println("⚠️ WARNING: Data too large!");
  }
  
  // Send to app
  pTxCharacteristic->setValue(jsonData.c_str());
  pTxCharacteristic->notify();
  
  // Print to Serial
  Serial.println("📤 Sent to app:");
  Serial.println(jsonData);
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // ============ HANDLE BLE CONNECTION ============
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("🔄 Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // Read sensor values
  irValue = particleSensor.getIR();
  redValue = particleSensor.getRed();
  
  // ============ FINGER DETECTION WITH DEBOUNCING ============
  bool currentFingerDetected = (irValue > 50000);
  
  if (currentFingerDetected != lastFingerState) {
    lastStateChangeTime = currentTime;
  }
  
  if (currentTime - lastStateChangeTime >= DEBOUNCE_TIME) {
    if (currentFingerDetected && !fingerDetected) {
      fingerDetected = true;
      Serial.println("🎯 FINGER DETECTED - Starting measurements...");
      // Reset calculations
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
  
  // ============ HEART RATE CALCULATION (Library Algorithm) ============
  if (fingerDetected) {
    // Use the proven checkForBeat() function from the library
    if (checkForBeat(irValue) == true) {
      // Beat detected!
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      // Filter physiologically valid beats
      if (delta > 300 && delta < 3000) { // Between 20-200 BPM
        interBeatInterval = delta;
        beatsPerMinute = 60000.0 / delta;
        
        // Additional filtering
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          
          // Calculate average BPM
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) {
            beatAvg += rates[x];
          }
          beatAvg /= RATE_SIZE;
          heartRate = beatAvg;
          
          Serial.print("💓 BEAT! BPM: ");
          Serial.print(beatAvg);
          Serial.print(" | IBI: ");
          Serial.print(delta);
          Serial.print("ms | IR: ");
          Serial.println(irValue);
        }
      }
    }
    
    // ============ SPO2 CALCULATION ============
    // Collect samples for SpO2
    if (spo2SampleCount < SPO2_BUFFER_SIZE) {
      irBuffer[spo2SampleCount] = irValue;
      redBuffer[spo2SampleCount] = redValue;
      spo2SampleCount++;
    } else {
      // Calculate SpO2 when buffer is full
      spo2SampleCount = 0;
      
      // Calculate DC averages
      double avgRed = 0, avgIR = 0;
      for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        avgRed += redBuffer[i];
        avgIR += irBuffer[i];
      }
      avgRed /= SPO2_BUFFER_SIZE;
      avgIR /= SPO2_BUFFER_SIZE;
      
      // Calculate AC RMS
      double rmsRed = 0, rmsIR = 0;
      for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        rmsRed += pow(redBuffer[i] - avgRed, 2);
        rmsIR += pow(irBuffer[i] - avgIR, 2);
      }
      rmsRed = sqrt(rmsRed / SPO2_BUFFER_SIZE);
      rmsIR = sqrt(rmsIR / SPO2_BUFFER_SIZE);
      
      // Calculate R value
      double R = (rmsRed / avgRed) / (rmsIR / avgIR);
      
      // SpO2 calculation using empirical formula
      spo2 = (int)(110 - 25 * R);
      
      // Limit to realistic values
      if (spo2 > 100) spo2 = 100;
      if (spo2 < 70) spo2 = 0; // Likely invalid
    }
    
    // ============ SIGNAL QUALITY ESTIMATION ============
    static long lastIrValue = 0;
    if (lastIrValue != 0) {
      long variation = abs(irValue - lastIrValue);
      signalQuality = constrain((variation / 1000.0) * 100.0, 0.0, 100.0);
    }
    lastIrValue = irValue;
    
    // ============ PERIODIC DISPLAY (Every 2 seconds) ============
    static unsigned long lastDisplayTime = 0;
    if (currentTime - lastDisplayTime >= 2000) {
      lastDisplayTime = currentTime;
      
      Serial.println("\n╔═══════════════════════════════════╗");
      Serial.println("║       MEASUREMENT RESULTS         ║");
      Serial.println("╠═══════════════════════════════════╣");
      
      Serial.print("║ 💓 Heart Rate: ");
      if (heartRate > 0) {
        Serial.print(heartRate, 0);
        Serial.println(" BPM");
      } else {
        Serial.println("Detecting...");
      }
      
      Serial.print("║ 🩸 SpO2: ");
      if (spo2 > 0) {
        Serial.print(spo2);
        Serial.println("%");
      } else {
        Serial.println("Calculating...");
      }
      
      Serial.print("║ ⏱️  IBI: ");
      Serial.print(interBeatInterval);
      Serial.println(" ms");
      
      Serial.print("║ 📊 Signal Quality: ");
      Serial.print(signalQuality, 1);
      Serial.println("%");
      
      Serial.print("║ 📡 IR Value: ");
      Serial.println(irValue);
      
      Serial.println("╚═══════════════════════════════════╝\n");
    }
    
  } else {
    // No finger detected
    static unsigned long lastNoFingerMsg = 0;
    if (currentTime - lastNoFingerMsg >= 3000) {
      lastNoFingerMsg = currentTime;
      Serial.println("⏳ Waiting for finger placement... (IR: " + String(irValue) + ")");
    }
  }
  
  // ============ SEND BLE DATA ============
  if (deviceConnected && (currentTime - lastBLEUpdate >= BLE_UPDATE_INTERVAL)) {
    lastBLEUpdate = currentTime;
    sendHeartRateAndSpO2Data();
  }
  
  delay(10); // Small delay for stability
}