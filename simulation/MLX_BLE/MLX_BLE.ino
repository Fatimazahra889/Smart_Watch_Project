#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ============ BLE CONFIGURATION ============
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "ESP32-C3-Device"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 1000; // Send every 1 second

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    
    // Request larger MTU
    pServer->updatePeerMTU(pServer->getConnId(), 517);
    
    Serial.println("\n✓✓✓ CLIENT CONNECTED! ✓✓✓");
    Serial.println("Your app is now connected!");
    Serial.println("Starting to send temperature data...\n");
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
  Serial.println("MLX90614 BLE Temperature Sender");
  Serial.println("DIAGNOSTIC MODE");
  Serial.println("================================");
  Serial.print("Device Name: ");
  Serial.println(DEVICE_NAME);
  Serial.println("\nWaiting for connection...\n");
  
  // Initialize I2C with custom pins for ESP32-C3
  Wire.begin(8, 9);  // SDA = GPIO 8, SCL = GPIO 9
  
  // Initialize MLX90614
  Serial.println("🌡️  Initializing MLX90614...");
  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 not detected! Check wiring.");
    while (1);
  }
  Serial.println("✅ MLX90614 initialized\n");
  
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
  
  // Create TX Characteristic (for sending data to app)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // Create RX Characteristic (for receiving data from app)
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

void sendTemperatureData() {
  // Read body temperature from MLX90614
  float bodyTemp = mlx.readObjectTempC();
  
  // Build compact JSON - use "temperature" not "bodyTemp"!
  String jsonData = "{\"heartRate\":0,\"spo2\":0,\"steps\":0,\"sleep\":\"0h00min\",\"temperature\":";
  jsonData += String(bodyTemp, 1);
  jsonData += "}";
  
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
  // Handle disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("🔄 Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }
  
  // Handle new connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Send temperature data periodically
  if (deviceConnected) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSendTime >= SEND_INTERVAL) {
      sendTemperatureData();
      lastSendTime = currentTime;
    }
  }

  delay(100);
}