#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>

// ============ GPIO PINS FOR ESP32-C3 SUPER MINI ============
#define I2C_SDA 8  // GPIO 8 for SDA
#define I2C_SCL 9  // GPIO 9 for SCL

// ============ OLED DISPLAY ============
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============ SENSOR OBJECTS ============
MPU6050 mpu;
MAX30105 heartSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ============ MAX30102 CONFIGURATION ============
#define MAX30102_ADDRESS 0x57
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;

// Heart Rate Variables
volatile bool fingerDetected = false;
volatile long irValue = 0;
volatile long redValue = 0;
volatile float heartRate = 0.0;
volatile int spo2 = 0;

// Finger Detection
bool lastFingerState = false;
unsigned long lastStateChangeTime = 0;
const unsigned long DEBOUNCE_TIME = 1000;

// SpO2 Calculation
const int SPO2_BUFFER_SIZE = 100;
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int spo2SampleCount = 0;

// ============ MPU6050 STEP COUNTER ============
unsigned long stepCount = 0;
float lastAccelMagnitude = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
const unsigned long MIN_STEP_INTERVAL = 200;
const float STEP_THRESHOLD = 1.3;
const float STEP_THRESHOLD_MAX = 3.0;

// Activity State
String activityState = "UNKNOWN";
float currentAccelMagnitude = 0;

// ============ SLEEP TRACKER ============
unsigned long totalSleepTime = 0;
unsigned long sleepStartTime = 0;
bool isSleeping = false;
bool wasSleeping = false;

const float SLEEP_THRESHOLD = 0.15;
const unsigned long SLEEP_START_DELAY = 15000; // 15 seconds
unsigned long stillStartTime = 0;
bool isStill = false;

const int DEMO_SPEED_MULTIPLIER = 60; // 1 sec real = 1 min displayed

// ============ MLX90614 TEMPERATURE ============
float bodyTemperature = 0.0;
float ambientTemperature = 0.0;

// ============ DISPLAY UPDATE ============
unsigned long lastOLEDUpdate = 0;
const unsigned long OLED_UPDATE_INTERVAL = 500; // Update OLED every 500ms

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   🚀 ESP32-C3 SMART WATCH - FULL SYSTEM      ║");
  Serial.println("║   📱 Multi-Sensor Health Monitoring Device    ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  // ============ INITIALIZE I2C ============
  Serial.println("🔧 Initializing I2C Bus...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz Fast I2C
  Serial.println("✅ I2C initialized (SDA: GPIO 8, SCL: GPIO 9)\n");
  
  // ============ INITIALIZE OLED DISPLAY ============
  Serial.println("🖥️  Initializing OLED Display (0x3C)...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("❌ OLED Display not found at 0x3C!");
    while(1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart Watch");
  display.println("Initializing...");
  display.display();
  Serial.println("✅ OLED Display initialized\n");
  
  // ============ INITIALIZE MPU6050 ============
  Serial.println("🏃 Initializing MPU6050 (Step Counter & Sleep Tracker)...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 connection failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 ERROR!");
    display.display();
    while (1);
  }
  
  Serial.println("✅ MPU6050 connected");
  Serial.println("🔧 Calibrating MPU6050...");
  Serial.println("   Keep device STILL on flat surface...");
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.println("Keep still!");
  display.display();
  
  delay(2000);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  
  Serial.println("✅ MPU6050 calibration complete\n");
  
  // ============ INITIALIZE MAX30102 ============
  Serial.println("💓 Initializing MAX30102 (Heart Rate & SpO2)...");
  if (!heartSensor.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDRESS)) {
    Serial.println("❌ MAX30102 not found at 0x57!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MAX30102 ERROR!");
    display.display();
    while (1);
  }
  
  Serial.println("✅ MAX30102 detected at 0x57");
  
  // Configure MAX30102
  byte ledBrightness = 0x1F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 400;
  int pulseWidth = 411;
  int adcRange = 4096;
  
  heartSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  heartSensor.enableDIETEMPRDY();
  
  Serial.println("✅ MAX30102 configured\n");
  
  // Initialize rate array
  for(byte x = 0; x < RATE_SIZE; x++) {
    rates[x] = 0;
  }
  
  // ============ INITIALIZE MLX90614 ============
  Serial.println("🌡️  Initializing MLX90614 (Temperature Sensor)...");
  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 not detected!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MLX90614 ERROR!");
    display.display();
    while (1);
  }
  
  Serial.println("✅ MLX90614 initialized\n");
  
  // ============ STARTUP COMPLETE ============
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║          ✅ ALL SENSORS INITIALIZED           ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  Serial.println("📊 Starting continuous monitoring...");
  Serial.println("🚶 Walk around to count steps");
  Serial.println("😴 Stay still for 15 seconds to start sleep tracking");
  Serial.println("👆 Place finger on MAX30102 for heart rate & SpO2");
  Serial.println("⚡ DEMO MODE: 1 real second = 1 minute displayed\n");
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("All Systems GO!");
  display.display();
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // ============ READ MPU6050 (ACCELEROMETER) ============
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  currentAccelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  float movementMagnitude = abs(currentAccelMagnitude - 1.0);
  
  // ============ STEP DETECTION ============
  if (movementMagnitude > STEP_THRESHOLD && 
      movementMagnitude < STEP_THRESHOLD_MAX &&
      !stepDetected &&
      (currentTime - lastStepTime) > MIN_STEP_INTERVAL) {
    
    if (movementMagnitude > lastAccelMagnitude) {
      stepCount++;
      stepDetected = true;
      lastStepTime = currentTime;
      
      Serial.print("👣 STEP DETECTED! Total Steps: ");
      Serial.print(stepCount);
      Serial.print(" | Magnitude: ");
      Serial.println(movementMagnitude, 3);
    }
  }
  
  if (movementMagnitude < STEP_THRESHOLD * 0.8) {
    stepDetected = false;
  }
  
  lastAccelMagnitude = movementMagnitude;
  
  // ============ ACTIVITY STATE CLASSIFICATION ============
  if (movementMagnitude < SLEEP_THRESHOLD) {
    activityState = "SLEEPING/REST";
  } else if (movementMagnitude < 0.3) {
    activityState = "SITTING";
  } else if (movementMagnitude < 0.6) {
    activityState = "LIGHT ACTIVITY";
  } else if (movementMagnitude < 1.0) {
    activityState = "WALKING";
  } else if (movementMagnitude < 1.5) {
    activityState = "FAST WALKING";
  } else {
    activityState = "RUNNING/ACTIVE";
  }
  
  // ============ SLEEP DETECTION ============
  if (movementMagnitude < SLEEP_THRESHOLD) {
    if (!isStill) {
      isStill = true;
      stillStartTime = currentTime;
      stepDetected = false;
      Serial.println("\n🛑 Device stopped moving - monitoring for sleep...");
    }
    
    if (!isSleeping && (currentTime - stillStartTime) >= SLEEP_START_DELAY) {
      isSleeping = true;
      sleepStartTime = currentTime;
      stepCount = 0;
      
      Serial.println("\n😴 SLEEP STARTED (After 15 seconds still)");
      Serial.println("═══════════════════════════════════");
      Serial.println("   💤 Now tracking sleep time...");
      Serial.println("   ⚡ DEMO: 1 real sec = 1 min shown");
      Serial.println("═══════════════════════════════════\n");
    }
  } else {
    if (isSleeping) {
      isSleeping = false;
      isStill = false;
      unsigned long sleepDuration = currentTime - sleepStartTime;
      totalSleepTime += sleepDuration;
      
      Serial.println("\n⏰ SLEEP ENDED (Movement detected!)");
      Serial.println("═══════════════════════════════════");
      Serial.print("   Sleep Duration (Real): ");
      printDuration(sleepDuration);
      Serial.print("   Sleep Duration (Demo): ");
      printDemoTime(sleepDuration);
      Serial.print("   Total Sleep (Demo): ");
      printDemoTime(totalSleepTime);
      Serial.println("   🚶 Now counting steps again...");
      Serial.println("═══════════════════════════════════\n");
      
      stillStartTime = 0;
    } else if (isStill) {
      isStill = false;
      stillStartTime = 0;
      Serial.println("✅ Movement detected - step counting active");
    }
  }
  
  // ============ READ MAX30102 (HEART RATE & SPO2) ============
  irValue = heartSensor.getIR();
  redValue = heartSensor.getRed();
  
  // Finger Detection with Debouncing
  bool currentFingerDetected = (irValue > 50000);
  
  if (currentFingerDetected != lastFingerState) {
    lastStateChangeTime = currentTime;
  }
  
  if (currentTime - lastStateChangeTime >= DEBOUNCE_TIME) {
    if (currentFingerDetected && !fingerDetected) {
      fingerDetected = true;
      Serial.println("🎯 FINGER DETECTED - Starting heart rate & SpO2 measurements...");
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
      spo2 = 0;
    }
  }
  lastFingerState = currentFingerDetected;
  
  // Heart Rate Calculation
  if (fingerDetected) {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      if (delta > 300 && delta < 3000) {
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
          Serial.print(beatAvg);
          Serial.print(" | IBI: ");
          Serial.print(delta);
          Serial.print("ms | IR: ");
          Serial.println(irValue);
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
  
  // ============ READ MLX90614 (TEMPERATURE) ============
  bodyTemperature = mlx.readObjectTempC();
  ambientTemperature = mlx.readAmbientTempC();
  
  // ============ UPDATE OLED DISPLAY ============
  if (currentTime - lastOLEDUpdate >= OLED_UPDATE_INTERVAL) {
    lastOLEDUpdate = currentTime;
    updateOLED();
  }
  
  // ============ PERIODIC DETAILED SERIAL OUTPUT ============
  static unsigned long lastDetailedOutput = 0;
  if (currentTime - lastDetailedOutput >= 5000) {
    lastDetailedOutput = currentTime;
    printDetailedStatus(currentTime);
  }
  
  delay(50);
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setCursor(20, 0);
  display.println("Smart Watch");
  
  // SpO2
  display.setCursor(0, 12);
  display.print("SpO2: ");
  if (spo2 > 0 && fingerDetected) {
    display.print(spo2);
    display.println("%");
  } else {
    display.println("--");
  }
  
  // Heart Rate
  display.setCursor(0, 22);
  display.print("HR: ");
  if (heartRate > 0 && fingerDetected) {
    display.print((int)heartRate);
    display.println(" bpm");
  } else {
    display.println("-- bpm");
  }
  
  // Body Temperature
  display.setCursor(0, 32);
  display.print("Temp: ");
  display.print(bodyTemperature, 1);
  display.println(" C");
  
  // Steps
  display.setCursor(0, 42);
  display.print("Steps: ");
  display.println(stepCount);
  
  // Sleep Time
  display.setCursor(0, 52);
  display.print("Sleep: ");
  unsigned long totalWithCurrent = totalSleepTime;
  if (isSleeping) {
    totalWithCurrent += (millis() - sleepStartTime);
  }
  unsigned long demoTotal = totalWithCurrent * DEMO_SPEED_MULTIPLIER;
  int hours = demoTotal / 3600000;
  int minutes = (demoTotal % 3600000) / 60000;
  display.print(hours);
  display.print("h ");
  display.print(minutes);
  display.println("m");
  
  display.display();
}

void printDetailedStatus(unsigned long currentTime) {
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║              📊 COMPLETE SYSTEM STATUS REPORT             ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  
  // MPU6050 Status
  Serial.println("║                    🏃 MPU6050 - ACTIVITY                   ║");
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.print("║ 👣 Steps Today: ");
  Serial.print(stepCount);
  Serial.println(" steps");
  Serial.print("║ 🏃 Activity State: ");
  Serial.println(activityState);
  Serial.print("║ 📊 Movement Level: ");
  Serial.print(currentAccelMagnitude, 3);
  Serial.println(" g");
  
  Serial.print("║ 😴 Sleep Status: ");
  if (isSleeping) {
    Serial.print("SLEEPING (");
    unsigned long currentSleep = currentTime - sleepStartTime;
    unsigned long demoTime = currentSleep * DEMO_SPEED_MULTIPLIER;
    int demoHours = demoTime / 3600000;
    int demoMinutes = (demoTime % 3600000) / 60000;
    Serial.print(demoHours);
    Serial.print("h ");
    Serial.print(demoMinutes);
    Serial.println("m)");
  } else if (isStill) {
    Serial.print("STILL (");
    Serial.print((currentTime - stillStartTime) / 1000);
    Serial.print("/15 sec → sleep)");
    Serial.println();
  } else {
    Serial.println("AWAKE & ACTIVE");
  }
  
  Serial.print("║ 🕐 Total Sleep (Demo): ");
  unsigned long totalWithCurrent = totalSleepTime;
  if (isSleeping) {
    totalWithCurrent += (currentTime - sleepStartTime);
  }
  unsigned long demoTotal = totalWithCurrent * DEMO_SPEED_MULTIPLIER;
  int hours = demoTotal / 3600000;
  int minutes = (demoTotal % 3600000) / 60000;
  Serial.print(hours);
  Serial.print("h ");
  Serial.print(minutes);
  Serial.println("m");
  
  // MAX30102 Status
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.println("║              💓 MAX30102 - HEART RATE & SPO2               ║");
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.print("║ 👆 Finger Status: ");
  Serial.println(fingerDetected ? "DETECTED" : "NOT DETECTED");
  Serial.print("║ 💓 Heart Rate: ");
  if (heartRate > 0 && fingerDetected) {
    Serial.print(heartRate, 0);
    Serial.println(" BPM");
  } else {
    Serial.println("Waiting for finger...");
  }
  Serial.print("║ 🩸 SpO2: ");
  if (spo2 > 0 && fingerDetected) {
    Serial.print(spo2);
    Serial.println("%");
  } else {
    Serial.println("Calculating...");
  }
  Serial.print("║ 📡 IR Signal: ");
  Serial.println(irValue);
  Serial.print("║ 🔴 Red Signal: ");
  Serial.println(redValue);
  
  // MLX90614 Status
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.println("║             🌡️  MLX90614 - TEMPERATURE SENSOR              ║");
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.print("║ 🧑 Body Temperature: ");
  Serial.print(bodyTemperature, 2);
  Serial.println(" °C");
  Serial.print("║ 🌡️  Ambient Temperature: ");
  Serial.print(ambientTemperature, 2);
  Serial.println(" °C");
  
  // System Info
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.println("║                    ⚙️  SYSTEM INFORMATION                  ║");
  Serial.println("╟────────────────────────────────────────────────────────────╢");
  Serial.print("║ ⏱️  Uptime: ");
  Serial.print(currentTime / 1000);
  Serial.println(" seconds");
  Serial.print("║ 🖥️  OLED Display: ");
  Serial.println("ACTIVE (0x3C)");
  Serial.print("║ 📡 I2C Bus: ");
  Serial.println("400kHz (GPIO 8 & 9)");
  
  Serial.println("╚════════════════════════════════════════════════════════════╝\n");
}

void printDuration(unsigned long milliseconds) {
  int hours = milliseconds / 3600000;
  int minutes = (milliseconds % 3600000) / 60000;
  int seconds = (milliseconds % 60000) / 1000;
  
  Serial.print(hours);
  Serial.print("h ");
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.println("s");
}

void printDemoTime(unsigned long realMilliseconds) {
  unsigned long demoMilliseconds = realMilliseconds * DEMO_SPEED_MULTIPLIER;
  int hours = demoMilliseconds / 3600000;
  int minutes = (demoMilliseconds % 3600000) / 60000;
  int seconds = (demoMilliseconds % 60000) / 1000;
  
  Serial.print(hours);
  Serial.print("h ");
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.println("s");
}