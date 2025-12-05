#include <Wire.h>
#include <MPU6050.h>

// GPIO Pins for ESP32-C3 Super Mini
#define I2C_SDA 8  // GPIO 8 for SDA
#define I2C_SCL 9  // GPIO 9 for SCL

// MPU6050 Object
MPU6050 mpu;

// ============ STEP COUNTER VARIABLES ============
unsigned long stepCount = 0;
float lastAccelMagnitude = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
const unsigned long MIN_STEP_INTERVAL = 200;  // Minimum 200ms between steps (max 5 steps/sec)
const float STEP_THRESHOLD = 1.3;              // Threshold for step detection
const float STEP_THRESHOLD_MAX = 3.0;          // Maximum threshold to filter jumps

// ============ SLEEP TRACKER VARIABLES ============
// DEMO MODE: 1 real second = 1 minute displayed (60x speed)
unsigned long totalSleepTime = 0;              // Total sleep in real milliseconds
unsigned long sleepStartTime = 0;
bool isSleeping = false;
bool wasSleeping = false;

// Movement detection for sleep
const float SLEEP_THRESHOLD = 0.15;            // Low movement threshold for sleep
const unsigned long SLEEP_START_DELAY = 15000; // 15 seconds real time to start sleep
unsigned long stillStartTime = 0;
bool isStill = false;

// Demo mode multiplier: 60x speed (1 sec real = 1 min displayed)
const int DEMO_SPEED_MULTIPLIER = 60;

// Activity state tracking
String activityState = "UNKNOWN";
float currentAccelMagnitude = 0;

// Calibration variables
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🚀 MPU6050 Step Counter & Sleep Tracker");
  Serial.println("📋 ESP32-C3 Super Mini Configuration");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // Initialize MPU6050
  Serial.println("🔧 Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 connection failed!");
    Serial.println("   Check your wiring:");
    Serial.println("   SDA -> GPIO 8");
    Serial.println("   SCL -> GPIO 9");
    Serial.println("   VCC -> 3.3V");
    Serial.println("   GND -> GND");
    while (1);
  }
  
  Serial.println("✅ MPU6050 connected successfully");
  
  // Calibrate sensor
  Serial.println("🔧 Calibrating MPU6050...");
  Serial.println("   Keep the device STILL on a flat surface...");
  delay(2000);
  
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  
  Serial.println("✅ Calibration complete!");
  Serial.println("\n📊 Starting measurements...");
  Serial.println("🚶 Walk around to count steps");
  Serial.println("😴 Stay still for 15 seconds to start sleep tracking");
  Serial.println("⚡ DEMO MODE: 1 real second = 1 minute displayed\n");
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // ============ READ ACCELEROMETER DATA ============
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Convert to g-force (MPU6050 default: ±2g, 16384 LSB/g)
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  // Calculate total acceleration magnitude (removing gravity baseline)
  currentAccelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  
  // Calculate movement deviation from gravity (1g)
  float movementMagnitude = abs(currentAccelMagnitude - 1.0);
  
  // ============ STEP DETECTION ALGORITHM ============
  // Detect peaks in acceleration that indicate steps
  if (movementMagnitude > STEP_THRESHOLD && 
      movementMagnitude < STEP_THRESHOLD_MAX &&
      !stepDetected &&
      (currentTime - lastStepTime) > MIN_STEP_INTERVAL) {
    
    // Check if this is a peak (higher than previous value)
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
  
  // Reset step detection flag when acceleration drops
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
  
  // ============ SLEEP DETECTION ALGORITHM ============
  
  // Check if device is still (minimal movement)
  if (movementMagnitude < SLEEP_THRESHOLD) {
    if (!isStill) {
      // Just became still
      isStill = true;
      stillStartTime = currentTime;
      
      // Stop counting steps when still
      stepDetected = false;
      
      Serial.println("\n🛑 Device stopped moving - monitoring for sleep...");
    }
    
    // If still for SLEEP_START_DELAY (15 seconds), start sleep
    if (!isSleeping && (currentTime - stillStartTime) >= SLEEP_START_DELAY) {
      isSleeping = true;
      sleepStartTime = currentTime;
      stepCount = 0; // Reset step count when sleeping
      
      Serial.println("\n😴 SLEEP STARTED (After 15 seconds still)");
      Serial.println("═══════════════════════════════════");
      Serial.println("   💤 Now tracking sleep time...");
      Serial.println("   ⚡ DEMO: 1 real sec = 1 min shown");
      Serial.println("═══════════════════════════════════\n");
    }
  } else {
    // Device is moving
    if (isSleeping) {
      // Was sleeping, now moving - WAKE UP IMMEDIATELY
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
      // Was still but not sleeping yet, now moving
      isStill = false;
      stillStartTime = 0;
      Serial.println("✅ Movement detected - step counting active");
    }
  }
  
  // Update current sleep time if sleeping
  unsigned long currentSleepDuration = 0;
  if (isSleeping) {
    currentSleepDuration = currentTime - sleepStartTime;
  }
  
  // ============ PERIODIC STATUS DISPLAY ============
  static unsigned long lastDisplayTime = 0;
  if (currentTime - lastDisplayTime >= 5000) {  // Every 5 seconds
    lastDisplayTime = currentTime;
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║         ACTIVITY SUMMARY               ║");
    Serial.println("╠════════════════════════════════════════╣");
    
    Serial.print("║ 👣 Steps Today: ");
    Serial.print(stepCount);
    Serial.println(" steps");
    
    Serial.print("║ 🏃 Activity: ");
    Serial.println(activityState);
    
    Serial.print("║ 📊 Movement Level: ");
    Serial.print(movementMagnitude, 3);
    Serial.println(" g");
    
    Serial.print("║ 😴 Sleep Status: ");
    if (isSleeping) {
      Serial.print("SLEEPING (");
      unsigned long currentSleep = currentTime - sleepStartTime;
      // Show demo time
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
    if (totalSleepTime > 0 || isSleeping) {
      unsigned long totalWithCurrent = totalSleepTime;
      if (isSleeping) {
        totalWithCurrent += (currentTime - sleepStartTime);
      }
      // Show demo time (60x speed)
      unsigned long demoTotal = totalWithCurrent * DEMO_SPEED_MULTIPLIER;
      int hours = demoTotal / 3600000;
      int minutes = (demoTotal % 3600000) / 60000;
      Serial.print(hours);
      Serial.print("h ");
      Serial.print(minutes);
      Serial.println("m");
    } else {
      Serial.println("0h 0m");
    }
    
    Serial.print("║ ⏱️  Uptime: ");
    Serial.print(currentTime / 1000);
    Serial.println(" sec");
    
    Serial.println("╚════════════════════════════════════════╝\n");
  }
  
  delay(50);  // 20Hz sampling rate (good for step detection)
}

// Helper function to print duration in readable format
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

// Helper function to print DEMO time (60x speed: 1 sec real = 1 min displayed)
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