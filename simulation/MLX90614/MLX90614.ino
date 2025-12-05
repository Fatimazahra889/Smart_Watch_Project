#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);

  // Use custom I2C pins for ESP32-C3
  Wire.begin(8, 9);  // SDA = 8, SCL = 9

  if (!mlx.begin()) {
    Serial.println("MLX90614 not detected! Check wiring.");
    while (1);
  }

  Serial.println("MLX90614 Body Temperature Reader");
}

void loop() {
  // Read object temperature (temperature of the body)
  float bodyTemp = mlx.readObjectTempC();

  Serial.print("Body Temperature: ");
  Serial.print(bodyTemp);
  Serial.println(" °C");

  delay(500); // refresh every 0.5s
}
