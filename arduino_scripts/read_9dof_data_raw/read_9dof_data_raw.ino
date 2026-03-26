#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

unsigned long lastPrint = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!gyro.init()) {
    Serial.println("Failed to detect L3GD20 gyro");
    while (1) {}
  }
  gyro.enableDefault();

  if (!compass.init()) {
    Serial.println("Failed to detect LSM303 accel/mag");
    while (1) {}
  }
  compass.enableDefault();

  Serial.println("IMU initialized.");
}

void loop() {
  // Read sensors
  gyro.read();
  compass.read();

  // Print at 50 Hz (every 20 ms)
  unsigned long now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;

    Serial.print("G ");
    Serial.print(gyro.g.x); Serial.print(" ");
    Serial.print(gyro.g.y); Serial.print(" ");
    Serial.print(gyro.g.z);

    Serial.print(" | A ");
    Serial.print(compass.a.x); Serial.print(" ");
    Serial.print(compass.a.y); Serial.print(" ");
    Serial.print(compass.a.z);

    Serial.print(" | M ");
    Serial.print(compass.m.x); Serial.print(" ");
    Serial.print(compass.m.y); Serial.print(" ");
    Serial.print(compass.m.z);

    Serial.println();
  }

  // No delay() here if you also handle encoders; keep the loop responsive.
}
