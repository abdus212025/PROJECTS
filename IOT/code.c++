#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>

#define BUZZER_PIN D2

MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi Connected!");

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found.");
    while (1);
  }
  particleSensor.setup(); // Default config
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  // ----- Fall Detection -----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  if (totalAccel > 25) { // Adjust threshold based on testing
    Serial.println("FALL DETECTED!");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ----- Health Monitoring -----
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    static uint32_t lastBeat = 0;
    uint32_t beatTime = millis();
    int bpm = 60000 / (beatTime - lastBeat);
    lastBeat = beatTime;

    Serial.print("Heart Rate: ");
    Serial.print(bpm);
    Serial.println(" BPM");

    // Simple SpO2 (not very accurate without calibration)
    float spo2 = 95 + random(-2, 2); // simulated
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.println(" %");
  }

  delay(100); // Loop delay
}
