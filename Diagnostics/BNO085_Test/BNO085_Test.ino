#include <Adafruit_BNO08x.h>
#include <math.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float initialYaw = 0.0f;
bool firstReading = true;

void setReports() {
  Serial.println("Setting desired reports");

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable Game Rotation Vector");
    while (1);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing BNO08x...");

  // If not detected, change address to 0x4A
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08x!");
    while (1);
  }

  Serial.println("BNO08x Found!");

  setReports();

  delay(100);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset");
    setReports();

    // Take a new zero reference after reset
    firstReading = true;
  }

  if (!bno08x.getSensorEvent(&sensorValue))
    return;

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {

    float qr = sensorValue.un.gameRotationVector.real;
    float qi = sensorValue.un.gameRotationVector.i;
    float qj = sensorValue.un.gameRotationVector.j;
    float qk = sensorValue.un.gameRotationVector.k;

    // Quaternion -> Yaw
    float yaw = atan2f(
                  2.0f * (qr * qk + qi * qj),
                  1.0f - 2.0f * (qj * qj + qk * qk)
                ) * 180.0f / PI;

    // Store startup orientation
    if (firstReading) {
      initialYaw = yaw;
      firstReading = false;
      Serial.println("Reference heading stored.");
    }

    // Relative yaw
    yaw -= initialYaw;

    // Wraping to -180° to +180°
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.println(" deg");
  }
}