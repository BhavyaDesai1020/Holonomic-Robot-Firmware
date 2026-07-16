#include <ESP32Encoder.h>

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting encoder distance measurement...");

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder1.attachFullQuad(39,36); 
  encoder2.attachFullQuad(33,32); 
  encoder3.attachFullQuad(35,34);
  Serial.println("Encoders initialized.");
}

void loop() {
  int32_t count1 = encoder1.getCount();
  int32_t count2 = encoder2.getCount();
  int32_t count3 = encoder3.getCount();

  Serial.print("C1: "); Serial.print(count1);
  Serial.print(" | C2: "); Serial.print(count2);
  Serial.print(" | C3: "); Serial.println(count3);
  delay(200);
}