#include <math.h>
#include <ESP32Servo.h>

struct __attribute__((packed)) ControlPacket {
  uint8_t head1 = 0xAA;
  uint8_t head2 = 0x55;
  int8_t lx, ly, rx, ry;
  uint8_t l2, r2;
  uint8_t l1, r1, square, circle, triangle, cross, up, down, right, left, l3, touchpad;
  uint8_t mode;
  uint8_t checksum;
  uint8_t footer = 0xFE;
};

ControlPacket incomingData;
unsigned long lastPacketTime = 0;

uint8_t rx_buffer[sizeof(ControlPacket)];
int rx_index = 0;

const int motor1_pin1 = 25;
const int motor1_pin2 = 26;
const int motor2_pin1 = 18;
const int motor2_pin2 = 19;
const int motor3_pin1 = 32;
const int motor3_pin2 = 33;

const int sh_servo1 = 21;
const int sh_servo2 = 4;
const int sh_servo3 = 5;

Servo myservo1;
Servo myservo2;
Servo myservo3;
int angle1 = 0;
int angle2 = 57;
int angle3 = 85;

void setMotor1(int pwm) {
  int p = constrain((int)abs(pwm), 0, 255);
  if (pwm >= 0) {
    analogWrite(motor1_pin1, p);
    analogWrite(motor1_pin2, 0);
  } else {
    analogWrite(motor1_pin1, 0);
    analogWrite(motor1_pin2, p);
  }
}
void setMotor2(int pwm) {
  int p = constrain((int)abs(pwm), 0, 255);
  if (pwm >= 0) {
    analogWrite(motor2_pin1, p);
    analogWrite(motor2_pin2, 0);
  } else {
    analogWrite(motor2_pin1, 0);
    analogWrite(motor2_pin2, p);
  }
}
void setMotor3(int pwm) {
  int p = constrain((int)abs(pwm), 0, 255);
  if (pwm >= 0) {
    analogWrite(motor3_pin1, p);
    analogWrite(motor3_pin2, 0);
  } else {
    analogWrite(motor3_pin1, 0);
    analogWrite(motor3_pin2, p);
  }
}
void stopAll() {
  setMotor1(0);
  setMotor2(0);
  setMotor3(0);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial2.setTimeout(5);

  myservo1.setPeriodHertz(50);
  myservo2.setPeriodHertz(50);
  myservo3.setPeriodHertz(50);

  myservo1.attach(sh_servo1, 500, 2400);
  myservo2.attach(sh_servo2, 500, 2400);
  myservo3.attach(sh_servo3, 500, 2400);
  myservo1.write(angle1);
  pinMode(motor1_pin1, OUTPUT);
  pinMode(motor1_pin2, OUTPUT);
  pinMode(motor2_pin1, OUTPUT);
  pinMode(motor2_pin2, OUTPUT);
  pinMode(motor3_pin1, OUTPUT);
  pinMode(motor3_pin2, OUTPUT);
  stopAll();
}

void loop() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    rx_buffer[rx_index] = b;

    if (rx_index == 0 && rx_buffer[0] != 0xAA) {
      rx_index = 0;
      continue;
    }
    if (rx_index == 1 && rx_buffer[1] != 0x55) {
      rx_index = 0;
      continue;
    }

    rx_index++;

    if (rx_index >= sizeof(ControlPacket)) {
      rx_index = 0;


      if (rx_buffer[sizeof(ControlPacket) - 1] != 0xFE) {
        Serial.println("Framing Error: Missing Footer");
        continue;
      }


      uint8_t calc_checksum = 0;
      for (int i = 2; i < sizeof(ControlPacket) - 2; i++) {
        calc_checksum ^= rx_buffer[i];
      }


      if (calc_checksum == rx_buffer[sizeof(ControlPacket) - 2]) {
        memcpy(&incomingData, rx_buffer, sizeof(ControlPacket));
        lastPacketTime = millis();

        int lx1 = map(incomingData.lx, -127, 127, -180, 180);
        if (abs(lx1) > 25) setMotor3(-lx1);
        else setMotor3(0);

        if (incomingData.l1) setMotor1(180);
        else if (incomingData.r1) setMotor1(-180);
        else setMotor1(0);

        int servoStep1 = 2;
        if (incomingData.l3) {
          angle2 = 147;
        } else if (incomingData.left) angle2 += servoStep1;
        else if (incomingData.right) angle2 -= servoStep1;

        angle2 = constrain(angle2, 0, 180);
        myservo2.write(angle2);

        if (incomingData.triangle) setMotor2(-255);
        else if (incomingData.cross) setMotor2(255);
        else setMotor2(0);

        int servoStep2 = 2;
        if (incomingData.up) angle3 = 146;
        else if (incomingData.down) angle3 -= servoStep2;

        angle3 = constrain(angle3, 0, 180);
        myservo3.write(angle3);

        if (incomingData.mode == 1) {
          if (incomingData.r2 > 10) angle1 += 8;
          if (incomingData.l2 > 10) angle1 -= 8;
          angle1 = constrain(angle1, 0, 180);
          myservo1.write(angle1);
        }
      } else {
        Serial.println("Data Error: Checksum Mismatch");
      }
    }
  }
  if (millis() - lastPacketTime > 500) {
    stopAll();
  }
}