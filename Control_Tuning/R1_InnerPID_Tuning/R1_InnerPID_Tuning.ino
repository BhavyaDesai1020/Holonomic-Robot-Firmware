#include <PS4Controller.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "Config.h" // Pulls in all pins and constants

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

ControlPacket dataPacket;
bool remoteMode = false;
bool lastShareState = false;
bool option_pin_state = false;
bool robot_ready = false;

#if ENABLE_IMU
  Adafruit_BNO08x bno08x(-1);
  sh2_SensorValue_t sensorValue;
  bool imu_initialized = false;
  unsigned long last_imu_time = 0;
#endif

bool field_centric_mode = true;
float field_offset = 0;
float current_heading = 0;
float target_heading = 0;

float sp1_r = 0, sp2_r = 0, sp3_r = 0;

struct MotorState {
    volatile bool collecting = false;
    volatile bool waiting_T = false;
    volatile bool ready = false;
    volatile unsigned long timeMstart = 0;
    volatile unsigned long timeforT = 0;
    volatile long countinM = 0;
    float rpm_raw = 0.0f;
    float rpm_filtered = 0.0f;
};

MotorState m1, m2, m3;

volatile long enc1_count = 0, enc2_count = 0, enc3_count = 0;
volatile int enc1_dir = 0, enc2_dir = 0, enc3_dir = 0;
volatile uint8_t enc1_last_state = 0, enc2_last_state = 0, enc3_last_state = 0;

class PIDController {
public:
  float Kp, Ki, Kd;
  float integral = 0, prev_error = 0;
  unsigned long last_t = 0;
  float i_max;
  
  PIDController(float p, float i, float d) {
    Kp = p; Ki = i; Kd = d;
    i_max = (Ki != 0) ? 100.0f / Ki : 0;
    last_t = micros();
  }
  
  void reset() {
    integral = 0; prev_error = 0; last_t = micros();
  }
  
  float compute(float sp, float act) {
    unsigned long now = micros();
    unsigned long dt_us = now - last_t; 
    float dt = dt_us / 1000000.0f;
    if (dt <= 0.0f) dt = 0.000001f;
    last_t = now;

    float err = sp - act;
    integral += err * dt;
    if (integral > i_max) integral = i_max;
    if (integral < -i_max) integral = -i_max;

    float derivative = (err - prev_error) / dt;
    prev_error = err;

    float output = Kp * err + Ki * integral + Kd * derivative;
    return constrain(output, -255, 255);
  }
};

PIDController pid1(0.55, 0.5, 0);
PIDController pid2(0.55, 0.5, 0);
PIDController pid3(0.45, 0.5, 0);

void IRAM_ATTR enc1ISR() {
  uint8_t state = (digitalRead(enc1_A) << 1) | digitalRead(enc1_B);
  int8_t d = 0;
  if ((enc1_last_state == 0 && state == 1) || (enc1_last_state == 1 && state == 3) || (enc1_last_state == 3 && state == 2) || (enc1_last_state == 2 && state == 0)) d = 1;
  else if ((enc1_last_state == 0 && state == 2) || (enc1_last_state == 2 && state == 3) || (enc1_last_state == 3 && state == 1) || (enc1_last_state == 1 && state == 0)) d = -1;
  enc1_last_state = state;
  if (d != 0) { enc1_count += d; enc1_dir = d; }
  
  unsigned long now = micros();
  if (m1.collecting) m1.countinM += d != 0 ? 1 : 0;
  else if (m1.waiting_T && d != 0) {
    m1.timeforT = (now > m1.timeMstart + MT_WINDOW_US) ? (now - (m1.timeMstart + MT_WINDOW_US)) : 1;
    m1.waiting_T = false; m1.ready = true;
  }
}

void IRAM_ATTR enc2ISR() {
  uint8_t state = (digitalRead(enc2_A) << 1) | digitalRead(enc2_B);
  int8_t d = 0;
  if ((enc2_last_state == 0 && state == 1) || (enc2_last_state == 1 && state == 3) || (enc2_last_state == 3 && state == 2) || (enc2_last_state == 2 && state == 0)) d = 1;
  else if ((enc2_last_state == 0 && state == 2) || (enc2_last_state == 2 && state == 3) || (enc2_last_state == 3 && state == 1) || (enc2_last_state == 1 && state == 0)) d = -1;
  enc2_last_state = state;
  if (d != 0) { enc2_count += d; enc2_dir = d; }
  
  unsigned long now = micros();
  if (m2.collecting) m2.countinM += d != 0 ? 1 : 0;
  else if (m2.waiting_T && d != 0) {
    m2.timeforT = (now > m2.timeMstart + MT_WINDOW_US) ? (now - (m2.timeMstart + MT_WINDOW_US)) : 1;
    m2.waiting_T = false; m2.ready = true;
  }
}

void IRAM_ATTR enc3ISR() {
  uint8_t state = (digitalRead(enc3_A) << 1) | digitalRead(enc3_B);
  int8_t d = 0;
  if ((enc3_last_state == 0 && state == 1) || (enc3_last_state == 1 && state == 3) || (enc3_last_state == 3 && state == 2) || (enc3_last_state == 2 && state == 0)) d = 1;
  else if ((enc3_last_state == 0 && state == 2) || (enc3_last_state == 2 && state == 3) || (enc3_last_state == 3 && state == 1) || (enc3_last_state == 1 && state == 0)) d = -1;
  enc3_last_state = state;
  if (d != 0) { enc3_count += d; enc3_dir = d; }
  
  unsigned long now = micros();
  if (m3.collecting) m3.countinM += d != 0 ? 1 : 0;
  else if (m3.waiting_T && d != 0) {
    m3.timeforT = (now > m3.timeMstart + MT_WINDOW_US) ? (now - (m3.timeMstart + MT_WINDOW_US)) : 1;
    m3.waiting_T = false; m3.ready = true;
  }
}

void calculateMotorRPM(MotorState &m, volatile int &dir_flag, unsigned long now) {
  if (!m.collecting && !m.waiting_T && !m.ready) {
    noInterrupts(); m.collecting = true; m.countinM = 0; m.timeMstart = now; interrupts();
  } else if (m.collecting && (now - m.timeMstart) >= MT_WINDOW_US) {
    noInterrupts(); m.collecting = false; m.waiting_T = true; interrupts();
  } else if (m.waiting_T && (now > (m.timeMstart + MT_WINDOW_US)) && (now - (m.timeMstart + MT_WINDOW_US) > TIMEOUT_US)) {
    noInterrupts(); m.waiting_T = false; m.collecting = false; m.ready = false; interrupts(); m.rpm_raw = 0;
  }
  
  if (m.ready) {
    noInterrupts(); long c = m.countinM; unsigned long t = m.timeforT; int d = dir_flag;
    m.ready = false; m.waiting_T = false; m.collecting = false; interrupts();
    if (c != 0) {
      m.rpm_raw = ((float)c / TICKS_PER_REV) * (60.0f / ((MT_WINDOW_US + t) / 1e6f));
      if (d < 0) m.rpm_raw = -fabsf(m.rpm_raw);
    } else { m.rpm_raw = 0; }
  }
  m.rpm_filtered = (FILTER_ALPHA * m.rpm_raw) + ((1.0f - FILTER_ALPHA) * m.rpm_filtered);
}

void updateRPMs() {
  unsigned long now = micros();
  calculateMotorRPM(m1, enc1_dir, now);
  calculateMotorRPM(m2, enc2_dir, now);
  calculateMotorRPM(m3, enc3_dir, now);
}

void setMotor1(int pwm) {
  int p = constrain(abs(pwm), 0, 255);
  if (pwm >= 0) { digitalWrite(motor1_pin1, HIGH); ledcWrite(motor1_pin2, p); } 
  else { digitalWrite(motor1_pin1, LOW); ledcWrite(motor1_pin2, p); }
}

void setMotor2(int pwm) {
  int p = constrain(abs(pwm), 0, 255);
  if (pwm >= 0) { ledcWrite(motor2_pin1, p); ledcWrite(motor2_pin2, 0); } 
  else { ledcWrite(motor2_pin1, 0); ledcWrite(motor2_pin2, p); }
}

void setMotor3(int pwm) {
  int p = constrain(abs(pwm), 0, 255);
  if (pwm >= 0) { ledcWrite(motor3_pin1, p); ledcWrite(motor3_pin2, 0); } 
  else { ledcWrite(motor3_pin1, 0); ledcWrite(motor3_pin2, p); }
}

void setAllMotors(int p1, int p2, int p3) { setMotor1(p1); setMotor2(p2); setMotor3(p3); }

void emergencyStop() {
  robot_ready = false; sp1_r = 0; sp2_r = 0; sp3_r = 0;
  pid1.reset(); pid2.reset(); pid3.reset();
  setAllMotors(0, 0, 0); digitalWrite(light, LOW);
}

void handleSerialTuning() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    char tag[10]; 
    float value;

    if (sscanf(cmd.c_str(), "%s %f", tag, &value) < 2) return;

    if (cmd.startsWith("P1")) pid1.Kp = value;
    else if (cmd.startsWith("I1")) {
        pid1.Ki = value;
        pid1.i_max = (value != 0) ? 100.0f / value : 0;
    }
    else if (cmd.startsWith("D1")) pid1.Kd = value;
    else if (cmd.startsWith("M1")) FF1 = value; 

    else if (cmd.startsWith("P2")) pid2.Kp = value;
    else if (cmd.startsWith("I2")) {
        pid2.Ki = value;
        pid2.i_max = (value != 0) ? 100.0f / value : 0;
    }
    else if (cmd.startsWith("D2")) pid2.Kd = value;
    else if (cmd.startsWith("M2")) FF2 = value;

    else if (cmd.startsWith("P3")) pid3.Kp = value;
    else if (cmd.startsWith("I3")) {
        pid3.Ki = value;
        pid3.i_max = (value != 0) ? 100.0f / value : 0;
    }
    else if (cmd.startsWith("D3")) pid3.Kd = value;
    else if (cmd.startsWith("M3")) FF3 = value;

    Serial.printf(
        "M1(P=%.3f I=%.3f D=%.3f FF=%d) | "
        "M2(P=%.3f I=%.3f D=%.3f FF=%d) | "
        "M3(P=%.3f I=%.3f D=%.3f FF=%d)\n",
        pid1.Kp, pid1.Ki, pid1.Kd, (int)FF1,
        pid2.Kp, pid2.Ki, pid2.Kd, (int)FF2,
        pid3.Kp, pid3.Ki, pid3.Kd, (int)FF3
    );
}

void calculate_kinematics(int x, int y, int w, int &w1, int &w2, int &w3) {
  const float A = 2.0f / 3.0f;
  const float B = 1.0f / 3.0f;
  const float C = 1.0f / sqrtf(3.0f);
  w2 = (int)((A * x + B * w * WHEEL_TO_CENTRE) * JOY_TO_RPM);
  w3 = (int)((-C * y - B * x + B * w * WHEEL_TO_CENTRE) * JOY_TO_RPM);
  w1 = (int)((C * y - B * x + B * w * WHEEL_TO_CENTRE) * JOY_TO_RPM);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  PS4.begin();
  pinMode(light, OUTPUT); digitalWrite(light, LOW);
  delay(1000);
  
#if ENABLE_IMU
  if (!bno08x.begin_I2C()) {
    Serial.println("IMU Fail - Driving in Robot-Centric Mode");
    imu_initialized = false; field_centric_mode = false;
  } else {
    Serial.println("IMU OK");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    delay(500); last_imu_time = millis(); imu_initialized = true;
  }
#else
  field_centric_mode = false;
#endif

  pinMode(motor1_pin1, OUTPUT); ledcAttach(motor1_pin2, 20000, 8);
  ledcAttach(motor2_pin1, 20000, 8); ledcAttach(motor2_pin2, 20000, 8);
  ledcAttach(motor3_pin1, 20000, 8); ledcAttach(motor3_pin2, 20000, 8);
  
  pinMode(arm, OUTPUT); digitalWrite(arm, LOW);
  
  pinMode(enc1_A, INPUT_PULLUP); pinMode(enc1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1_A), enc1ISR, CHANGE); attachInterrupt(digitalPinToInterrupt(enc1_B), enc1ISR, CHANGE);
  pinMode(enc2_A, INPUT_PULLUP); pinMode(enc2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2_A), enc2ISR, CHANGE); attachInterrupt(digitalPinToInterrupt(enc2_B), enc2ISR, CHANGE);
  pinMode(enc3_A, INPUT_PULLUP); pinMode(enc3_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc3_A), enc3ISR, CHANGE); attachInterrupt(digitalPinToInterrupt(enc3_B), enc3ISR, CHANGE);
}

unsigned long lastPS4Time = 0;

void loop() {
  handleSerialTuning();
  updateRPMs();

#if ENABLE_IMU
  if (imu_initialized) {
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        last_imu_time = millis();
        float qr = sensorValue.un.gameRotationVector.real;
        float qi = sensorValue.un.gameRotationVector.i;
        float qj = sensorValue.un.gameRotationVector.j;
        float qk = sensorValue.un.gameRotationVector.k;
        float siny_cosp = 2.0f * (qr * qk + qi * qj);
        float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
        current_heading = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
      }
    }
    if (millis() - last_imu_time > IMU_TIMEOUT_MS) field_centric_mode = false;
  }
#endif

  if (!PS4.isConnected() || millis() - lastPS4Time > PS4_TIMEOUT) {
    emergencyStop();
  }

  static unsigned long lastControl = 0;
  static bool square_pressed = false, circle_pressed = false, option_pressed = false, touch_pressed = false;
  static bool precision_mode = false;
  
  if (millis() - lastControl < 30) return;
  lastControl = millis();

  int x = 0, y = 0, w = 0;

  if (PS4.isConnected()) {
    bool currentShareState = PS4.Share();
    if (currentShareState && !lastShareState) {
      remoteMode = !remoteMode; robot_ready = false; square_pressed = false; lastPS4Time = millis();
      pid1.reset(); pid2.reset(); pid3.reset(); sp1_r = 0; sp2_r = 0; sp3_r = 0;
      setAllMotors(0, 0, 0); digitalWrite(light, LOW);
      Serial.println(remoteMode ? "MODE: REMOTE (espB)" : "MODE: LOCAL (espA)");
    }
    lastShareState = currentShareState;

    dataPacket.lx = PS4.LStickX(); dataPacket.ly = PS4.LStickY();
    dataPacket.rx = PS4.RStickX(); dataPacket.ry = PS4.RStickY();
    dataPacket.l2 = PS4.L2Value(); dataPacket.r2 = PS4.R2Value();
    dataPacket.l1 = PS4.L1();      dataPacket.r1 = PS4.R1();
    dataPacket.square = PS4.Square(); dataPacket.circle = PS4.Circle();
    dataPacket.triangle = PS4.Triangle(); dataPacket.cross = PS4.Cross();
    dataPacket.up = PS4.Up();      dataPacket.down = PS4.Down();
    dataPacket.right = PS4.Right(); dataPacket.left = PS4.Left();
    dataPacket.l3 = PS4.L3();      dataPacket.touchpad = PS4.Touchpad();
    dataPacket.mode = remoteMode ? 1 : 0;

    dataPacket.checksum = 0;
    uint8_t *ptr = (uint8_t *)&dataPacket;
    for (int i = 2; i < sizeof(dataPacket) - 2; i++) dataPacket.checksum ^= ptr[i];
    
    Serial2.write((uint8_t *)&dataPacket, sizeof(dataPacket));
    lastPS4Time = millis();

    if (remoteMode) {
      robot_ready = false; sp1_r = 0; sp2_r = 0; sp3_r = 0;
      setAllMotors(0, 0, 0); digitalWrite(light, (millis() / 500) % 2);
    } else {
      if (PS4.Square() && !square_pressed) {
        robot_ready = !robot_ready;
        if (!robot_ready) {
          pid1.reset(); pid2.reset(); pid3.reset(); sp1_r = 0; sp2_r = 0; sp3_r = 0;
          setMotor1(0); setMotor2(0); setMotor3(0); digitalWrite(light, LOW);
        } else { target_heading = current_heading; digitalWrite(light, HIGH); }
        square_pressed = true;
      } else if (!PS4.Square()) square_pressed = false;

      if (robot_ready) {
        digitalWrite(light, HIGH);
        int L2 = PS4.L2Value(), R2 = PS4.R2Value();

        if (PS4.Circle() && !circle_pressed) { field_centric_mode = !field_centric_mode; circle_pressed = true; } 
        else if (!PS4.Circle()) circle_pressed = false;
        
        digitalWrite(light, field_centric_mode ? HIGH : LOW);

        if (PS4.Options() && !option_pressed) {
          option_pin_state = !option_pin_state; digitalWrite(arm, option_pin_state ? LOW : HIGH); option_pressed = true;
        } else if (!PS4.Options()) option_pressed = false;

        if (PS4.Touchpad() && !touch_pressed) { precision_mode = !precision_mode; touch_pressed = true; } 
        else if (!PS4.Touchpad()) touch_pressed = false;

        if (PS4.R3()) field_offset = current_heading;

        if (L2 > 10 || R2 > 10) { w = (L2 - R2) / 2; target_heading = current_heading; } 
        else {
          float error = target_heading - current_heading;
          if (error > 180.0f) error -= 360.0f;
          if (error < -180.0f) error += 360.0f;
          if (fabsf(error) > HEADING_DEADZONE) {
            int raw_w = (int)(error * HEADING_KP * ROTATION_DIR);
            w = constrain(raw_w, -JOYSTICK_LIMIT, JOYSTICK_LIMIT);
          } else { w = 0; }
        }

        x = PS4.RStickX(); y = PS4.RStickY();
        float r = sqrtf((float)x * x + (float)y * y);
        float theta = atan2f((float)y, (float)x);
        float theta_deg = theta * 180.0f / PI;
        if ((theta_deg < 95 && theta_deg > 85) || (theta_deg > -95 && theta_deg < -85)) theta = (theta_deg > 0) ? (PI / 2.0f) : (-PI / 2.0f);
        
        if (r < 15) { x = 0; y = 0; } 
        else {
          float rnew = r - 15;
          x = (int)(rnew * cosf(theta)); y = (int)(rnew * sinf(theta));
        }
        
#if ENABLE_IMU
        if (field_centric_mode) {
          float adjusted_heading = current_heading - field_offset;
          float rad = adjusted_heading * PI / 180.0f;
          float cosA = cosf(rad); float sinA = sinf(rad);
          int x_robot = (int)(x * cosA + y * sinA);
          int y_robot = (int)(-x * sinA + y * cosA);
          x = x_robot; y = y_robot;
        }
#endif

        if (precision_mode) {
          x = (int)(x * PRECISION_MULTIPLIER); y = (int)(y * PRECISION_MULTIPLIER); w = (int)(w * PRECISION_MULTIPLIER);
        }
        
        int w1_sp, w2_sp, w3_sp;
        calculate_kinematics(x, y, w, w1_sp, w2_sp, w3_sp);

        float max_requested = max(abs(w1_sp), max(abs(w2_sp), abs(w3_sp)));
        if (max_requested > MAX_RPM_REQUIRED) {
          float scale = MAX_RPM_REQUIRED / max_requested;
          w1_sp = (int)(w1_sp * scale); w2_sp = (int)(w2_sp * scale); w3_sp = (int)(w3_sp * scale);
        }

        sp1_r += constrain(w1_sp - sp1_r, -current_step, current_step);
        sp2_r += constrain(w2_sp - sp2_r, -current_step, current_step);
        sp3_r += constrain(w3_sp - sp3_r, -current_step, current_step);
        
        float pwm1 = 0, pwm2 = 0, pwm3 = 0;

#if ENABLE_PID
        if (w1_sp == 0 && abs(sp1_r) < 1.0f) { sp1_r = 0; pid1.reset(); pwm1 = 0; } 
        else { pwm1 = pid1.compute(sp1_r, m1.rpm_filtered); if (abs(sp1_r) > 1.0f) pwm1 += FF1 * (sp1_r > 0 ? 1 : -1); }
        
        if (w2_sp == 0 && abs(sp2_r) < 1.0f) { sp2_r = 0; pid2.reset(); pwm2 = 0; } 
        else { pwm2 = pid2.compute(sp2_r, m2.rpm_filtered); if (abs(sp2_r) > 1.0f) pwm2 += FF2 * (sp2_r > 0 ? 1 : -1); }
        
        if (w3_sp == 0 && abs(sp3_r) < 1.0f) { sp3_r = 0; pid3.reset(); pwm3 = 0; } 
        else { pwm3 = pid3.compute(sp3_r, m3.rpm_filtered); if (abs(sp3_r) > 1.0f) pwm3 += FF3 * (sp3_r > 0 ? 1 : -1); }
#else
        pwm1 = map(sp1_r, -MAX_RPM_REQUIRED, MAX_RPM_REQUIRED, -255, 255);
        pwm2 = map(sp2_r, -MAX_RPM_REQUIRED, MAX_RPM_REQUIRED, -255, 255);
        pwm3 = map(sp3_r, -MAX_RPM_REQUIRED, MAX_RPM_REQUIRED, -255, 255);
        if (abs(sp1_r) > 1.0f) pwm1 += FF1 * (sp1_r > 0 ? 1 : -1);
        if (abs(sp2_r) > 1.0f) pwm2 += FF2 * (sp2_r > 0 ? 1 : -1);
        if (abs(sp3_r) > 1.0f) pwm3 += FF3 * (sp3_r > 0 ? 1 : -1);
#endif

        pwm1 = constrain(pwm1, -255, 255); pwm2 = constrain(pwm2, -255, 255); pwm3 = constrain(pwm3, -255, 255);
        setMotor1((int)pwm1); setMotor2((int)pwm2); setMotor3((int)pwm3);
        Serial.print("SP1:");
Serial.print(sp1_r);
Serial.print(",");

Serial.print("RPM1:");
Serial.print(m1.rpm_filtered);
Serial.print(",");

Serial.print("PWM1:");
Serial.print(pwm1);
Serial.print(",");

Serial.print("SP2:");
Serial.print(sp2_r);
Serial.print(",");

Serial.print("RPM2:");
Serial.print(m2.rpm_filtered);
Serial.print(",");

Serial.print("PWM2:");
Serial.print(pwm2);
Serial.print(",");

Serial.print("SP3:");
Serial.print(sp3_r);
Serial.print(",");

Serial.print("RPM3:");
Serial.println(m3.rpm_filtered);

Serial.print("PWM3:");
Serial.print(pwm3);
Serial.print(",");
      } else {
        digitalWrite(light, (millis() / 200) % 2 == 0 ? HIGH : LOW);
        setAllMotors(0, 0, 0);
      }
    }
  }
}