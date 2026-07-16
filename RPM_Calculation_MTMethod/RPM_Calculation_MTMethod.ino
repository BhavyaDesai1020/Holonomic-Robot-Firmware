#include <Arduino.h>

const int motor1_pin = 16;
const int motor1_dir = 17;

const int enc1_A = 25;
const int enc1_B = 26;

const int TICKS_PER_REV = 1300;
const unsigned long MT_WINDOW_US = 20000;

volatile long enc1_count = 0;
volatile int enc1_dir = 0;
volatile uint8_t enc1_last_state = 0;

volatile bool mt_collecting = false;  // M-phase active
volatile bool mt_waiting_T = false;   // waiting for first tick after window
volatile bool mt_ready = false;       // M+T data ready to compute

volatile unsigned long mt_timeMstart = 0;
volatile long mt_countinM = 0;           // M part: counts in window
volatile unsigned long mt_timeforT = 0;  // T part: time from window end to next tick


double rpm_mt = 0;
bool invert1 = false;

void setMotor(int pin, int dir, double pwm) {
  int p = constrain((int)abs(pwm), 0, 255);
  digitalWrite(dir, (pwm >= 0) ? LOW : HIGH);
  analogWrite(pin, p);
}

void IRAM_ATTR enc1ISR() {
  unsigned long now = micros();

  uint8_t a = digitalRead(enc1_A);
  uint8_t b = digitalRead(enc1_B);
  uint8_t state = (a << 1) | b;

  int8_t d = 0;

  // Forward transitions
  if ((enc1_last_state == 0b00 && state == 0b01) || (enc1_last_state == 0b01 && state == 0b11) || (enc1_last_state == 0b11 && state == 0b10) || (enc1_last_state == 0b10 && state == 0b00)) {
    d = +1;
  }
  // Reverse transitions
  else if ((enc1_last_state == 0b00 && state == 0b10) || (enc1_last_state == 0b10 && state == 0b11) || (enc1_last_state == 0b11 && state == 0b01) || (enc1_last_state == 0b01 && state == 0b00)) {
    d = -1;
  }

  if (d != 0) {
    enc1_count += d;
    enc1_dir = d;

    if (mt_collecting) {
      mt_countinM++;
    } else if (mt_waiting_T) {

      unsigned long window_end = mt_timeMstart + MT_WINDOW_US;
      if (now > window_end) {
        mt_timeforT = now - window_end;
      } else {
        mt_timeforT = 1;
      }
      mt_waiting_T = false;
      mt_ready = true;
    }
  }

  enc1_last_state = state;
}

void update_rpm_MT() {
  unsigned long now = micros();
  if (!mt_collecting && !mt_waiting_T && !mt_ready) {
    noInterrupts();
    mt_collecting = true;
    mt_countinM = 0;
    mt_timeMstart = now;
    interrupts();
    return;
  }
  if (mt_collecting && (now - mt_timeMstart) >= MT_WINDOW_US) {
    noInterrupts();
    mt_collecting = false;
    mt_waiting_T = true;
    interrupts();
    return;
  }
  if (mt_waiting_T) {

    unsigned long window_end = mt_timeMstart + MT_WINDOW_US;


    if (now > window_end && (now - window_end) > 100000) {
      noInterrupts();
      mt_waiting_T = false;
      mt_collecting = false;
      mt_ready = false;
      interrupts();

      rpm_mt = 0;
    }
  }
  if (mt_ready) {
    noInterrupts();
    long count = mt_countinM;
    unsigned long T_dt = mt_timeforT;
    long dir = enc1_dir;


    mt_ready = false;
    mt_waiting_T = false;
    mt_collecting = false;
    interrupts();

    if (count == 0) {
      rpm_mt = 0;
      return;
    }
    double total_time_us = (double)MT_WINDOW_US + (double)T_dt;
    double revs = (double)count / (double)TICKS_PER_REV;
    double time_sec = total_time_us / 1e6;

    double rpm = (revs / time_sec) * 60.0;

    if (invert1) rpm = -rpm;
    if (dir < 0) rpm = -fabs(rpm);
    rpm_mt = rpm;
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(motor1_pin, OUTPUT);
  pinMode(motor1_dir, OUTPUT);

  pinMode(enc1_A, INPUT_PULLUP);
  pinMode(enc1_B, INPUT_PULLUP);

  enc1_last_state = (digitalRead(enc1_A) << 1) | digitalRead(enc1_B);

  attachInterrupt(digitalPinToInterrupt(enc1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1_B), enc1ISR, CHANGE);
}


void loop() {
  static unsigned long lastPWMUpdate = 0;
  static unsigned long lastPrint = 0;
  static int PWM = 0;


  if (millis() - lastPWMUpdate > 300) {
    lastPWMUpdate = millis();
    PWM++;
    if (PWM > 255) PWM = 255;
  }


  update_rpm_MT();
  setMotor(motor1_pin, motor1_dir, PWM);

  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.print(PWM);
    Serial.print(" ");
    Serial.println(rpm_mt);
  }
}