const int motor1_pin1 = 26, motor1_pin2 = 25;
const int motor2_pin1 = 18, motor2_pin2 = 19;
const int motor3_pin1 = 13, motor3_pin2 = 14;

int pwmValue = 0;

void setMotor1(int pwm) {
    int p = constrain(abs(pwm), 0, 255);
    if (pwm >= 0) {
        digitalWrite(motor1_pin1, HIGH);
        ledcWrite(motor1_pin2, p);
    } else {
        digitalWrite(motor1_pin1, LOW);
        ledcWrite(motor1_pin2, p);
    }
}

void setMotor2(int pwm) {
    int p = constrain(abs(pwm), 0, 255);
    if (pwm >= 0) {
        ledcWrite(motor2_pin1, p);
        ledcWrite(motor2_pin2, 0);
    } else {
        ledcWrite(motor2_pin1, 0);
        ledcWrite(motor2_pin2, p);
    }
}

void setMotor3(int pwm) {
    int p = constrain(abs(pwm), 0, 255);
    if (pwm >= 0) {
        ledcWrite(motor3_pin1, p);
        ledcWrite(motor3_pin2, 0);
    } else {
        ledcWrite(motor3_pin1, 0);
        ledcWrite(motor3_pin2, p);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(motor1_pin1, OUTPUT);

    ledcAttach(motor1_pin2, 20000, 8);

    ledcAttach(motor2_pin1, 20000, 8);
    ledcAttach(motor2_pin2, 20000, 8);

    ledcAttach(motor3_pin1, 20000, 8);
    ledcAttach(motor3_pin2, 20000, 8);

    Serial.println("Enter PWM (-255 to 255)");
}


void loop() {

    if (Serial.available()) {

        String s = Serial.readStringUntil('\n');
        s.trim();

        if (s.length() > 0) {
            pwmValue = constrain(s.toInt(), -255, 255);

            Serial.print("PWM set to: ");
            Serial.println(pwmValue);
        }
    }

    setMotor1(pwmValue);
    setMotor2(pwmValue);
    setMotor3(pwmValue);
}