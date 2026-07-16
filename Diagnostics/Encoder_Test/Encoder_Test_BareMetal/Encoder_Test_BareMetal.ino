
const int enc1_A = 39;
const int enc1_B = 36;

const int enc2_A = 33;
const int enc2_B = 32;

const int enc3_A = 35;
const int enc3_B = 34;

volatile long enc1_count = 0;
volatile long enc2_count = 0;
volatile long enc3_count = 0;

volatile int enc1_dir = 0;
volatile int enc2_dir = 0;
volatile int enc3_dir = 0;

volatile uint8_t enc1_last_state = 0;
volatile uint8_t enc2_last_state = 0;
volatile uint8_t enc3_last_state = 0;

void IRAM_ATTR enc1ISR()
{
    uint8_t state = (digitalRead(enc1_A) << 1) | digitalRead(enc1_B);

    int8_t delta = 0;

    if ((enc1_last_state == 0 && state == 1) ||
        (enc1_last_state == 1 && state == 3) ||
        (enc1_last_state == 3 && state == 2) ||
        (enc1_last_state == 2 && state == 0))
    {
        delta = 1;
    }
    else if ((enc1_last_state == 0 && state == 2) ||
             (enc1_last_state == 2 && state == 3) ||
             (enc1_last_state == 3 && state == 1) ||
             (enc1_last_state == 1 && state == 0))
    {
        delta = -1;
    }

    enc1_last_state = state;

    if (delta != 0)
    {
        enc1_count += delta;
        enc1_dir = delta;
    }
}

void IRAM_ATTR enc2ISR()
{
    uint8_t state = (digitalRead(enc2_A) << 1) | digitalRead(enc2_B);

    int8_t delta = 0;

    if ((enc2_last_state == 0 && state == 1) ||
        (enc2_last_state == 1 && state == 3) ||
        (enc2_last_state == 3 && state == 2) ||
        (enc2_last_state == 2 && state == 0))
    {
        delta = 1;
    }
    else if ((enc2_last_state == 0 && state == 2) ||
             (enc2_last_state == 2 && state == 3) ||
             (enc2_last_state == 3 && state == 1) ||
             (enc2_last_state == 1 && state == 0))
    {
        delta = -1;
    }

    enc2_last_state = state;

    if (delta != 0)
    {
        enc2_count += delta;
        enc2_dir = delta;
    }
}

void IRAM_ATTR enc3ISR()
{
    uint8_t state = (digitalRead(enc3_A) << 1) | digitalRead(enc3_B);

    int8_t delta = 0;

    if ((enc3_last_state == 0 && state == 1) ||
        (enc3_last_state == 1 && state == 3) ||
        (enc3_last_state == 3 && state == 2) ||
        (enc3_last_state == 2 && state == 0))
    {
        delta = 1;
    }
    else if ((enc3_last_state == 0 && state == 2) ||
             (enc3_last_state == 2 && state == 3) ||
             (enc3_last_state == 3 && state == 1) ||
             (enc3_last_state == 1 && state == 0))
    {
        delta = -1;
    }

    enc3_last_state = state;

    if (delta != 0)
    {
        enc3_count += delta;
        enc3_dir = delta;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(enc1_A, INPUT_PULLUP);
    pinMode(enc1_B, INPUT_PULLUP);

    pinMode(enc2_A, INPUT_PULLUP);
    pinMode(enc2_B, INPUT_PULLUP);

    pinMode(enc3_A, INPUT_PULLUP);
    pinMode(enc3_B, INPUT_PULLUP);

    enc1_last_state = (digitalRead(enc1_A) << 1) | digitalRead(enc1_B);
    enc2_last_state = (digitalRead(enc2_A) << 1) | digitalRead(enc2_B);
    enc3_last_state = (digitalRead(enc3_A) << 1) | digitalRead(enc3_B);

    attachInterrupt(digitalPinToInterrupt(enc1_A), enc1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc1_B), enc1ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(enc2_A), enc2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc2_B), enc2ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(enc3_A), enc3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc3_B), enc3ISR, CHANGE);

    Serial.println("3-Wheel Encoder Tick Counter");
}

void loop()
{
    noInterrupts();

    long c1 = enc1_count;
    long c2 = enc2_count;
    long c3 = enc3_count;

    int d1 = enc1_dir;
    int d2 = enc2_dir;
    int d3 = enc3_dir;

    interrupts();

    Serial.print("Encoder 1: ");
    Serial.print(c1);
    Serial.print("  ");

    Serial.print("\tEncoder 2: ");
    Serial.print(c2);
    Serial.print("  ");

    Serial.print("\tEncoder 3: ");
    Serial.print(c3);
    Serial.print("  ");

    delay(100);
}
