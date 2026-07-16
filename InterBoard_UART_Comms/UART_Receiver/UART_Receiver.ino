// UART Protocol Integration Test - RECEIVER (espB)

// Connect espA TX(17) to espB RX(16)
// Connect espA GND to espB GND

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial2.setTimeout(5);
  
  Serial.println("--- UART TEST: RECEIVER (espB) INIT ---");
  Serial.println("Waiting for valid packets...");
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
        
        Serial.printf("SUCCESS! Valid Packet -> LX: %d | LY: %d | Triangle: %d\n", 
                      incomingData.lx, incomingData.ly, incomingData.triangle);
      } else {
        Serial.println("Data Error: Checksum Mismatch");
      }
    }
  }
}