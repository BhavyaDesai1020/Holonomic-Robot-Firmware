// UART Protocol Integration Test - SENDER (espA)

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

ControlPacket dataPacket;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  Serial.println("--- UART TEST: SENDER (espA) INIT ---");
  Serial.println("Sending test packets every 1000ms...");
}

void loop() {

  static int8_t sweep = -127;
  dataPacket.lx = sweep;
  dataPacket.ly = sweep / 2;
  dataPacket.triangle = (sweep > 0) ? 1 : 0; 
  

  dataPacket.checksum = 0;
  uint8_t *ptr = (uint8_t *)&dataPacket;
  for (int i = 2; i < sizeof(dataPacket) - 2; i++) {
    dataPacket.checksum ^= ptr[i];
  }
  

  Serial2.write((uint8_t *)&dataPacket, sizeof(dataPacket));
  

  Serial.printf("Sent Packet -> LX: %d | LY: %d | Triangle: %d | Checksum: 0x%02X\n", 
                dataPacket.lx, dataPacket.ly, dataPacket.triangle, dataPacket.checksum);
  
  sweep += 15;
  if (sweep > 127) sweep = -127;
  
  delay(1000); 
}