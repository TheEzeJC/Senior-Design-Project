// Main 74HC595
#define DATAPIN 42   // DS
#define LATCHPIN 2  // ST_CP
#define CLKPIN 1  // SH_CP

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  // 74HC595
  pinMode(DATAPIN, OUTPUT);
  pinMode(LATCHPIN, OUTPUT);
  pinMode(CLKPIN, OUTPUT);
}

// Set everything to 0
uint8_t Byte1[8] = {0};
uint8_t Byte2[8] = {0};
uint8_t Byte3[8] = {0};
uint8_t packedByte1 = 0b00000000;
uint8_t packedByte2 = 0b00000000;
uint8_t packedByte3 = 0b00000000;

void loop() {
  Serial.println("Start");
  //uint8_t exampleData[8] = {1,1,1,1,1,1,1,1};
  //uint8_t exampleData[8] = {0,0,0,0,0,0,0,0};
  //uint8_t exampleData[8] = {1,0,1,0,1,0,1,0};
  uint8_t exampleData[8] = {0,0,0,0,1,1,1,1};

  // Write Data To Register
  for (int i = 0; i < 8; i++) {
    // Set Data bit
    Byte2[1] = exampleData[i];  // Set DATA line
    Byte2[4] = exampleData[i];  // Set DATA line
    Byte2[7] = exampleData[i];  // Set DATA line

    // Send Latch LOW at end
    Byte2[2] = 0; // Latch LOW
    Byte2[5] = 0; // Latch LOW
    Byte3[0] = 0; // Latch LOW
    packedByte2 = packBits(Byte2);
    packedByte3 = packBits(Byte3);
    // Send LATCH Position To Main Shiters To Control Secondary
    digitalWrite(LATCHPIN, LOW);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
    digitalWrite(LATCHPIN, HIGH);

    // Pulse
    pulseClock();

    // Send Latch HIGH at end
    Byte2[2] = 1; // Latch HIGH
    Byte2[5] = 1; // Latch HIGH
    Byte3[0] = 1; // Latch HIGH
    packedByte2 = packBits(Byte2);
    packedByte3 = packBits(Byte3);
    // Send LATCH Position To Main Shiters To Control Secondary
    digitalWrite(LATCHPIN, LOW);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
    digitalWrite(LATCHPIN, HIGH);

    //pulse
    pulseClock();
  }

  Serial.println("Data Sent");
  delay(4000);

  Serial.println("Shifting Test");
  // Shift 2 times
  for (int i = 0; i < 4; i++) {
    // Send Latch HIGH at end
    Byte2[2] = 0; // Latch HIGH
    Byte2[5] = 0; // Latch HIGH
    Byte3[0] = 0; // Latch HIGH
    packedByte2 = packBits(Byte2);
    packedByte3 = packBits(Byte3);
    // Send LATCH Position To Main Shiters To Control Secondary
    digitalWrite(LATCHPIN, LOW);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
    digitalWrite(LATCHPIN, HIGH);

    // Pulse
    pulseClockData(0);

    // Send Latch HIGH at end
    Byte2[2] = 1; // Latch HIGH
    Byte2[5] = 1; // Latch HIGH
    Byte3[0] = 1; // Latch HIGH
    packedByte2 = packBits(Byte2);
    packedByte3 = packBits(Byte3);
    // Send LATCH Position To Main Shiters To Control Secondary
    digitalWrite(LATCHPIN, LOW);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
    shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
    digitalWrite(LATCHPIN, HIGH);

    //pulse
    pulseClockData(0);
  }
  Serial.println("Done Moving");
  delay(4000);
}

// Convert Bit Array to Byte
uint8_t packBits(uint8_t bits[8]) {
  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    result |= (bits[i] & 0x01) << i;  // shift bit to correct place
  }
  return result;
}

// Control Secondary Clock Pulse
// Shifts All Data at once
void pulseClock() {
  Byte2[3] = 1;  // Clock HIGH
  Byte2[6] = 1;  // Clock HIGH
  Byte3[1] = 1;  // Clock HIGH
  packedByte2 = packBits(Byte2);
  packedByte3 = packBits(Byte3);
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
  digitalWrite(LATCHPIN, HIGH);

  // Send LOW Clock Position To Main Shiters To Control Secondary
  Byte2[3] = 0;  // Clock LOW
  Byte2[6] = 0;  // Clock LOW
  Byte3[1] = 0;  // Clock LOW
  packedByte2 = packBits(Byte2);
  packedByte3 = packBits(Byte3);
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
  digitalWrite(LATCHPIN, HIGH);
}

// Control Secondary Clock Pulse by sending 0
// Shifts only one data at once
void pulseClockData(int color) {
  // Set Data bit
  Byte2[1] = 0;  // Set DATA line
  Byte2[4] = 0;  // Set DATA line
  Byte2[7] = 0;  // Set DATA line

  if (color == 1) {
    Byte2[3] = 1;  // Clock HIGH RED
  } else if (color == 2) {
    Byte2[6] = 1;  // Clock HIGH GREEN
  } else if (color == 3) {
    Byte3[1] = 1;  // Clock HIGH BLUE
  } else {
    Byte2[3] = 1;  // Clock HIGH RED
    Byte2[6] = 1;  // Clock HIGH GREEN
    Byte3[1] = 1;  // Clock HIGH BLUE
  }

  packedByte2 = packBits(Byte2);
  packedByte3 = packBits(Byte3);
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
  digitalWrite(LATCHPIN, HIGH);

  // Send LOW Clock Position To Main Shiters To Control Secondary
  Byte2[3] = 0;  // Clock LOW
  Byte2[6] = 0;  // Clock LOW
  Byte3[1] = 0;  // Clock LOW
  packedByte2 = packBits(Byte2);
  packedByte3 = packBits(Byte3);
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
  digitalWrite(LATCHPIN, HIGH);
}