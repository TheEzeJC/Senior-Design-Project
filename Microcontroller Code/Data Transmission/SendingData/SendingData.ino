#define A0 36
#define B0 37
#define C0 38
#define D0 39
#define E0 40
#define F0 41
#define G0 42
#define H0 2

#define A1 19
#define B1 20
#define C1 21
#define D1 47
#define E1 48
#define F1 45
#define G1 0
#define H1 35

int dataSize = 8;
uint8_t pixelData[8] = {1, 0, 1, 0, 1, 0, 1, 0};

// Pins connected to 74HC595
int dataPin = 4;   // DS
int latchPin = 5;  // ST_CP
int clockPin = 6;  // SH_CP

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Set input selector pins
  pinMode(A0, INPUT);
  pinMode(B0, INPUT);
  pinMode(C0, INPUT);
  pinMode(D0, INPUT);
  pinMode(E0, INPUT);
  pinMode(F0, INPUT);
  pinMode(G0, INPUT);
  pinMode(H0, INPUT);

  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  pinMode(C1, INPUT);
  pinMode(D1, INPUT);
  pinMode(E1, INPUT);
  pinMode(F1, INPUT);
  pinMode(G1, INPUT);
  pinMode(H1, INPUT);
}

void loop() {
  // // Convert pixelData array into a single byte
  // uint8_t pixelByte = 0;
  // for (int i = 0; i < dataSize; i++) {
  //   pixelByte |= (pixelData[i] & 0x01) << (7 - i);  // MSB first
  // }

// Split the 16-bit pattern into two 8-bit bytes
//uint8_t upperByte = 0b00000000; // goes to second 595 (A1–H1)
uint8_t lowerByte = 0b00000000; // goes to first 595 (A0–H0)

// Shift out upper first, then lower (for MSB-first)
digitalWrite(latchPin, LOW);
shiftOut(dataPin, clockPin, LSBFIRST, upperByte);  // goes into second 595
shiftOut(dataPin, clockPin, LSBFIRST, lowerByte);  // goes into first 595
digitalWrite(latchPin, HIGH);

  // Display the states of input pins
  Serial.printf("Inputs FOR 1: %d%d%d%d%d%d%d%d\n",
                digitalRead(A0), digitalRead(B0), digitalRead(C0), digitalRead(D0),
                digitalRead(E0), digitalRead(F0), digitalRead(G0), digitalRead(H0));

  // Serial.printf("Inputs FOR 2: %d%d%d%d%d%d%d%d\n",
  //               digitalRead(A1), digitalRead(B1), digitalRead(C1), digitalRead(D1),
  //               digitalRead(E1), digitalRead(F1), digitalRead(G1), digitalRead(H1));

  delay(500);
}


// void sendData(uint8_t Ddata, uint8_t Mmask) {
//   // Start at 7
//   for (int i = dataSize-1; i >= 0; i--) {
//     switch (i) {
//       case 0: digitalWrite(S2, 0b0); digitalWrite(S1, 0b0); digitalWrite(S0, 0b0); break;
//       case 1: digitalWrite(S2, 0b0); digitalWrite(S1, 0b0); digitalWrite(S0, 0b1); break;
//       case 2: digitalWrite(S2, 0b0); digitalWrite(S1, 0b1); digitalWrite(S0, 0b0); break;
//       case 3: digitalWrite(S2, 0b0); digitalWrite(S1, 0b1); digitalWrite(S0, 0b1); break;
//       case 4: digitalWrite(S2, 0b1); digitalWrite(S1, 0b0); digitalWrite(S0, 0b0); break;
//       case 5: digitalWrite(S2, 0b1); digitalWrite(S1, 0b0); digitalWrite(S0, 0b1); break;
//       case 6: digitalWrite(S2, 0b1); digitalWrite(S1, 0b1); digitalWrite(S0, 0b0); break;
//       case 7: digitalWrite(S2, 0b1); digitalWrite(S1, 0b1); digitalWrite(S0, 0b1); break;
//       default: Serial.println("Invalid case"); break;
//     }
//     uint8_t temp = (Ddata >> i) & 1;
//     uint8_t temp1 = (Mmask >> i) & 1;
//     Serial.printf("Data: 0x%02X\n", temp);
//     Serial.printf("Mask: 0x%02X\n", temp1);

//     digitalWrite(DATA, temp);
//     digitalWrite(MASK, temp1);
//     //delay(10);
//     Serial.printf("Monitor: %d\n", digitalRead(MONITOR));
//     maskedPixel[dataSize-1-i] = digitalRead(MONITOR);
//   }
// }
