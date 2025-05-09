// Problem was using pin 48 was interfering with WS2812
#define DATAPIN 42   // DS
#define LATCHPIN 2  // ST_CP
#define CLKPIN 1  // SH_CP
#define RED_PROC 21
#define GREEN_PROC 47
#define BLUE_PROC 48

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  // 74HC595
  pinMode(DATAPIN, OUTPUT);
  pinMode(LATCHPIN, OUTPUT);
  pinMode(CLKPIN, OUTPUT);

  // Input
  pinMode(RED_PROC, INPUT);
  pinMode(GREEN_PROC, INPUT);
  pinMode(BLUE_PROC, INPUT);
}

void loop() {
  // int8_t packedByte1 = 0b11111000;
  // int8_t packedByte1 = 0b11111001;
  // int8_t packedByte1 = 0b11111010;
  // int8_t packedByte1 = 0b11111011;
  // int8_t packedByte1 = 0b11111100;
  // int8_t packedByte1 = 0b11111101;
  // int8_t packedByte1 = 0b11111110;
  int8_t packedByte1 = 0b11111111;
  int8_t packedByte2 = 0b00000001;

  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);  // goes into second 595
  shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);  // goes into first 595
  digitalWrite(LATCHPIN, HIGH);


  Serial.printf("Processed Bits: %d%d%d\n", 
                digitalRead(RED_PROC), digitalRead(GREEN_PROC), digitalRead(BLUE_PROC));
  
  if (digitalRead(RED_PROC) == 1 &&
      digitalRead(GREEN_PROC) == 1 &&
       digitalRead(BLUE_PROC) == 1) {
        Serial.printf("Processed Bits: %d%d%d\n", error);
       }
}