const uint8_t inputPins[4] = {36, 39, 34, 35};  // Only 4 pins defined
uint32_t conditionCount = 0;  // Counter for alert condition

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {  // Loop over 4 pins
    pinMode(inputPins[i], INPUT);
  }
}

void loop() {
  uint8_t inputState = 0;  // Only 4 bits needed

  for (int i = 0; i < 4; i++) {  // Read all 4 pins
    if (digitalRead(inputPins[i]) == HIGH) {
      inputState |= (1 << i);
    }
  }

  // Print the states
  for (int i = 0; i < 4; i++) {
    Serial.print("Pin ");
    Serial.print(inputPins[i]);
    Serial.print(": ");
    Serial.print((inputState >> i) & 0x01);
    if (i < 3) Serial.print(" | ");  // Only up to 3 separators
  }

  // Example: Check a condition based on pins
  // (adjust as needed)
  // if (((inputState >> 0) & 0x01) && ((inputState >> 1) & 0x01) && !((inputState >> 2) & 0x01)) {
  //   conditionCount++;
  //   Serial.printf(" Condition met! Count: %d\n", conditionCount);
  // }

  Serial.println();
  delay(50);
}
