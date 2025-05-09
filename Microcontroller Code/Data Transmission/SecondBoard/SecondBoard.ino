#include <Wire.h>

#define I2C_ADDRESS 0x25
#define INPUTAMOUNT 5           // Number of input pins
#define DATASIZE 5              // Matches number of inputs

int inputPins[INPUTAMOUNT] = {9, 14, 15, 16, 17};
uint8_t buttonBits[DATASIZE] = {0};       // Buffer sent on request
uint8_t currentBits[DATASIZE] = {0};      // Updated every loop

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < INPUTAMOUNT; i++) {
    pinMode(inputPins[i], INPUT);
  }

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
}

void loop() {
  // Clear all bits
  for (int i = 0; i < DATASIZE; i++) {
    currentBits[i] = 0;
  }

  if (digitalRead(9)  == HIGH)  currentBits[4] = 1; // NEW — Pin 9
  if (digitalRead(14) == HIGH) currentBits[3] = 1; // CAM
  if (digitalRead(15) == HIGH) currentBits[2] = 1; // R
  if (digitalRead(16) == HIGH) currentBits[1] = 1; // G
  if (digitalRead(17) == HIGH) currentBits[0] = 1; // B
                                                        
}

void requestEvent() {
  // Copy only when master requests
  memcpy(buttonBits, currentBits, DATASIZE);
  Wire.write(buttonBits, DATASIZE);

  // Debug:
  Serial.print("SENT: ");
  for (int i = DATASIZE - 1; i >= 0; i--) Serial.print(buttonBits[i]);
  delayMicroseconds(10); // small delay for stabilitydelay(50);
  Serial.println();
}
