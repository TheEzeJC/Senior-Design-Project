#include "esp_camera.h"
#include "camera_pins.h"
#include "sd_read_write.h"
#include "ScreenPins.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_system.h>

#define SLAVE_ADDRESS 0x25 // SDA=18 SCL=19 for SECOND BOARD
#define SDA_PIN 19
#define SCL_PIN 20
#define DATASIZE 5
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Pins connected to 74HC595
#define DATAPIN 42   // DS
#define LATCHPIN 2  // ST_CP
#define CLKPIN 1  // SH_CP

#define RED_PROC 21
#define GREEN_PROC 47
#define BLUE_PROC 48
#define GRAY_PROC 41

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Wire.begin(SDA_PIN, SCL_PIN);

  // 74HC595
  pinMode(DATAPIN, OUTPUT);
  pinMode(LATCHPIN, OUTPUT);
  pinMode(CLKPIN, OUTPUT);

  // Processed Image Results
  pinMode(RED_PROC, INPUT);
  pinMode(GREEN_PROC, INPUT);
  pinMode(BLUE_PROC, INPUT);
  pinMode(GRAY_PROC, INPUT);
  Serial.println();
  
  tft.initR(INITR_BLACKTAB);
  sdmmcInit();
  //removeDir(SD_MMC, "/camera");
  createDir(SD_MMC, "/camera");
  listDir(SD_MMC, "/camera", 0);
  if(cameraSetup()==1){
    tft.fillScreen(ST77XX_BLACK);
    tft.setRotation(3);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10,22);
    //tft.print("Image Color  Filter Using Logic Gates  Ezequiel V.  Logan D.     @MSU_2025");
    tft.print("Image Color  Filter Using Logic Gates");
    tft.setTextColor(ST77XX_BLUE);
    tft.print("  Ezequiel V.  Logan D.");
    tft.setTextColor(ST77XX_RED);
    tft.print("     @MSU_2025");
  }
  else{
    Serial.printf("Failed to setup camera in SETUP");
    return;
  }
}

// Variables
bool picTaken = false;
bool picDisplayed = false;
int imgResolution = 128;
int header_size = 54;
int data_size = 49152; // bmp_len[128*128*3(TGB)] - header_size;
int pixel_total = 16384; // data_size/(color_depth[24]/8);  or 128x128//8bits=1byte
uint8_t buttonBits[5];
uint8_t buttonBitsData[5];
int bitPosition = 0;
uint8_t bmpHeader[54];
uint8_t bmpData[49152];
uint8_t bmpModifiedData[49152];
uint8_t bmpFile[49206];
int pIndex;
int failed2Load = 0; // Used to keep track of fails

void loop() {
  // Read buttons from second MCU
  delay(4000); // Allows user enough time to change switches
  Wire.requestFrom(SLAVE_ADDRESS, DATASIZE);
  while (Wire.available() && bitPosition < DATASIZE) {
    buttonBits[bitPosition++] = Wire.read();
  }
  bitPosition = 0;

  // Reset Camera If button low & picTaken
  if((buttonBits[3] == LOW) && (picTaken == true)) {
    ESP.restart();
  }

  // Take Picture
  if((buttonBits[3] == HIGH) && (picTaken == false)) {
    Serial.print("Attempting To Take Image \n");
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (fb != NULL) {
      Serial.print("Taken Image \n");
      failed2Load = 0;
      int photo_index = readFileNum(SD_MMC, "/camera");
      if(photo_index!=-1) {
        pIndex = photo_index;

        // Convert JPEG to BMP
        uint8_t *bmp_buf;
        size_t bmp_len;
        if (!frame2bmp(fb, &bmp_buf, &bmp_len)) {
          Serial.println("BMP conversion FAIL");
          esp_camera_fb_return(fb);
          return;
        } else {
          Serial.println("BMP conversion SUCCESS");
        }

        // Save BMP data to destination
        // Save header information
        for (int i = 0; i < header_size; i++) {
          bmpHeader[i] = bmp_buf[i];
          //Serial.printf("bmpHeader[%d]: 0x%02X\n", i, bmpHeader[i]);
           bmpFile[i] = bmp_buf[i];
        }

        // Save image data
        for (int i = 0; i < data_size; i++) {
          bmpData[i] = bmp_buf[header_size+i]; //save at index 0, start from header
          //Serial.printf("bmpData[%d]: 0x%02X\n", i, bmpData[i]);
          bmpFile[header_size+i] = bmp_buf[header_size+i]; //Fills in data starting from header
        }
        free(bmp_buf); //free space
        picTaken = true;
      }
    }
    else {
      Serial.println("Camera capture failed.");
      picTaken = false;
      picDisplayed = false;
      esp_camera_return_all();
      failed2Load = 1 + failed2Load;
      Serial.printf("Attempt: %d\n", failed2Load);
      // Restart after failing to load
      if (failed2Load == 3){
        ESP.restart();
      }
    }
  }

  // Show or reprocess image
  if (picTaken == true) {
    if (picDisplayed == false) {
      memcpy(buttonBitsData, buttonBits, sizeof(buttonBits));
      processImage();
      saveImage();
      displayLCD(bmpModifiedData);
      picDisplayed = true;
    } else {
      // Check for button changes if already displayed
      bool buttonsChanged = false;
      for (int i = 0; i < sizeof(buttonBits); i++) {
        if (buttonBits[i] != buttonBitsData[i]) {
          buttonsChanged = true;
          break;
        }
      }

      if (buttonsChanged) {
        memcpy(buttonBitsData, buttonBits, sizeof(buttonBits));
        Serial.printf("Reprocessing Image\n");
        processImage();
        saveImage();
        displayLCD(bmpModifiedData);
      }
    }
  }

}

int cameraSetup(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_128X128;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound()){
    config.jpeg_quality = 7;
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.printf("PSRAM Found");
  } else {
    // Limit the frame size when PSRAM is not available
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.printf("No PSRAM Found \n");
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation

  Serial.println("Camera configuration complete!");
  return 1;
}

// Color stored as BGR
// Color needs to be displayed via RGB
void displayLCD(uint8_t* bmpDataT) {
  tft.fillScreen(ST77XX_BLACK);
  int pixel_counter = 0;
  for (int y = 0; y < imgResolution; y++){
    for(int x = 0; x < imgResolution; x++) {
      uint8_t blue  = (bmpDataT[pixel_counter]     >> 3) & 0b00011111; // B extract 5
      uint8_t green = (bmpDataT[pixel_counter + 1] >> 2) & 0b00111111; // G extract 6
      uint8_t red   = (bmpDataT[pixel_counter + 2] >> 3) & 0b00011111; // R extract 5

      uint16_t rgb = (red << 11) | (green << 5) | blue;

      //Serial.printf("Pixel[%d, %d]: 0x%04X\n", x, y, rgb);
      tft.drawPixel(x, y, rgb);
      pixel_counter+=3;
    }
  }
  Serial.printf("Image finished displaying \n");
}

// Used to assign selector bits
const uint8_t selector[8][3] = {
  {0, 0, 0}, // 0
  {1, 0, 0}, // 1
  {0, 1, 0}, // 2
  {1, 1, 0}, // 3
  {0, 0, 1}, // 4
  {1, 0, 1}, // 5
  {0, 1, 1}, // 6
  {1, 1, 1}  // 7
};
uint8_t Byte1[8] = {0};
uint8_t Byte2[8] = {0};
uint8_t Byte3[8] = {0};
uint8_t packedByte1 = 0b00000000;
uint8_t packedByte2 = 0b00000000;
uint8_t packedByte3 = 0b00000000;
  
uint8_t redData[8] = {0};
uint8_t greenData[8] = {0};
uint8_t blueData[8] = {0};
uint8_t grayData[8] = {0};

void processImage(void) {
  // Prep LCD
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,22);
  tft.setTextSize(3);

  // Override grayscale mode
  if (buttonBitsData[4] == HIGH) {
    Serial.printf("Filtering Image \n");
    tft.setTextColor(ST77XX_WHITE);
    tft.print("GRAY MAP");

    // Process Information Based On Inputs RGB
    for (int i = 0; i < data_size; i+=3) {
      Serial.printf("Data Range is %d to %d \n", i, i+2);
      // Send all RGB data
      for (int j = 0; j < 8; j++) {
        // Create Byte Structure
        // Save data to byte 1 bit at a time 
        Byte2[1] = (bmpData[i] >> j) & 0b00000001; // RED
        Byte2[4] = (bmpData[i+1] >> j) & 0b00000001; // GREEN
        Byte2[7] = (bmpData[i+2] >> j) & 0b00000001; // BLUE

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
      delayMicroseconds(10);

      // Apply shifint to corresponding data (Divide by 2)
      // RED shift 2
      for (int i = 0; i < 2; i++) {
        // Send Latch LOW at end
        Byte2[2] = 0; // Latch LOW
        packedByte2 = packBits(Byte2);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        // Pulse
        pulseClockData(1);

        // Send Latch HIGH at end
        Byte2[2] = 1; // Latch HIGH
        packedByte2 = packBits(Byte2);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        //pulse
        pulseClockData(1);
      }
      delayMicroseconds(10);

      // GREEN shifts 1
      for (int i = 0; i < 1; i++) {
        // Send Latch LOW at end
        Byte2[5] = 0; // Latch LOW
        packedByte2 = packBits(Byte2);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        // Pulse
        pulseClockData(2);

        // Send Latch HIGH at end
        Byte2[5] = 1; // Latch HIGH
        packedByte2 = packBits(Byte2);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        //pulse
        pulseClockData(2);
      }
      delayMicroseconds(10);

      // BLUE shifts 3
      for (int i = 0; i < 3; i++) {
        // Send Latch LOW at end
        Byte3[0] = 0; // Latch LOW
        packedByte3 = packBits(Byte3);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        // Pulse
        pulseClockData(3);

        // Send Latch HIGH at end
        Byte3[0] = 1; // Latch HIGH
        packedByte3 = packBits(Byte3);
        // Send LATCH Position To Main Shiters To Control Secondary
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);

        //pulse
        pulseClockData(3);
      }
      delayMicroseconds(10);

      // Read Proccessed Results
      delayMicroseconds(10);
      for (int k = 0; k < 8; k++) {
        // Give bit position to store back
        Byte3[2] = selector[k][0];
        Byte3[3] = selector[k][1];
        Byte3[4] = selector[k][2];
        // Store Instruction
        packedByte3 = packBits(Byte3);

        // Send instruction
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte3);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);
        digitalWrite(LATCHPIN, HIGH);
        delayMicroseconds(10); // Allows time for circuit to compute

        // Read bit and store bits
        grayData[k] = digitalRead(GRAY_PROC);
      }

      // Save proccessed data back (8 bits at a time)
      // Shared result amongs all colors
      uint8_t grayByte = packBits(grayData);
      bmpModifiedData[i]     = grayByte; // Blue
      bmpModifiedData[i + 1] = grayByte; // Green
      bmpModifiedData[i + 2] = grayByte; // Red
    }

  } else {
    // If RGB is LOW, Don't Do Processing
    if ((buttonBitsData[2] == LOW && buttonBitsData[1] == LOW && buttonBitsData[0] == LOW)) {
      memcpy(bmpModifiedData, bmpData, sizeof(bmpData)); //Make copy to other array
      Serial.printf("No Filterting By User\n");
      goto endProcessing;
    }

    Serial.printf("Filtering Image \n");
    // Send Mask Information 0 to remove and 1 to keep
    // RED enabled
    if (buttonBitsData[2] == HIGH) {
      Byte1[4] = 0;
    } else {
      Byte1[4] = 1;
      tft.setTextColor(ST77XX_RED);
      tft.print("RED     ");
    }

    // GREEN enabled
    if (buttonBitsData[1] == HIGH) {
      Byte1[6] = 0;
    } else {
      Byte1[6] = 1;
      tft.setTextColor(ST77XX_GREEN);
      tft.print("GREEN   ");
    }

    // BLUE enabled
    if (buttonBitsData[0] == HIGH) {
      Byte2[0] = 0;
    } else {
      Byte2[0] = 1;
      tft.setTextColor(ST77XX_BLUE);
      tft.print("BLUE    ");
    }

    // Grayscale using RGB
    if (buttonBitsData[2] == HIGH && buttonBitsData[1] == HIGH && buttonBitsData[0] == HIGH) {
      tft.setTextColor(ST77XX_WHITE);
      tft.print("GRAY MAP w/ RGB");
    }

    // Process Information Based On Inputs RGB
    for (int i = 0; i < data_size; i+=3) {
    // for (int i = 0; i < 1; i+=3) {
      Serial.printf("Data Range is %d to %d \n", i, i+2);
      for (int j = 0; j < 8; j++) {
        // Create Byte Structure
        // Give bit position
        Byte1[0] = selector[j][0];
        Byte1[1] = selector[j][1];
        Byte1[2] = selector[j][2];

        // Save data to byte 1 bit at a time 
        Byte1[3] = (bmpData[i] >> j) & 0b00000001; // RED
        Byte1[5] = (bmpData[i+1] >> j) & 0b00000001; // GREEN
        Byte1[7] = (bmpData[i+2] >> j) & 0b00000001; // BLUE

        //Convert
        packedByte1 = packBits(Byte1);
        packedByte2 = packBits(Byte2);
        //Serial.printf("Byte1 0x%02X\n", packedByte1);
        //Serial.printf("Byte2 0x%02X\n", packedByte2);

        // Send Bytes
        // Serial.printf("Data Bits %d: %d%d%d\n", j, Byte1[3], Byte1[5], Byte1[7]);
        digitalWrite(LATCHPIN, LOW);
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte2);  // goes into second 595
        shiftOut(DATAPIN, CLKPIN, MSBFIRST, packedByte1);  // goes into first 595
        digitalWrite(LATCHPIN, HIGH);
        delayMicroseconds(10);

        // Read Proccessed Results
        redData[j] = digitalRead(RED_PROC);
        greenData[j] = digitalRead(GREEN_PROC);
        blueData[j] = digitalRead(BLUE_PROC);

        // Serial.printf("Processed Bits: %d%d%d\n", redData[j], greenData[j], blueData[j]);
      }

      // Save proccessed data back (8 bits at a time)
      uint8_t redBits = packBits(redData);
      uint8_t greenBits = packBits(greenData);
      uint8_t blueBits = packBits(blueData);
      // Serial.printf("Red Byte: 0x%02X\n", redBits);
      // Serial.printf("Green Byte: 0x%02X\n", greenBits);
      // Serial.printf("Blue Byte: 0x%02X\n", blueBits);
      // bmpData[i] = redBits;
      // bmpData[i+1] = greenBits;
      // bmpData[i+2] = blueBits;

      bmpModifiedData[i]     = blueBits;
      bmpModifiedData[i + 1] = greenBits;
      bmpModifiedData[i + 2] = redBits;

    }
  }
  endProcessing:
    Serial.printf("Finished Filtering Image \n");
}

// Convert Bit Array to Byte
uint8_t packBits(uint8_t bits[8]) {
  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    result |= (bits[i] & 0x01) << i;  // shift bit to correct place
  }
  return result;
}

bool OGSaved = false;
// Save as BMP File
void saveImage() {
  // Build the extension based on button states
  String extension = "_";
  if (buttonBitsData[2] == LOW) extension += "R";
  if (buttonBitsData[1] == LOW) extension += "G";
  if (buttonBitsData[0] == LOW) extension += "B";
  if (buttonBitsData[2] == HIGH && buttonBitsData[1] == HIGH && buttonBitsData[0] == HIGH) {
              extension += "RGBGrayscale";}
  if (buttonBitsData[4] == HIGH) extension = "_Grayscale";

  // Build the full file path
  String path = "/camera/" + String(pIndex) + extension + ".bmp";

  // Save unedited image
  if (OGSaved == false) {
    String pathOG = "/camera/" + String(pIndex) + "_RGB" + ".bmp";
    writebmp(SD_MMC, pathOG.c_str(), bmpFile, sizeof(bmpFile));
    OGSaved = true;
  } else if (buttonBitsData[2] == LOW && buttonBitsData[1] == LOW && buttonBitsData[0] == LOW && buttonBitsData[4] == LOW) {
    // Skip saving if RGB is off AND Grayscale is off (no processing selected)
    return;
  }

  // Store Modified Image
  // Save header information
  for (int i = 0; i < header_size; i++) {
    bmpFile[i] = bmpHeader[i];
  }

  // Save image data
  for (int i = 0; i < data_size; i++) {
    bmpFile[header_size + i] = bmpModifiedData[i]; // Fills in data starting from header
  }

  // Only write if file doesn't already exist
  if (!SD_MMC.exists(path.c_str())) {
    writebmp(SD_MMC, path.c_str(), bmpFile, sizeof(bmpFile));
  } else {
    Serial.printf("File %s already exists. Skipping write.\n", path.c_str());
  }
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
