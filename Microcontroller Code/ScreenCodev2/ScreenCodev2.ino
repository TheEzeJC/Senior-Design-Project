#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#define TFT_SCLK 3
#define TFT_MOSI 46
#define TFT_DC 14 //( RS on board )
#define TFT_RST 19
#define TFT_CS 20
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup()
{
tft.initR(INITR_BLACKTAB);
tft.fillScreen(ST77XX_BLACK);
tft.setRotation(3);
tft.setTextColor(ST77XX_GREEN);
tft.setTextSize(2);
tft.setCursor(10,22);
tft.print("kekw");
}

void loop() {

}