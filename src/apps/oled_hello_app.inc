#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Onboard OLED header on the Maker-ESP32, wired to the shared I2C bus
// (SDA=21, SCL=22 - see include/pins.h). SSD1306-style display at the
// classic default address 0x3C - confirmed via I2C scan while debugging
// the lidar_matrix app (this is what we were mistaking for the matrix
// ToF sensor not showing up). 128x64 assumed - the most common size at
// this address; adjust OLED_HEIGHT to 32 if the display is the shorter
// variant.
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// This panel is monochrome per pixel - "yellow" vs "blue" is purely
// which physical row a lit pixel falls on (yellow ~rows 0-15, blue
// ~rows 16-63 on the common split), not a settable color. SSD1306_WHITE
// is the only "on" state; landing text in the blue band is a matter of
// y-position, done below.
const char *LINE1 = "Hello, world!";
const char *LINE2 = "I'm JetsonCortex";
const int16_t BLUE_BAND_TOP = 16;

// Hardcoded rather than measured at runtime: at the default font's
// 6px/char (size 1) advance, size 2 would be 12px/char - 156px for
// LINE1 and 192px for LINE2, both over the 128px-wide screen. Size 1
// (78px/96px) is the only integer size that fits either line without
// overflow, so "slightly bigger" isn't achievable for this exact text
// on this display without clipping - shorten the strings if you want
// size 2.
const uint8_t TEXT_SIZE = 1;

void printCentered(const char *text, uint8_t size, int16_t y) {
  display.setTextSize(size);
  int16_t x, boundsY;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x, &boundsY, &w, &h);
  display.setCursor((OLED_WIDTH - w) / 2, y);
  display.print(text);
}

void setup() {
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);  // clip rather than bleed onto the next line if ever too wide

  int16_t lineHeight = 8 * TEXT_SIZE + 4;  // font is 8px tall/size unit, plus a gap
  int16_t y1 = BLUE_BAND_TOP + 4;
  int16_t y2 = y1 + lineHeight;

  display.clearDisplay();
  printCentered(LINE1, TEXT_SIZE, y1);
  printCentered(LINE2, TEXT_SIZE, y2);
  display.display();
}

void loop() {
}
