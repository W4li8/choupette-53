#include <Adafruit_NeoPixel.h>

#define LED_PIN 16
#define LED_COUNT 4

Adafruit_NeoPixel rgb_display_16 = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  rgb_display_16.begin();
  rgb_display_16.setBrightness(20);

  // orange
  for (int i = 0; i < LED_COUNT; i++) {
    rgb_display_16.setPixelColor(i, rgb_display_16.Color(255, 100, 0));
  }
  rgb_display_16.show();
}

void loop() {
}
