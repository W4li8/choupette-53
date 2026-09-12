// Temporary bring-up utility: scans the I2C bus and prints every
// address that ACKs. Not part of the normal app set - used to debug why
// a sensor isn't responding at its assumed default address.
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  Serial.println("Scanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  no devices found");
  }
  delay(2000);
}
