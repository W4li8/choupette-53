// Minimal example: echoes each line received over the USB serial line
// back out the same line. Used to smoke-test ../../ros2_chatter_bridge.py
// (a pixi ROS2 node that publishes "chatter N" on the /chatter topic once
// a second and relays the same text over /dev/esp32maker) - not part of
// the zenoh/micro-ROS pipeline discussed for the real robot, just proof
// the ROS2 <-> ESP32 wiring works end to end.
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("esp32 got: ");
      Serial.println(line);
    }
  }
}
