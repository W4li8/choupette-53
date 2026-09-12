#include "motor_control.inc"

void setup() {
  initMotorsAndLeds();
}

void loop() {
  runNormalCycleStep();
}
