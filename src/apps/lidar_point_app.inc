#include "motor_control.inc"
#include <Wire.h>
#include <VL53L0X.h>

// DFRobot/Gravity VL53L0X single-point ToF sensor over I2C, default
// address 0x29 (matches this sensor's factory default - no jumper/init
// override needed for a single unit on the bus).
VL53L0X pointSensor;

const int OBSTACLE_THRESHOLD_MM = 100;  // 10cm
const int SENSOR_RETRY_INTERVAL_MS = 500;

// VL53L0X hardware quirk (confirmed against the Pololu library source,
// not documentation): when it can't get a valid reading (no target in
// range, weak/absent reflection), it reports ~8190mm - NOT a timeout
// (that returns 65535 and sets timeoutOccurred()) and NOT zero. Treated
// as a safe/clear reading (nothing within range), not as "no data" -
// it's already >= OBSTACLE_THRESHOLD_MM so no separate check is needed;
// only a real timeout (no data at all) counts as "no reading" below.

bool sensorReady = false;
unsigned long lastInitAttemptMs = 0;

// (Re)initializes the sensor. Returns true once it's ready to read.
// Called from setup() and retried from loop() on failure, so a sensor
// plugged in after boot (or power-cycled) recovers without a reflash.
bool tryInitSensor() {
  Wire.begin();
  if (!pointSensor.init()) {
    return false;
  }
  pointSensor.setTimeout(500);
  pointSensor.startContinuous();
  return true;
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif
  initMotorsAndLeds();
  sensorReady = tryInitSensor();
  lastInitAttemptMs = millis();
}

void loop() {
  if (!sensorReady) {
    if (millis() - lastInitAttemptMs >= SENSOR_RETRY_INTERVAL_MS) {
      sensorReady = tryInitSensor();
      lastInitAttemptMs = millis();
#ifdef SERIAL_DEBUG
      Serial.println(sensorReady ? "sensor init OK" : "sensor init failed, retrying...");
#endif
    }
    stopMotorsAndSetRed();  // no reading yet - stay stopped
    return;
  }

  uint16_t distanceMm = pointSensor.readRangeContinuousMillimeters();

  // Timeout, or a bogus/zero reading, is treated the same as "no
  // reading" per spec: stop and go red until real values resume. A
  // timeout also means the sensor may have dropped off the bus -
  // re-probe it next loop by clearing sensorReady.
  if (pointSensor.timeoutOccurred()) {
#ifdef SERIAL_DEBUG
    Serial.println("sensor timeout");
#endif
    sensorReady = false;
    stopMotorsAndSetRed();
    return;
  }

#ifdef SERIAL_DEBUG
  Serial.print("distance_mm=");
  Serial.println(distanceMm);
#endif

  if (distanceMm < OBSTACLE_THRESHOLD_MM) {
    stopMotorsAndSetRed();
  } else {
    runNormalCycleStep();
  }
}
