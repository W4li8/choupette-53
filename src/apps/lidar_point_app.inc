#include "motor_control.inc"
#include <Wire.h>
#include <VL53L0X.h>

// DFRobot/Gravity VL53L0X single-point ToF sensor(s) sharing one I2C
// bus. They all boot at the same factory-default address (0x29), so
// with more than one on the bus, EVERY sensor needs its own XSHUT
// (shutdown) pin - including one left at the default address, even
// though its address never changes: VL53L0X address reassignment is
// volatile (lives in SRAM, not flash), so it's lost on every power-
// cycle and this whole sequence must run on every boot. If a sensor
// staying at 0x29's XSHUT weren't controllable, it would still be
// sitting at 0x29 while others are being isolated and reassigned - a
// real bus collision, and worse, a risk that it silently accepts the
// same address-change command intended for whichever sensor is being
// reassigned at that moment. Hold every sensor in reset, bring them up
// one at a time (order doesn't matter), and reassign each while it's
// the only one live at 0x29. XSHUT is pulled HIGH (active) by default
// on this breakout; driving it LOW powers the sensor down/holds it in
// reset.
//
// A sensor with no XSHUT wired at all (xshutPin = -1 below) is assumed
// always powered/active and left at whatever address it's already at
// - fine as long as it's the only sensor on the bus, or the only one
// sitting at that address. Currently only one sensor is wired (default
// address, no XSHUT), the other two physical slots below are commented
// out pending rewiring.
//
// XSHUT wired to the SPI header's SS/SCK/MISO pins (5/18/19 - unused
// here, freeing up 25/26/32 for servos later). See include/pins.h
// PIN_SPI_SS, PIN_SPI_SCK, PIN_SPI_MISO.
//
// GPIO5 is an ESP32 strapping pin (SDIO slave timing select at boot) -
// fairly benign as strapping pins go, and since this breakout's XSHUT
// is pulled HIGH by default and we only drive it low from firmware
// (well after the ESP32's own boot-time strap sampling), this should be
// safe in practice.
struct PointSensorConfig {
  int8_t xshutPin;  // -1 = not wired (sensor always active, no reset control)
  uint8_t address;
};

PointSensorConfig sensorConfigs[] = {
  {-1, 0x29},  // only sensor currently wired - default address, no XSHUT
  // {5, 0x28},   // left - not wired right now
  // {19, 0x30},  // right - not wired right now
};
const int numPointSensors = 1;
VL53L0X pointSensors[numPointSensors];

const int OBSTACLE_THRESHOLD_MM = 100;  // 10cm
const int SENSOR_RETRY_INTERVAL_MS = 500;

// VL53L0X hardware quirk (confirmed against the Pololu library source,
// not documentation): when it can't get a valid reading (no target in
// range, weak/absent reflection), it reports ~8190mm - NOT a timeout
// (that returns 65535 and sets timeoutOccurred()) and NOT zero. Treated
// as a safe/clear reading (nothing within range), not as "no data" -
// it's already >= OBSTACLE_THRESHOLD_MM so no separate check is needed;
// only a real timeout (no data at all) counts as "no reading" below.
//
// Max reading rate over I2C (ST datasheet): continuous ranging can go
// up to ~50Hz (20ms/reading) at a reduced timing budget; the default
// timing budget used here (Pololu library default, ~33ms) gives ~30Hz.
// Actual achieved rate is measured and printed under SERIAL_DEBUG below
// rather than assumed, since I2C bus contention across 3 sensors plus
// our own loop overhead could bring it down further.

bool allSensorsReady = false;
unsigned long lastInitAttemptMs = 0;

// Brings up every sensor and gives each its own address. All-or-
// nothing: if any sensor fails to respond, every controllable XSHUT is
// driven back LOW and the whole sequence is retried from scratch next
// call - never left half-done, since a partially-addressed sensor
// would still be sitting at the shared default 0x29 and collide with
// the next one brought out of reset. Sensors with no XSHUT wired
// (xshutPin = -1) are skipped in the reset/enable steps - they're
// always powered and assumed to already be alone at their configured
// address.
bool tryInitAllSensors() {
  Wire.begin();

  for (int i = 0; i < numPointSensors; i++) {
    if (sensorConfigs[i].xshutPin < 0) continue;
    pinMode(sensorConfigs[i].xshutPin, OUTPUT);
    digitalWrite(sensorConfigs[i].xshutPin, LOW);
  }
  delay(10);  // XSHUT low long enough to fully reset every sensor

  for (int i = 0; i < numPointSensors; i++) {
    if (sensorConfigs[i].xshutPin >= 0) {
      digitalWrite(sensorConfigs[i].xshutPin, HIGH);
      delay(10);  // boot time before it'll ACK on the bus
    }

    pointSensors[i].setTimeout(500);
    if (!pointSensors[i].init()) {
      for (int j = 0; j < numPointSensors; j++) {
        if (sensorConfigs[j].xshutPin < 0) continue;
        digitalWrite(sensorConfigs[j].xshutPin, LOW);
      }
      return false;
    }
    if (sensorConfigs[i].address != 0x29) {
      pointSensors[i].setAddress(sensorConfigs[i].address);
    }
    pointSensors[i].startContinuous();
  }
  return true;
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif
  initMotorsAndLeds();
  allSensorsReady = tryInitAllSensors();
  lastInitAttemptMs = millis();
}

void loop() {
  if (!allSensorsReady) {
    if (millis() - lastInitAttemptMs >= SENSOR_RETRY_INTERVAL_MS) {
      allSensorsReady = tryInitAllSensors();
      lastInitAttemptMs = millis();
#ifdef SERIAL_DEBUG
      Serial.println(allSensorsReady ? "all sensors init OK" : "sensor init failed, retrying...");
#endif
    }
    stopMotorsAndSetRed();  // no reading yet - stay stopped
    return;
  }

#ifdef SERIAL_DEBUG
  static unsigned long readsThisWindow = 0;
  static unsigned long windowStartMs = 0;
#endif

  bool obstacleDetected = false;
  for (int i = 0; i < numPointSensors; i++) {
    uint16_t distanceMm = pointSensors[i].readRangeContinuousMillimeters();

    // Timeout means we've lost a sensor - re-run the whole addressing
    // sequence next loop rather than trust the other two's addresses
    // are still intact (e.g. after a shared power blip).
    if (pointSensors[i].timeoutOccurred()) {
#ifdef SERIAL_DEBUG
      Serial.print("sensor ");
      Serial.print(i);
      Serial.println(" timeout");
#endif
      allSensorsReady = false;
      stopMotorsAndSetRed();
      return;
    }

#ifdef SERIAL_DEBUG
    Serial.print("sensor");
    Serial.print(i);
    Serial.print("_mm=");
    Serial.print(distanceMm);
    Serial.print("  ");
#endif

    if (distanceMm < OBSTACLE_THRESHOLD_MM) {
      obstacleDetected = true;
    }
  }

#ifdef SERIAL_DEBUG
  Serial.println();
  readsThisWindow++;
  unsigned long now = millis();
  if (now - windowStartMs >= 1000) {
    Serial.print("measured rate: ");
    Serial.print(readsThisWindow * 1000.0 / (now - windowStartMs));
    Serial.println(" reads/sec (all 3 sensors, per loop)");
    readsThisWindow = 0;
    windowStartMs = now;
  }
#endif

  if (obstacleDetected) {
    stopMotorsAndSetRed();
  } else {
    runNormalCycleStep();
  }
}
