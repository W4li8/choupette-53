#include "motor_control.inc"
#include "dfrobot_matrix_lidar.inc"

// DFRobot Gravity 8x8 Matrix ToF sensor (VL53L7CX + onboard RP2040,
// SKU SEN0628) over I2C. Default address is 0x33 (NOT 0x30 - checked
// against the library source, github.com/DFRobot/DFRobot_MatrixLidar).
DFRobot_MatrixLidar_I2C matrixSensor(0x33);

const int MATRIX_ZONES = 64;  // 8x8
uint16_t matrixBuf[MATRIX_ZONES];

const int OBSTACLE_THRESHOLD_MM = 100;  // 10cm
const int SENSOR_RETRY_INTERVAL_MS = 500;

bool sensorReady = false;
unsigned long lastInitAttemptMs = 0;

// (Re)initializes the sensor and configures 8x8 ranging mode. Returns
// true once it's ready to read. Called from setup() and retried from
// loop() on failure, so a sensor plugged in after boot (or
// power-cycled) recovers without a reflash.
bool tryInitSensor() {
  if (matrixSensor.begin() != 0) {
    return false;
  }
  if (matrixSensor.setRangingMode(eMatrix_8X8) != 0) {
    return false;
  }
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

  uint8_t err = matrixSensor.getAllData(matrixBuf);

  // A failed read (bus error, sensor dropped off, etc.) is treated the
  // same as "no reading" per spec: stop and go red until real values
  // resume, and re-probe the sensor from scratch next loop.
  if (err != 0) {
#ifdef SERIAL_DEBUG
    Serial.println("getAllData failed");
#endif
    sensorReady = false;
    stopMotorsAndSetRed();
    return;
  }

#ifdef SERIAL_DEBUG
  // True 8x8 grid, matching the sensor's own row-major zone layout
  // (buf[y*8+x], y=0 is the row printed first).
  Serial.println("matrix_mm:");
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      Serial.print(matrixBuf[y * 8 + x]);
      Serial.print('\t');
    }
    Serial.println();
  }
#endif

  // Per spec: stop only when the ENTIRE grid reads under the threshold,
  // not just some zones - an out-of-range zone (~4000mm, this sensor's
  // own "no target in range" sentinel) is a safe/clear reading, so its
  // mere presence anywhere in the frame blocks the stop from triggering
  // even if other zones are close. Deliberate, but worth knowing: given
  // how differently individual zones tend to read (see the live grid -
  // an angled/uneven scene rarely reads uniformly close), this condition
  // may rarely or never trip in practice.
  bool allZonesClose = true;
  for (int i = 0; i < MATRIX_ZONES; i++) {
    if (matrixBuf[i] >= OBSTACLE_THRESHOLD_MM) {
      allZonesClose = false;
      break;
    }
  }

  if (allZonesClose) {
    stopMotorsAndSetRed();
  } else {
    runNormalCycleStep();
  }
}
