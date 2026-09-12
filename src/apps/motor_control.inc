// Shared motor + onboard-LED control, factored out of motors_app.inc so
// the lidar apps can reuse the exact same "normal operation" cycle and
// override it with a safety stop when an obstacle is too close. Only
// ever included by exactly one app .inc per build (selected in
// main.cpp), so defining functions directly here (not just declaring
// them) is safe - there's only ever one translation unit.
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ESP32's "LEDC" peripheral is just a general PWM generator (the name is
// historical - Espressif built it for LED dimming). Alias it to "pwmc" so
// call sites read as what they actually do here: motor speed PWM.
#define pwmcAttach ledcAttach
#define pwmcWrite ledcWrite

// Onboard pixels.
#define LED_PIN 16
#define LED_COUNT 4
Adafruit_NeoPixel rgb_display_16 = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor pins per Maker-ESP32 README: M1(27,13) M2(4,2) M3(17,12) M4(14,15).
// Each pair drives one side of the TB67H450FNG H-bridge: pinA gets PWM
// (speed), pinB stays LOW (direction = forward). Swap A/B to reverse.
//
// M3/M4 (17,12,14,15) are shared with the board's Motor/IO switch -
// it must be flipped to the "Motor" side for M3/M4 to drive motors.
// The DC barrel jack (6-16V) must be connected and the power switch on;
// USB power alone will not drive the motor stage.

struct MotorPins {
  uint8_t pwmPin;
  uint8_t lowPin;
};

MotorPins motors[] = {
  {27, 13},  // M1
  {4,  2},   // M2
  {12, 17},  // M3 - A/B swapped vs README order: spun opposite the other 3 at bring-up
  {14, 15},  // M4
};
const int numMotors = 4;

// Per-motor speed trim, applied on top of DUTY_MAX/DUTY_MIN to compensate
// for motor-to-motor variance (winding/magnet tolerance, ~5-10% is
// normal) since there's no speed feedback (encoders) to close the loop
// on. 1.0 = no trim. Nudge a motor's factor down if it's visibly running
// faster than the others at the same commanded duty (checked load-free -
// rule out one wheel just being unloaded/lower-friction before trimming).
const float motorDutyTrim[] = {
  1.0,  // M1
  1.0,  // M2
  1.0,  // M3
  1.0,  // M4
};

// Average motor voltage is duty * SUPPLY_VOLTAGE_V, independent of PWM
// frequency. Motors are rated 6V, so duty is capped so the average never
// exceeds that; the slowest speed we drive down to is 3V.
const float SUPPLY_VOLTAGE_V = 9.0;      // barrel jack supply
const float MOTOR_MIN_VOLTAGE_V = 3.0;   // slowest speed we drive to
const float MOTOR_MAX_VOLTAGE_V = 6.0;   // motor's rated voltage - never exceed average

// TB67H450FNG datasheet: both IN pins held LOW for >=1ms(typ.) triggers
// standby; waking from it then takes up to 30us. NOTE: every PWM period
// we've used (past and current) is <=122us, two orders of magnitude
// under that 1ms threshold - so continuous PWM switching should never
// actually trigger standby entry in the first place, at any duty or
// frequency tried so far. The 30us wake time governs the transition out
// of a genuinely idle driver (e.g. before the first PWM edge after
// power-up), not every low period within a running PWM cycle. An
// earlier version of this file derived a ~9.3kHz frequency ceiling from
// misapplying that spec to steady-state operation - kept below as
// PWM_FREQ_MAX_SAFE_HZ for reference, but PWM_FREQ_HZ is no longer
// clamped to it pending an actual hardware test (see below).
const float WAKE_PULSE_FLOOR_US = 30.0;
const float WAKE_PULSE_MARGIN = 1.20;  // 20% headroom over the floor

const int PWM_RESOLUTION_BITS = 8;
const int DUTY_MIN = round((MOTOR_MIN_VOLTAGE_V / SUPPLY_VOLTAGE_V) * 255);
const int DUTY_MAX = round((MOTOR_MAX_VOLTAGE_V / SUPPLY_VOLTAGE_V) * 255);

const int PWM_FREQ_MAX_SAFE_HZ = (int)(1e6 / ((WAKE_PULSE_FLOOR_US * WAKE_PULSE_MARGIN) /
                                               (MOTOR_MIN_VOLTAGE_V / SUPPLY_VOLTAGE_V)));
// = 9259Hz - likely overly conservative per the note above, not applied below.

// Tested at 32768Hz (fully ultrasonic): motors spun uniformly slower
// than at 8192Hz despite identical commanded duty, at every point in
// the ramp, not just DUTY_MIN. That disproves "no constraint at all" -
// while the specific 1ms-standby-entry condition genuinely can't be
// triggering (period is still <<1ms), something else with similar
// effect is real: most likely a fixed turn-on/propagation delay per
// pulse in the driver's output stage, which eats a proportionally
// larger bite out of a ~20-30us pulse (32768Hz) than a ~40-80us one
// (8192Hz), even at the same duty%.
//
// Settled on 16384Hz after hardware A/B testing: inaudible (unlike
// 12288Hz, which was audible again) and the speed loss vs. the
// full-speed 8192Hz baseline was judged an acceptable trade for
// silence. Pulse widths 20.3-40.7us across DUTY_MIN-DUTY_MAX.
const int PWM_FREQ_HZ = 16384;

// Repeating cycle: hold low (3V) -> ramp up -> hold high (6V) -> ramp
// down -> repeat. LED brightness tracks how far duty is through its own
// DUTY_MIN..DUTY_MAX range (not the raw duty value - that only spans
// 85-170, i.e. 33-67% of full 0-255 brightness, which read as both too
// bright at rest and too narrow a band to look like it's varying).
// Scaled onto a deliberately dim band instead of full 0-255.
const int LED_BRIGHTNESS_MIN = 8;
const int LED_BRIGHTNESS_MAX = 60;
const int RAMP_STEP_DELAY_MS = 20;
const int LOW_HOLD_STEPS = 3000 / RAMP_STEP_DELAY_MS;
const int RAMP_UP_STEPS = 2000 / RAMP_STEP_DELAY_MS;
const int HIGH_HOLD_STEPS = 3000 / RAMP_STEP_DELAY_MS;
const int RAMP_DOWN_STEPS = 2000 / RAMP_STEP_DELAY_MS;
const int CYCLE_STEPS = LOW_HOLD_STEPS + RAMP_UP_STEPS + HIGH_HOLD_STEPS + RAMP_DOWN_STEPS;

// Pin setup + LEDC attach for all 4 motors, and NeoPixel init. Call once
// from setup().
//
// Both pins of each motor are LEDC-attached (not just pwmPin, with lowPin
// as a plain digital output) so setMotorSignedDuty() below can reverse a
// motor by moving the PWM to whichever pin should carry it, rather than
// only ever driving pwmPin - lowPin held at 0 duty is electrically the
// same as a constant digitalWrite LOW, so this doesn't change forward-only
// behavior for existing callers (runNormalCycleStep, stopMotorsAndSetRed).
void initMotorsAndLeds() {
  rgb_display_16.begin();

  for (int i = 0; i < numMotors; i++) {
    pwmcAttach(motors[i].pwmPin, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    pwmcAttach(motors[i].lowPin, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    pwmcWrite(motors[i].lowPin, 0);
  }
}

// Drives one motor at a signed duty in [-DUTY_MAX, DUTY_MAX]: magnitude
// (with that motor's trim applied) goes on whichever pin corresponds to
// the requested direction, 0 on the other. Magnitudes below DUTY_MIN are
// treated as stopped, same floor runNormalCycleStep ramps down to.
void setMotorSignedDuty(int motorIndex, int signedDuty) {
  const MotorPins &m = motors[motorIndex];
  int magnitude = abs(signedDuty);
  if (magnitude > DUTY_MAX) magnitude = DUTY_MAX;
  if (magnitude < DUTY_MIN) magnitude = 0;
  magnitude = round(magnitude * motorDutyTrim[motorIndex]);

  if (signedDuty >= 0) {
    pwmcWrite(m.pwmPin, magnitude);
    pwmcWrite(m.lowPin, 0);
  } else {
    pwmcWrite(m.pwmPin, 0);
    pwmcWrite(m.lowPin, magnitude);
  }
}

// Differential drive: motors[0..1] (M1,M2) are the left side, motors[2..3]
// (M3,M4) the right side (see motors[] above). linearX/angularZ are each
// expected in [-1, 1]; combined and rescaled so neither side ever exceeds
// +-1 before mapping onto DUTY_MAX, so a hard turn (large angularZ) doesn't
// get silently clamped asymmetrically.
void driveDifferential(float linearX, float angularZ) {
  linearX = constrain(linearX, -1.0f, 1.0f);
  angularZ = constrain(angularZ, -1.0f, 1.0f);

  float left = linearX - angularZ;
  float right = linearX + angularZ;
  float maxMag = max(1.0f, max(fabs(left), fabs(right)));
  left /= maxMag;
  right /= maxMag;

  int leftDuty = round(left * DUTY_MAX);
  int rightDuty = round(right * DUTY_MAX);
  setMotorSignedDuty(0, leftDuty);   // M1
  setMotorSignedDuty(1, leftDuty);   // M2
  setMotorSignedDuty(2, rightDuty);  // M3
  setMotorSignedDuty(3, rightDuty);  // M4
}

// Sets all 4 onboard NeoPixels to one color. r/g/b/brightness are each
// 0..1 (matching std_msgs/ColorRGBA's r/g/b/a floats directly).
void setRgbColor(float r, float g, float b, float brightness) {
  r = constrain(r, 0.0f, 1.0f);
  g = constrain(g, 0.0f, 1.0f);
  b = constrain(b, 0.0f, 1.0f);
  brightness = constrain(brightness, 0.0f, 1.0f);

  rgb_display_16.setBrightness(round(brightness * 255));
  uint32_t color = rgb_display_16.Color(round(r * 255), round(g * 255), round(b * 255));
  for (int i = 0; i < LED_COUNT; i++) {
    rgb_display_16.setPixelColor(i, color);
  }
  rgb_display_16.show();
}

// Normal operation: advances the low-ramp-high-ramp cycle by one step,
// drives the motors accordingly, and shows purple LEDs at a brightness
// proportional to how far duty is through its own range. Call once per
// loop() iteration while nothing is blocking normal operation - the
// step counter (and thus the 10s cycle) only advances on calls to this,
// so pausing calls (e.g. during an obstacle stop) simply pauses the
// cycle in place rather than resetting or skipping ahead.
void runNormalCycleStep() {
  static unsigned long step = 0;
  int cycleStep = step % CYCLE_STEPS;

  int currentDuty;
  if (cycleStep < LOW_HOLD_STEPS) {
    currentDuty = DUTY_MIN;
  } else if (cycleStep < LOW_HOLD_STEPS + RAMP_UP_STEPS) {
    int rampStep = cycleStep - LOW_HOLD_STEPS;
    currentDuty = DUTY_MIN + (long)(DUTY_MAX - DUTY_MIN) * rampStep / RAMP_UP_STEPS;
  } else if (cycleStep < LOW_HOLD_STEPS + RAMP_UP_STEPS + HIGH_HOLD_STEPS) {
    currentDuty = DUTY_MAX;
  } else {
    int rampStep = cycleStep - (LOW_HOLD_STEPS + RAMP_UP_STEPS + HIGH_HOLD_STEPS);
    currentDuty = DUTY_MAX - (long)(DUTY_MAX - DUTY_MIN) * rampStep / RAMP_DOWN_STEPS;
  }

  for (int i = 0; i < numMotors; i++) {
    pwmcWrite(motors[i].pwmPin, round(currentDuty * motorDutyTrim[i]));
  }

  int dutyFraction256 = (long)(currentDuty - DUTY_MIN) * 256 / (DUTY_MAX - DUTY_MIN);
  int ledBrightness = LED_BRIGHTNESS_MIN +
      (long)(LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN) * dutyFraction256 / 256;
  rgb_display_16.setBrightness(ledBrightness);  // proportional to duty cycle within its own range
  for (int i = 0; i < LED_COUNT; i++) {
    rgb_display_16.setPixelColor(i, rgb_display_16.Color(160, 0, 220));  // purple
  }
  rgb_display_16.show();

  step++;
  delay(RAMP_STEP_DELAY_MS);
}

// Safety stop: zero-duty (stopped) on all motors, solid red LEDs. Used
// by the lidar apps when an obstacle is within range or a reading can't
// be obtained at all.
void stopMotorsAndSetRed() {
  for (int i = 0; i < numMotors; i++) {
    pwmcWrite(motors[i].pwmPin, 0);
  }

  rgb_display_16.setBrightness(LED_BRIGHTNESS_MAX);
  for (int i = 0; i < LED_COUNT; i++) {
    rgb_display_16.setPixelColor(i, rgb_display_16.Color(255, 0, 0));  // red
  }
  rgb_display_16.show();

  delay(RAMP_STEP_DELAY_MS);  // keep the same pacing as the normal cycle
}
