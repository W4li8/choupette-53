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

// TB67H450FNG datasheet: standby-release (both IN pins were LOW -> one
// goes HIGH) takes up to 30us to actually start driving the output, so
// every PWM HIGH pulse needs to clear that floor with margin. The
// shortest pulse of the whole speed range comes from the lowest duty
// (MOTOR_MIN_VOLTAGE_V), so frequency is picked to keep that pulse
// WAKE_PULSE_MARGIN above the floor - lower frequency (longer period)
// gives more margin at a given duty; raising frequency shrinks pulses
// and eats the margin, it doesn't help.
const float WAKE_PULSE_FLOOR_US = 30.0;
const float WAKE_PULSE_MARGIN = 1.20;  // 20% headroom over the floor

const int PWM_RESOLUTION_BITS = 8;
const int DUTY_MIN = round((MOTOR_MIN_VOLTAGE_V / SUPPLY_VOLTAGE_V) * 255);
const int DUTY_MAX = round((MOTOR_MAX_VOLTAGE_V / SUPPLY_VOLTAGE_V) * 255);

// Max frequency that still keeps the min-duty (slowest speed) pulse
// WAKE_PULSE_MARGIN above the floor: period_us >= (FLOOR*MARGIN)/duty_min,
// freq = 1e6/period_us. Anything at or below this is safe re: wake time.
const int PWM_FREQ_MAX_SAFE_HZ = (int)(1e6 / ((WAKE_PULSE_FLOOR_US * WAKE_PULSE_MARGIN) /
                                               (MOTOR_MIN_VOLTAGE_V / SUPPLY_VOLTAGE_V)));
// = 9259Hz.

// 2x LEDC's 4096-tick reference, comfortably under the ceiling above (more
// margin than requested: 40.7us pulse at min duty vs the 30us*1.2=36us
// ask). Audible-range - motors will have a PWM whine at this frequency,
// traded for wake-pulse margin across the full 3-6V speed range. Doesn't
// change peak voltage the motor sees (that's always the full supply
// during each HIGH pulse, at any frequency) - only DUTY_MIN/DUTY_MAX
// above control the average voltage the motor is rated for.
const int PWM_FREQ_HZ = 8192;

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
void initMotorsAndLeds() {
  rgb_display_16.begin();

  for (int i = 0; i < numMotors; i++) {
    pinMode(motors[i].lowPin, OUTPUT);
    digitalWrite(motors[i].lowPin, LOW);
    pwmcAttach(motors[i].pwmPin, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  }
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
