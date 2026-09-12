#pragma once

// Maker-ESP32 pin map.
// Sources: https://github.com/nulllaborg/maker-esp32 README, cross-checked
// against the KiCad schematic (maker-esp32.pdf) in that repo for pins the
// README doesn't spell out (I2C/SPI GPIO numbers, ADC header pairings).

// ---- DC motor ports (M1-M4) ----
// Each port is a PWM/direction pair into its own TB67H450FNG H-bridge
// (schematic: U10=M1, U9=M2, U8=M3, U7=M4): the _A pin takes PWM (speed),
// _B stays LOW for forward (swap A/B to reverse). All four ICs' VM pin
// is fed from raw VIN (the barrel jack, 6-16V, upstream of the 5V/3.3V
// regulators) - USB power alone won't drive the motor stage.
#define PIN_M1_A 27
#define PIN_M1_B 13

#define PIN_M2_A 4
#define PIN_M2_B 2

// M3/M4 double as the general-IO/ADC pins below (JP17/JP23/JP20/JP24) -
// only usable as motor drivers when the board's Motor/IO switch (SW1, a
// 4-pole DIP switch) is set to "Motor". See MOTOR_IO_SWITCH_SET_TO_*.
#define PIN_M3_A 17
#define PIN_M3_B 12

#define PIN_M4_A 14
#define PIN_M4_B 15

// ---- Stepper motor pins (share wiring with the DC motor ports) ----
// A stepper can't run at the same time as the DC motor(s) it shares
// pins with.
#define PIN_STEPPER1_A1 PIN_M1_A  // 27
#define PIN_STEPPER1_A2 PIN_M1_B  // 13
#define PIN_STEPPER1_B1 PIN_M2_A  // 4
#define PIN_STEPPER1_B2 PIN_M2_B  // 2

#define PIN_STEPPER2_A1 PIN_M3_A  // 17
#define PIN_STEPPER2_A2 PIN_M3_B  // 12
#define PIN_STEPPER2_B1 PIN_M4_A  // 14
#define PIN_STEPPER2_B2 PIN_M4_B  // 15

// ---- Servo headers ----
// Each of the 4 connectors (JP12/JP14/JP16/JP19) is individually fused
// (1A) and powered from +5V, signal-only pin listed here.
#define PIN_SERVO1 25
#define PIN_SERVO2 26
#define PIN_SERVO3 32
#define PIN_SERVO4 33

// ---- Onboard RGB LEDs (4x WS2812, daisy-chained, 100R series resistor) ----
#define PIN_RGB_LED 16
#define RGB_LED_COUNT 4

// ---- I2C bus ----
// One shared bus broken out to 5 physical 4-pin (VCC/SDA/SCL/GND)
// headers (JP3-JP6 general-purpose + JP2 dedicated OLED header) - not
// 5 separate buses. Standard ESP32 Wire default pins.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_OLED_SDA PIN_I2C_SDA
#define PIN_OLED_SCL PIN_I2C_SCL

// ---- SPI header (VSPI) ----
// 4 separate 3-pin (+3V3/signal/GND) connectors (JP7/JP8/JP10/JP11), one
// SPI signal each. GPIO5 is an ESP32 strapping pin (VSPI CS0 default) -
// avoid holding it low externally at boot.
#define PIN_SPI_SS 5
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23

// ---- Motor/IO switch ----
// SW1: 4-pole DIP switch reassigning pins 12, 14, 15, 17 between
// motor-driver duty (M3/M4, above) and the general-IO/ADC headers below.
// The ESP32 can't read the switch's position itself, so define
// MOTOR_IO_SWITCH_SET_TO_MOTOR or MOTOR_IO_SWITCH_SET_TO_IO via
// build_flags to match its physical position before including this
// header anywhere that cares which role those pins are playing.
#if defined(MOTOR_IO_SWITCH_SET_TO_IO)
  // Pins 12, 14, 15, 17 are general GPIO on the ADC headers below (JP24,
  // JP23, JP20, JP17 respectively), not connected to the H-bridges.
  #define PIN_IO_1 12  // = PIN_M3_B when switch is on Motor instead
  #define PIN_IO_2 14  // = PIN_M4_A when switch is on Motor instead
  #define PIN_IO_3 15  // = PIN_M4_B when switch is on Motor instead
  #define PIN_IO_4 17  // = PIN_M3_A when switch is on Motor instead
#elif defined(MOTOR_IO_SWITCH_SET_TO_MOTOR)
  // Pins 12, 14, 15, 17 drive M3/M4 - see PIN_M3_*/PIN_M4_* above.
#else
  #error "Define MOTOR_IO_SWITCH_SET_TO_MOTOR or MOTOR_IO_SWITCH_SET_TO_IO (build_flags) to match the board's physical Motor/IO switch position."
#endif

// ---- ADC / general-IO headers ----
// 4 combo connectors, each pairing one switch-gated pin above with a
// dedicated input-only ADC pin on the same physical header:
//   JP17: PIN_IO_4/PIN_M3_A (17) + PIN_ADC_1 (34)
//   JP23: PIN_IO_2/PIN_M4_A (14) + PIN_ADC_2 (35)
//   JP20: PIN_IO_3/PIN_M4_B (15) + PIN_ADC_3 (39)
//   JP24: PIN_IO_1/PIN_M3_B (12) + PIN_ADC_4 (36)
// 34/35/36/39 are input-only: no output driver, no internal pull-up/down
// (add external resistors if using them for buttons/digital signals).
#define PIN_ADC_1 34  // input-only, on the JP17 header alongside PIN_M3_A
#define PIN_ADC_2 35  // input-only, on the JP23 header alongside PIN_M4_A
#define PIN_ADC_3 39  // input-only, on the JP20 header alongside PIN_M4_B
#define PIN_ADC_4 36  // input-only, on the JP24 header alongside PIN_M3_B

// ---- USB / programming (reserved - not user-facing headers) ----
// Onboard CH340G USB-serial bridge onto ESP32 UART0, with DTR/RTS wired
// through Q1/Q2 to EN/IO0 for auto-reset-into-bootloader. Standard ESP32
// UART0 pins; avoid repurposing while using the USB serial port.
#define PIN_UART0_TX 1
#define PIN_UART0_RX 3

// ---- Power rails (not GPIOs - listed for reference) ----
// VIN:  raw barrel-jack input (5.5x2.1mm, 6-16V), also selectable from
//       USB VBUS via a MOSFET OR-ing circuit for the regulated rails
//       below - but the motor H-bridges are fed raw VIN directly, so
//       USB-only power will not drive motors.
// +5V:  buck-regulated from VIN (or USB VBUS), feeds servos, RGB LEDs,
//       CH340G.
// +3V3: regulated from +5V, feeds the ESP32 module and logic.
