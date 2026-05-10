/*
 * pins.h — GPIO pin assignments for the CUSF Actuation Board
 *
 * All pin numbers are defined here so they're easy to change
 * if the PCB layout moves things around. Change a number here
 * and it updates everywhere in the firmware.
 *
 * Hardware:
 *   ESP32-S3-DevKitC-1
 *   2x MPQ6610 solenoid drivers (DNP — pads on board, not soldered yet)
 *   4x FT5330M servos (7.4V, 500–2500µs PWM)
 *
 * Pins to avoid on ESP32-S3:
 *   GPIO0, 3, 45, 46  — strapping pins (affect boot mode)
 *   GPIO26–32          — internal SPI flash
 *   GPIO19, 20         — native USB (used automatically)
 */

#ifndef PINS_H
#define PINS_H

// ─── Solenoids (MPQ6610) ─── DNP: pads exist but not soldered yet
// When propulsion team decides on solenoids, solder the MPQ6610s
// and set SOLENOIDS_FITTED to true
#define SOLENOIDS_FITTED  false

#define PIN_SOL1_EN       4     // MPQ6610 #1 enable
#define PIN_SOL1_IN       5     // MPQ6610 #1 input (high = energise)
#define PIN_SOL2_EN       6     // MPQ6610 #2 enable
#define PIN_SOL2_IN       7     // MPQ6610 #2 input
#define PIN_NFAULT1       8     // MPQ6610 #1 fault output (active low)
#define PIN_NFAULT2       9     // MPQ6610 #2 fault output (active low)

// ─── Servos (FT5330M) ───
#define PIN_SERVO1        10    // LEDC channel 0
#define PIN_SERVO2        11    // LEDC channel 1
#define PIN_SERVO3        12    // LEDC channel 2
#define PIN_SERVO4        13    // LEDC channel 3

// ─── Servo PWM settings ───
// FT5330M specs: 50Hz, 500–2500µs pulse range, 180° rotation
#define SERVO_FREQ_HZ     50
#define SERVO_RESOLUTION   14   // 14-bit = 16384 steps per 20ms period
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500

// ─── Bus to instrumentation board ───
// Sensors are read directly from the instrumentation board's digital ICs
// (ADS1115 on I2C, MAX31856 on SPI, HX711). Pins to be assigned once
// PCB layout is finalised.
// #define PIN_I2C_SDA    14
// #define PIN_I2C_SCL    15

#endif
