/*
 * pins_v3.h — GPIO pin assignments for the CUSF Actuation Board v2
 *
 * Hardware redesign summary vs v1:
 *   - Servos moved to GPIO 4–7 (J1–J4 signal headers)
 *   - MPQ6610 solenoid drivers removed from this board entirely;
 *     GPIO 9–14 are routed to the J6 expansion header for a future
 *     solenoid daughterboard (do not drive these for now)
 *   - All sensor reading moved to the instrumentation board;
 *     communication is now via CAN bus (SN65HVD230) on GPIO 15/16
 *     instead of the old I2C/SPI/HX711 wiring
 *
 * Pins to avoid on ESP32-S3:
 *   GPIO 0, 3, 45, 46  — strapping pins (affect boot mode)
 *   GPIO 26–32          — internal SPI flash
 *   GPIO 19, 20         — native USB (used automatically by Serial)
 *
 * CAN protocol summary (500 kbps, little-endian, 11-bit standard IDs):
 *   0x010  instr → act   Heartbeat (uptime u32 | fault_flags u16 | fw u8 u8)
 *   0x020  act → instr   Heartbeat (same layout)
 *   0x100  instr → act   Pressure ch 1–4  (4× int16 raw counts)
 *   0x101  instr → act   Pressure ch 5–8  (4× int16 raw counts)
 *   0x102  instr → act   Thermocouples 1–4 (4× int16, MAX31856 format)
 *   0x103  instr → act   Load cell (int32 reading | status u8 | 3 reserved)
 *   0x200  act → instr   Arm request    (reserved, not used yet)
 *   0x201  act → instr   Emergency stop (reserved, not used yet)
 */

#ifndef PINS_V3_H
#define PINS_V3_H

// ─── Servos (FT5330M, 7.4 V) ────────────────────────────────────────────────
// Four servo channels on J1–J4 signal headers.
// These occupy GPIO 4–7 in v2 (moved from 10–13 in v1).
#define PIN_SERVO1        4    // J1 — LEDC channel 0
#define PIN_SERVO2        5    // J2 — LEDC channel 1
#define PIN_SERVO3        6    // J3 — LEDC channel 2
#define PIN_SERVO4        7    // J4 — LEDC channel 3

// ─── Servo PWM settings ──────────────────────────────────────────────────────
// FT5330M spec: 50 Hz carrier, 500–2500 µs pulse range, 180° rotation.
// 14-bit resolution gives 16384 steps across the 20 ms period, so each
// microsecond corresponds to ~0.82 LEDC counts — more than enough precision.
#define SERVO_FREQ_HZ      50
#define SERVO_RESOLUTION   14   // 14-bit → 16384 steps per 20 ms period
#define SERVO_MIN_US       500  // fully closed
#define SERVO_MID_US       1500 // centre
#define SERVO_MAX_US       2500 // fully open

// ─── Solenoid expansion header J6 (FUTURE — do not drive) ───────────────────
// GPIO 9–14 are connected to J6 for a future MPQ6610 solenoid daughterboard.
// Initialise to a safe state (output pins low, nFAULT pins as inputs) and
// leave them alone. Do NOT connect external signals to these pins without a
// daughterboard installed — the ESP32 actively holds EN and IN low.
#define PIN_SOL_EXP1_EN      9   // MPQ6610 #1 enable  (future output)
#define PIN_SOL_EXP1_IN     10   // MPQ6610 #1 input   (future output)
#define PIN_SOL_EXP1_NFAULT 11   // MPQ6610 #1 nFAULT  (future input, active-low)
#define PIN_SOL_EXP2_EN     12   // MPQ6610 #2 enable  (future output)
#define PIN_SOL_EXP2_IN     13   // MPQ6610 #2 input   (future output)
#define PIN_SOL_EXP2_NFAULT 14   // MPQ6610 #2 nFAULT  (future input, active-low)

// ─── CAN bus (SN65HVD230 transceiver) ───────────────────────────────────────
// The ESP32 TWAI peripheral produces single-ended TX/RX logic signals.
// The SN65HVD230 converts these to the differential CAN_H / CAN_L bus.
//   TX (ESP32 GPIO 15) → SN65HVD230 pin 1 (D — driver input)
//   RX (ESP32 GPIO 16) ← SN65HVD230 pin 4 (R — receiver output)
#define PIN_CAN_TX         15
#define PIN_CAN_RX         16

#endif // PINS_V3_H
