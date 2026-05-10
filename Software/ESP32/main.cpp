/*
 * main.cpp — CUSF Actuation Board Firmware
 *
 * Runs on: ESP32-S3-DevKitC-1
 * Talks to: Laptop GUI over native USB serial
 *
 * What it does:
 *   1. Receives ASCII commands from the GUI ("SOL1:ON\n", "SRV1:2500\n", etc.)
 *   2. Drives 4x FT5330M servos via LEDC PWM
 *   3. Drives 2x MPQ6610 solenoid drivers via GPIO (when fitted)
 *   4. Reads nFAULT pins and reports faults to the GUI
 *   5. (Future) Reads sensors from the instrumentation board and forwards them
 *
 * Serial protocol:
 *   All messages are ASCII text, terminated with newline (\n).
 *   See protocol.py in the ground-station folder for the full specification.
 *   This firmware is the other side of that contract.
 *
 * Build with PlatformIO:
 *   pio run              (compile)
 *   pio run -t upload    (flash to ESP32)
 *   pio run -t monitor   (open serial monitor)
 */

#include <Arduino.h>
#include "pins.h"


// ═══════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════

// Current servo positions in microseconds (start fully closed)
int servo_pulse[5] = {0, 500, 500, 500, 500};  // Index 0 unused, 1–4 are servos

// Current solenoid states
bool sol_state[3] = {false, false, false};      // Index 0 unused, 1–2 are solenoids

// Timing for periodic status updates
unsigned long last_fault_report = 0;
const unsigned long FAULT_REPORT_INTERVAL_MS = 200;  // Send nFAULT every 200ms

// Timing for periodic sensor forwarding (future)
unsigned long last_sensor_report = 0;
const unsigned long SENSOR_REPORT_INTERVAL_MS = 100;

// Serial input buffer
String serial_buffer = "";


// ═══════════════════════════════════════════════════════════════
// SERVO FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/*
 * Convert a pulse width in microseconds to a LEDC duty value.
 *
 * How this works:
 *   At 50Hz, one full period is 20,000µs (1/50 = 0.02s = 20ms).
 *   With 14-bit resolution, the duty range is 0–16383.
 *   So 1µs = 16384 / 20000 = 0.8192 duty units.
 *
 *   For a 1500µs pulse: duty = 1500 * 16384 / 20000 = 1229
 *   The LEDC hardware then holds the pin HIGH for 1229 out of
 *   16384 ticks per period, producing a 1500µs pulse.
 */
uint32_t microseconds_to_duty(int us) {
    // Period in microseconds at the configured frequency
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;  // 20000µs for 50Hz
    // Maximum duty value at our resolution
    uint32_t max_duty = (1 << SERVO_RESOLUTION) - 1;  // 16383 for 14-bit
    // Convert
    return (uint32_t)((float)us / period_us * max_duty);
}


/*
 * Set a servo to a specific pulse width.
 *
 * channel: 1–4
 * pulse_us: 500, 1500, or 2500
 *
 * The LEDC channel number is (channel - 1), matching the order
 * they were set up in setup().
 */
void set_servo(int channel, int pulse_us) {
    if (channel < 1 || channel > 4) return;
    if (pulse_us != 500 && pulse_us != 1500 && pulse_us != 2500) return;

    servo_pulse[channel] = pulse_us;
    uint32_t duty = microseconds_to_duty(pulse_us);
    ledcWrite(channel - 1, duty);  // LEDC channels are 0-indexed
}


// ═══════════════════════════════════════════════════════════════
// SOLENOID FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/*
 * Turn a solenoid on or off.
 *
 * When SOLENOIDS_FITTED is false, this still toggles the GPIOs
 * (which are connected to empty pads on the PCB). No harm done.
 *
 * Hardware behaviour (when MPQ6610 is soldered):
 *   ON:  EN=HIGH, IN=HIGH → high-side MOSFET on → 12V to solenoid
 *   OFF: EN=LOW           → both MOSFETs off (Hi-Z) → solenoid de-energised
 */
void set_solenoid(int channel, bool on) {
    if (channel < 1 || channel > 2) return;

    sol_state[channel] = on;

    int pin_en, pin_in;
    if (channel == 1) {
        pin_en = PIN_SOL1_EN;
        pin_in = PIN_SOL1_IN;
    } else {
        pin_en = PIN_SOL2_EN;
        pin_in = PIN_SOL2_IN;
    }

    if (on) {
        digitalWrite(pin_en, HIGH);
        digitalWrite(pin_in, HIGH);
    } else {
        digitalWrite(pin_en, LOW);
        // IN state doesn't matter when EN is low (MPQ6610 ignores it)
        digitalWrite(pin_in, LOW);
    }
}


// ═══════════════════════════════════════════════════════════════
// nFAULT READING
// ═══════════════════════════════════════════════════════════════

/*
 * Read the nFAULT pin for a solenoid channel and send the status.
 *
 * The MPQ6610 nFAULT pin is active-low with a 10kΩ pull-up to 3.3V:
 *   HIGH (1) = no fault, everything OK
 *   LOW  (0) = fault detected (OCP, OTP, open load, or UVLO)
 *
 * When MPQ6610 is not fitted (DNP), the pull-up resistor is also
 * not fitted, so the pin floats. We enable the ESP32's internal
 * pull-up to read HIGH (no fault) in this case.
 */
void report_faults() {
    if (!SOLENOIDS_FITTED) {
        // No MPQ6610 soldered — always report OK
        Serial.println("FAULT:1:1");
        Serial.println("FAULT:2:1");
        return;
    }

    // Read the actual nFAULT pins
    // digitalRead returns HIGH (1) for OK, LOW (0) for fault
    int fault1 = digitalRead(PIN_NFAULT1);
    int fault2 = digitalRead(PIN_NFAULT2);

    Serial.print("FAULT:1:");
    Serial.println(fault1);  // 1 = OK, 0 = fault

    Serial.print("FAULT:2:");
    Serial.println(fault2);
}


// ═══════════════════════════════════════════════════════════════
// SENSOR FORWARDING (STUB — future implementation)
// ═══════════════════════════════════════════════════════════════

/*
 * Read sensors from the instrumentation board and forward to the GUI.
 *
 * This is a stub. Once the instrumentation board's digital ICs
 * (ADS1115 over I2C, MAX31856 over SPI, HX711) are wired up,
 * this function will poll them and send SENSOR: lines.
 *
 * For now, it sends placeholder values so the GUI has something to show.
 * Remove or replace these when real sensors are connected.
 */
void report_sensors() {
    // ── Placeholder values ──
    // Replace these with actual reads from the instrumentation board

    // 8 pressure sensors
    for (int i = 1; i <= 8; i++) {
        Serial.print("SENSOR:PRESS");
        Serial.print(i);
        Serial.print(":");
        Serial.println(0.0, 2);  // 0.00 bar — placeholder
    }

    // 4 temperature sensors
    for (int i = 1; i <= 4; i++) {
        Serial.print("SENSOR:TEMP");
        Serial.print(i);
        Serial.print(":");
        Serial.println(0.0, 1);  // 0.0 °C — placeholder
    }

    // 1 force sensor
    Serial.println("SENSOR:FORCE:0.0");
}


// ═══════════════════════════════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════════════════════════════

/*
 * Parse and execute a command received from the GUI.
 *
 * Commands are ASCII strings:
 *   "SOL1:ON"    → energise solenoid 1
 *   "SOL1:OFF"   → de-energise solenoid 1
 *   "SRV1:2500"  → set servo 1 to 2500µs
 *   "STATUS"     → report full status
 *
 * Every valid command gets an "OK" response.
 * Invalid commands get an "ERROR:..." response.
 */
void handle_command(String cmd) {
    cmd.trim();

    // ── Solenoid commands ──
    if (cmd == "SOL1:ON") {
        set_solenoid(1, true);
        Serial.println("OK");
    }
    else if (cmd == "SOL1:OFF") {
        set_solenoid(1, false);
        Serial.println("OK");
    }
    else if (cmd == "SOL2:ON") {
        set_solenoid(2, true);
        Serial.println("OK");
    }
    else if (cmd == "SOL2:OFF") {
        set_solenoid(2, false);
        Serial.println("OK");
    }

    // ── Servo commands ──
    else if (cmd.startsWith("SRV") && cmd.indexOf(':') == 4) {
        // Parse "SRV1:2500" → channel=1, pulse=2500
        int channel = cmd.charAt(3) - '0';  // '1' → 1
        int pulse = cmd.substring(5).toInt();

        if (channel >= 1 && channel <= 4 &&
            (pulse == 500 || pulse == 1500 || pulse == 2500)) {
            set_servo(channel, pulse);
            Serial.println("OK");
        } else {
            Serial.print("ERROR:Invalid servo command: ");
            Serial.println(cmd);
        }
    }

    // ── Status request ──
    else if (cmd == "STATUS") {
        Serial.print("STATUS:");
        Serial.print("SOL1:");
        Serial.print(sol_state[1] ? "ON" : "OFF");
        Serial.print(":SOL2:");
        Serial.print(sol_state[2] ? "ON" : "OFF");
        for (int i = 1; i <= 4; i++) {
            Serial.print(":SRV");
            Serial.print(i);
            Serial.print(":");
            Serial.print(servo_pulse[i]);
        }
        Serial.println();
        Serial.println("OK");
    }

    // ── Unknown command ──
    else {
        Serial.print("ERROR:Unknown command: ");
        Serial.println(cmd);
    }
}


// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    // ── Serial (USB) ──
    // The ESP32-S3 native USB is configured by the build flags in
    // platformio.ini (ARDUINO_USB_CDC_ON_BOOT=1). Serial uses USB
    // automatically — no UART pins needed.
    Serial.begin(115200);

    // Wait for USB connection (up to 3 seconds)
    // This prevents the first few messages from being lost
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {
        delay(10);
    }

    // ── Servo PWM setup ──
    // LEDC = LED Control peripheral, but it's just a general-purpose
    // PWM generator. We configure 4 channels at 50Hz with 14-bit
    // resolution, then attach each to its GPIO pin.
    //
    // Channel 0 → GPIO10 → Servo 1
    // Channel 1 → GPIO11 → Servo 2
    // Channel 2 → GPIO12 → Servo 3
    // Channel 3 → GPIO13 → Servo 4

    int servo_pins[4] = {PIN_SERVO1, PIN_SERVO2, PIN_SERVO3, PIN_SERVO4};

    for (int ch = 0; ch < 4; ch++) {
        ledcSetup(ch, SERVO_FREQ_HZ, SERVO_RESOLUTION);
        ledcAttachPin(servo_pins[ch], ch);
    }

    // Set all servos to closed position (500µs)
    for (int i = 1; i <= 4; i++) {
        set_servo(i, 500);
    }

    // ── Solenoid GPIO setup ──
    // These pins are configured even when MPQ6610 is not fitted.
    // They just toggle empty pads — no harm done.
    pinMode(PIN_SOL1_EN, OUTPUT);
    pinMode(PIN_SOL1_IN, OUTPUT);
    pinMode(PIN_SOL2_EN, OUTPUT);
    pinMode(PIN_SOL2_IN, OUTPUT);
    digitalWrite(PIN_SOL1_EN, LOW);  // Start with solenoids off
    digitalWrite(PIN_SOL1_IN, LOW);
    digitalWrite(PIN_SOL2_EN, LOW);
    digitalWrite(PIN_SOL2_IN, LOW);

    // nFAULT inputs with internal pull-up
    // When MPQ6610 is fitted, the external 10kΩ pull-up does the job
    // and the internal pull-up just adds a parallel path (no problem).
    // When MPQ6610 is NOT fitted, the internal pull-up ensures the
    // pin reads HIGH (no fault) instead of floating randomly.
    pinMode(PIN_NFAULT1, INPUT_PULLUP);
    pinMode(PIN_NFAULT2, INPUT_PULLUP);

    Serial.println("CUSF Actuation Board ready");
    Serial.print("Solenoids fitted: ");
    Serial.println(SOLENOIDS_FITTED ? "YES" : "NO (DNP)");
}


// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════

/*
 * The main loop does three things:
 *
 * 1. Check if the GUI sent a command over USB serial.
 *    Read characters one at a time until we get a newline,
 *    then parse and execute the complete command.
 *
 * 2. Periodically report nFAULT status (every 200ms).
 *
 * 3. Periodically forward sensor data (every 100ms) — stubbed for now.
 */
void loop() {
    // ── 1. Read serial commands ──
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            // Got a complete line — parse it
            handle_command(serial_buffer);
            serial_buffer = "";
        } else if (c != '\r') {
            // Add character to buffer (ignore carriage returns)
            serial_buffer += c;
        }
    }

    // ── 2. Report nFAULT status periodically ──
    unsigned long now = millis();
    if (now - last_fault_report >= FAULT_REPORT_INTERVAL_MS) {
        last_fault_report = now;
        report_faults();
    }

    // ── 3. Forward sensor data periodically ──
    if (now - last_sensor_report >= SENSOR_REPORT_INTERVAL_MS) {
        last_sensor_report = now;
        report_sensors();
    }
}
