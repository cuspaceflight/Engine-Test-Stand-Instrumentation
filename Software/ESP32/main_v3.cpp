/*
 * main_v3.cpp — CUSF Actuation Board v2 Firmware
 *
 * Runs on: ESP32-S3-DevKitC-1
 * Talks to:
 *   Laptop GUI      — native USB serial, ASCII protocol (unchanged from v1)
 *   Instr. board    — CAN bus at 500 kbps via SN65HVD230 transceiver
 *
 * What this firmware does:
 *   1. Receives ASCII commands from the GUI ("SRV1:2500\n", "STATUS\n", etc.)
 *      and drives four FT5330M servos via the LEDC PWM peripheral.
 *   2. Polls the CAN RX queue each loop iteration. For every frame received
 *      from the instrumentation board, it decodes the payload and forwards
 *      the data to the GUI as a "SENSOR:..." line — the GUI never needs to
 *      know that sensor data now travels over CAN instead of being read
 *      locally. This is the transparent CAN → USB serial bridge.
 *   3. Sends a CAN heartbeat (ID 0x020) every second so the instrumentation
 *      board knows the actuation board is running.
 *   4. Watches for the instrumentation board's heartbeat (ID 0x010). If it
 *      goes missing for 3 seconds, a fault flag is set and "FAULT:1:0\n" is
 *      sent to the GUI to illuminate the CAN health indicator. "FAULT:1:1\n"
 *      is sent periodically to keep the GUI's link watchdog satisfied and to
 *      show that the CAN link is healthy.
 *
 * Architecture change from v1:
 *   v1: this board read sensors directly (ADS1115 I2C, MAX31856 SPI, HX711)
 *       and drove solenoids via MPQ6610 half-bridge chips on GPIO 4–9.
 *   v2: the instrumentation board now owns all sensor reading and communicates
 *       results via CAN. MPQ6610 chips are removed from this board; solenoid
 *       control is deferred to a future J6 expansion daughterboard. GPIO 4–7
 *       now drive servos (moved from GPIO 10–13).
 *
 * USB serial protocol (unchanged — GUI does not need updating):
 *   GUI → ESP32  "SRVn:500\n" / ":1500\n" / ":2500\n"  set servo pulse
 *                "STATUS\n"                              request status
 *                "SOLn:ON\n" / ":OFF\n"                 not supported, returns ERROR
 *   ESP32 → GUI  "OK\n"                                 command acknowledged
 *                "ERROR:<text>\n"                        command rejected
 *                "FAULT:1:<0|1>\n"                       CAN link health (0=fault)
 *                "SENSOR:<name>:<value>\n"               decoded sensor reading
 *                "STATUS:SRV1:...:CAN:OK\n"              full status
 *
 * Build with PlatformIO:
 *   pio run -e esp32s3_v3              (compile)
 *   pio run -e esp32s3_v3 -t upload    (flash)
 *   pio run -e esp32s3_v3 -t monitor   (open serial monitor)
 */

#include <Arduino.h>
#include "driver/twai.h"   // ESP-IDF CAN (TWAI) driver — bundled, no extra library

#include "pins_v3.h"
#include "calibration.h"


// ═══════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════

// Current servo positions in microseconds. Index 0 is unused —
// servos are 1-indexed throughout to match the J1–J4 labelling on the PCB.
int servo_pulse[5] = {0, SERVO_MIN_US, SERVO_MIN_US, SERVO_MIN_US, SERVO_MIN_US};

// ── CAN health ──────────────────────────────────────────────────────────────
// Start in the faulted state. can_fault is cleared when the first heartbeat
// arrives from the instrumentation board and set again if heartbeats stop.
bool can_fault = true;
unsigned long last_instr_hb_ms = 0;  // millis() when last 0x010 frame arrived

// How long without an instrumentation heartbeat before declaring a fault.
// The instrumentation board sends 0x010 once per second; 3 s gives three
// missed heartbeats before we complain.
const unsigned long CAN_WATCHDOG_MS = 3000;

// Serial input buffer — accumulates characters until a newline arrives
String serial_buffer = "";

// ── Periodic task timers ────────────────────────────────────────────────────
unsigned long last_can_health_report_ms = 0;
const unsigned long CAN_HEALTH_REPORT_MS = 200;  // Keep GUI watchdog satisfied

unsigned long last_heartbeat_send_ms = 0;
const unsigned long HEARTBEAT_SEND_MS = 1000;    // Send our 0x020 once per second

// Firmware version embedded in the CAN heartbeat payload
const uint8_t FW_VERSION_MAJOR = 3;
const uint8_t FW_VERSION_MINOR = 0;


// ═══════════════════════════════════════════════════════════════
// SERVO FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/*
 * Convert a pulse width in microseconds to a LEDC duty value.
 *
 * How it works:
 *   At 50 Hz, one full period is 20 000 µs.
 *   With 14-bit resolution, the duty range is 0–16 383 counts.
 *   So 1 µs maps to 16 384 / 20 000 = 0.8192 counts.
 *
 *   Example: 1 500 µs → duty = round(1500 × 16384 / 20000) = 1229.
 *   The LEDC hardware holds the output pin HIGH for 1229 ticks out of
 *   every 16 384, producing a 1 500 µs pulse at 50 Hz.
 */
uint32_t microseconds_to_duty(int us) {
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;         // 20 000 µs at 50 Hz
    uint32_t max_duty  = (1u << SERVO_RESOLUTION) - 1;    // 16 383 at 14-bit
    return (uint32_t)((float)us / (float)period_us * (float)max_duty);
}


/*
 * Set servo [channel] to [pulse_us] microseconds.
 *
 * channel: 1–4, matching J1–J4 on the PCB.
 * pulse_us: must be one of SERVO_MIN_US (500), SERVO_MID_US (1500),
 *           or SERVO_MAX_US (2500) — these are the three GUI presets.
 *
 * LEDC channel index = servo channel − 1 (LEDC is 0-indexed).
 */
void set_servo(int channel, int pulse_us) {
    if (channel < 1 || channel > 4) return;
    if (pulse_us != SERVO_MIN_US && pulse_us != SERVO_MID_US && pulse_us != SERVO_MAX_US) return;

    servo_pulse[channel] = pulse_us;
    ledcWrite(channel - 1, microseconds_to_duty(pulse_us));
}


// ═══════════════════════════════════════════════════════════════
// CAN BUS FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/*
 * Decode a 4× int16 little-endian CAN payload and emit SENSOR: lines.
 *
 * Used for pressure frames (0x100 and 0x101) and thermocouple frames (0x102).
 *
 * msg:            the received CAN frame (must have at least 8 data bytes)
 * sensor_prefix:  base name sent to the GUI, e.g. "PRESS" or "TEMP"
 * first_channel:  channel number corresponding to msg.data[0..1];
 *                 subsequent pairs map to first_channel+1, +2, +3
 * scale[]:        calibration scale array, indexed by channel number
 * offset[]:       calibration offset array, indexed by channel number
 *
 * The engineering value formula is:  value = raw * scale[ch] + offset[ch]
 * With the default pass-through coefficients (scale=1, offset=0) this just
 * emits raw counts so you can verify CAN reception before calibrating.
 */
void emit_four_int16(const twai_message_t &msg,
                     const char *sensor_prefix,
                     int first_channel,
                     const float *scale,
                     const float *offset) {
    for (int i = 0; i < 4; i++) {
        int16_t raw;
        // memcpy instead of a cast avoids undefined behaviour from
        // reading a potentially unaligned int16 out of the byte array.
        memcpy(&raw, &msg.data[i * 2], sizeof(int16_t));

        int ch = first_channel + i;
        float value = (float)raw * scale[ch] + offset[ch];

        Serial.print("SENSOR:");
        Serial.print(sensor_prefix);
        Serial.print(ch);
        Serial.print(":");
        Serial.println(value, 2);
    }
}


/*
 * Decode and forward one CAN frame to the GUI over USB serial.
 *
 * Frame dispatch table (see full protocol in pins_v3.h header):
 *   0x010  Instrumentation heartbeat — reset watchdog, clear fault if set
 *   0x100  Pressure channels 1–4
 *   0x101  Pressure channels 5–8
 *   0x102  Thermocouple channels 1–4
 *   0x103  Load cell (int32 reading + status byte)
 *   other  Silently ignored (this includes our own 0x020 heartbeat echoed
 *          back on RX because the SN65HVD230 is not in a split TX/RX config)
 */
void handle_can_frame(const twai_message_t &msg) {
    switch (msg.identifier) {

        // ── Instrumentation board heartbeat ──────────────────────────────
        case 0x010: {
            // Payload: uptime_ms u32 | fault_flags u16 | fw_major u8 | fw_minor u8
            // We only need to know a frame arrived; payload inspection can
            // be added later if the instrumentation board's fault flags matter.
            last_instr_hb_ms = millis();

            if (can_fault) {
                // Link just recovered — notify the GUI immediately rather than
                // waiting up to CAN_HEALTH_REPORT_MS for the next periodic tick.
                can_fault = false;
                Serial.println("FAULT:1:1");
            }
            break;
        }

        // ── Pressure channels 1–4 ────────────────────────────────────────
        case 0x100: {
            if (msg.data_length_code < 8) break;
            emit_four_int16(msg, "PRESS", 1, PRESS_SCALE, PRESS_OFFSET);
            break;
        }

        // ── Pressure channels 5–8 ────────────────────────────────────────
        case 0x101: {
            if (msg.data_length_code < 8) break;
            emit_four_int16(msg, "PRESS", 5, PRESS_SCALE, PRESS_OFFSET);
            break;
        }

        // ── Thermocouple channels 1–4 ────────────────────────────────────
        case 0x102: {
            if (msg.data_length_code < 8) break;
            // The MAX31856 linearised temperature register is 19-bit signed,
            // 1 LSB = 1/64 °C. Confirm how the instrumentation firmware packs
            // this into an int16, then update TEMP_SCALE in calibration.h.
            emit_four_int16(msg, "TEMP", 1, TEMP_SCALE, TEMP_OFFSET);
            break;
        }

        // ── Load cell ────────────────────────────────────────────────────
        case 0x103: {
            if (msg.data_length_code < 5) break;

            int32_t raw;
            memcpy(&raw, &msg.data[0], sizeof(int32_t));

            // data[4] is a status byte from the instrumentation board.
            // Non-zero means the HX711 was not ready or reported an error.
            // We forward the reading regardless — if FORCE_SCALE is still
            // the default 1.0 the GUI will show raw counts, which is fine
            // for initial bringup. Log a warning in the serial monitor only.
            uint8_t lc_status = msg.data[4];
            if (lc_status != 0) {
                Serial.printf("WARNING:Load cell status = 0x%02X\r\n", lc_status);
            }

            float force = (float)raw * FORCE_SCALE + FORCE_OFFSET;
            Serial.print("SENSOR:FORCE:");
            Serial.println(force, 2);
            break;
        }

        default:
            // Ignore frames we do not recognise. This includes the echo of
            // our own 0x020 heartbeat (the SN65HVD230 loopback) and any
            // future frames added to the protocol.
            break;
    }
}


/*
 * Drain the TWAI RX queue, dispatching every waiting frame.
 *
 * Called every loop iteration. twai_receive() with a 0-tick timeout returns
 * ESP_ERR_TIMEOUT immediately when the queue is empty, so this never blocks.
 * Processing all pending frames in a tight loop minimises the latency between
 * a CAN frame arriving and the GUI seeing the resulting SENSOR: line.
 */
void poll_can() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        handle_can_frame(msg);
    }
}


/*
 * Send the actuation board's CAN heartbeat (ID 0x020).
 *
 * Payload layout (8 bytes, little-endian):
 *   Bytes 0–3  uptime_ms   uint32  milliseconds since boot
 *   Bytes 4–5  fault_flags uint16  bit 0 = instrumentation heartbeat missing
 *   Byte  6    fw_major    uint8   firmware major version
 *   Byte  7    fw_minor    uint8   firmware minor version
 *
 * The instrumentation board should watch for our 0x020 frames with a symmetric
 * 3-second watchdog and flag a fault if they stop (the bit 0 convention helps
 * it distinguish "actuation board missing" from "instrumentation board missing").
 */
void send_heartbeat() {
    twai_message_t hb = {};      // zero-initialise; standard 11-bit ID, no RTR
    hb.identifier       = 0x020;
    hb.data_length_code = 8;

    uint32_t uptime = (uint32_t)millis();
    memcpy(&hb.data[0], &uptime, 4);

    uint16_t fault_flags = can_fault ? 0x0001u : 0x0000u;  // bit 0 = no instr hb
    memcpy(&hb.data[4], &fault_flags, 2);

    hb.data[6] = FW_VERSION_MAJOR;
    hb.data[7] = FW_VERSION_MINOR;

    // Block at most 10 ms if the TX queue is momentarily full. Under normal
    // bus conditions transmission is nearly instant. Anything other than
    // success or a queue-full timeout is worth logging.
    esp_err_t err = twai_transmit(&hb, pdMS_TO_TICKS(10));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        Serial.printf("WARNING:CAN TX error %d\r\n", err);
    }
}


/*
 * Report CAN bus health to the GUI and run the heartbeat watchdog.
 *
 * Called every CAN_HEALTH_REPORT_MS (200 ms). Emitting a FAULT:1 line on
 * every tick — not just when the state changes — is important: the GUI's
 * link watchdog marks the connection "STALE" if no message arrives for 1500 ms,
 * so these 200 ms ticks are what keep the "● Connected" indicator green.
 *
 * Watchdog logic:
 *   If the CAN link is currently healthy (can_fault == false) but the last
 *   heartbeat was more than CAN_WATCHDOG_MS ago, flip to faulted and log it.
 *   The fault is only cleared inside handle_can_frame() when a 0x010 arrives.
 */
void report_can_health() {
    // Check watchdog — only act when transitioning healthy → faulted
    if (!can_fault && (millis() - last_instr_hb_ms > CAN_WATCHDOG_MS)) {
        can_fault = true;
        // This appears in the GUI log as "<unparsed> WARNING:..." because the
        // protocol parser has no WARNING: handler. That is intentional — it is
        // a firmware diagnostic, not a structured GUI message.
        Serial.println("WARNING:CAN heartbeat from instrumentation board lost");
    }

    // Emit health status unconditionally on every tick
    Serial.println(can_fault ? "FAULT:1:0" : "FAULT:1:1");
}


// ═══════════════════════════════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════════════════════════════

/*
 * Parse and execute one newline-terminated ASCII command from the GUI.
 *
 * Recognised commands:
 *   "SRVn:500"   set servo n (1–4) to 500 µs (closed)
 *   "SRVn:1500"  set servo n to 1500 µs (centre)
 *   "SRVn:2500"  set servo n to 2500 µs (open)
 *   "STATUS"     print current servo positions and CAN health
 *   "SOLn:..."   rejected — no solenoid driver on this board
 *
 * Every accepted command replies with "OK\n".
 * Rejected / malformed commands reply with "ERROR:<reason>\n".
 */
void handle_command(String cmd) {
    cmd.trim();

    // ── Servo commands ──────────────────────────────────────────────────────
    // Format: "SRVn:pppp" where n is channel 1–4 and pppp is pulse in µs.
    // The colon is always at position 4 (SRV + single digit + colon).
    if (cmd.startsWith("SRV") && cmd.indexOf(':') == 4) {
        int channel = cmd.charAt(3) - '0';   // ASCII '1' → int 1
        int pulse   = cmd.substring(5).toInt();

        if (channel >= 1 && channel <= 4 &&
            (pulse == SERVO_MIN_US || pulse == SERVO_MID_US || pulse == SERVO_MAX_US)) {
            set_servo(channel, pulse);
            Serial.println("OK");
        } else {
            Serial.print("ERROR:Invalid servo command: ");
            Serial.println(cmd);
        }
    }

    // ── Solenoid commands — driver chip not present on v2 board ────────────
    // The MPQ6610 chips were removed in the hardware redesign. Solenoid
    // control is deferred to the J6 expansion daughterboard. Return ERROR
    // so the GUI's optimistic-update revert path fires immediately (faster
    // than waiting 500 ms for the ACK timeout).
    else if (cmd.startsWith("SOL")) {
        Serial.println("ERROR:Solenoid driver not fitted on v2 board");
    }

    // ── Status request ──────────────────────────────────────────────────────
    else if (cmd == "STATUS") {
        // Build the response: servo positions + CAN health, colon-delimited.
        // The GUI logs this raw string; it does not field-parse the STATUS line.
        Serial.print("STATUS:");
        for (int i = 1; i <= 4; i++) {
            Serial.print("SRV");
            Serial.print(i);
            Serial.print(":");
            Serial.print(servo_pulse[i]);
            if (i < 4) Serial.print(":");
        }
        Serial.print(":CAN:");
        Serial.println(can_fault ? "FAULT" : "OK");
        Serial.println("OK");
    }

    // ── Unknown command ─────────────────────────────────────────────────────
    else {
        Serial.print("ERROR:Unknown command: ");
        Serial.println(cmd);
    }
}


// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    // ── USB serial ──────────────────────────────────────────────────────────
    // The ESP32-S3 native USB peripheral is activated by the two build flags
    // in platformio.ini: ARDUINO_USB_MODE=1 and ARDUINO_USB_CDC_ON_BOOT=1.
    // Serial automatically uses the native USB interface; no UART pins needed.
    Serial.begin(115200);

    // Block up to 3 s for the USB host to enumerate the CDC device.
    // Without this, the first few log lines are lost before the host is ready.
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) {
        delay(10);
    }

    // ── Servo PWM ───────────────────────────────────────────────────────────
    // LEDC (LED Control) is ESP32's general-purpose PWM peripheral.
    // We configure four independent channels at 50 Hz / 14-bit resolution.
    //
    // v2 GPIO mapping (changed from v1's GPIO 10–13):
    //   LEDC channel 0 → GPIO 4 → Servo 1 (J1)
    //   LEDC channel 1 → GPIO 5 → Servo 2 (J2)
    //   LEDC channel 2 → GPIO 6 → Servo 3 (J3)
    //   LEDC channel 3 → GPIO 7 → Servo 4 (J4)
    const int servo_pins[4] = {PIN_SERVO1, PIN_SERVO2, PIN_SERVO3, PIN_SERVO4};

    for (int ch = 0; ch < 4; ch++) {
        ledcSetup(ch, SERVO_FREQ_HZ, SERVO_RESOLUTION);
        ledcAttachPin(servo_pins[ch], ch);
    }

    // Start all servos in the fully-closed position (500 µs)
    for (int i = 1; i <= 4; i++) {
        set_servo(i, SERVO_MIN_US);
    }

    // ── Solenoid expansion header safe-state ────────────────────────────────
    // GPIO 9–14 are wired to J6, reserved for a future solenoid daughterboard.
    // Drive the EN and IN lines low (disable) and configure nFAULT lines as
    // inputs with pull-ups (so they read HIGH = no fault when unloaded).
    // Do NOT change these GPIO states elsewhere in the code.
    const int exp_out_pins[] = {
        PIN_SOL_EXP1_EN, PIN_SOL_EXP1_IN,
        PIN_SOL_EXP2_EN, PIN_SOL_EXP2_IN
    };
    for (int p : exp_out_pins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }
    pinMode(PIN_SOL_EXP1_NFAULT, INPUT_PULLUP);
    pinMode(PIN_SOL_EXP2_NFAULT, INPUT_PULLUP);

    // ── CAN bus (TWAI driver) ────────────────────────────────────────────────
    // TWAI = Two-Wire Automotive Interface — ESP-IDF's built-in CAN driver.
    // It handles bit-timing, framing, error counting, and bus-off recovery
    // automatically. We just configure it and call twai_receive() / transmit().
    //
    // TWAI_GENERAL_CONFIG_DEFAULT — normal mode (not listen-only or self-test);
    //   TX on GPIO 15 → SN65HVD230 D pin; RX on GPIO 16 ← SN65HVD230 R pin.
    // TWAI_TIMING_CONFIG_500KBITS — standard 500 kbps timing parameters.
    // TWAI_FILTER_CONFIG_ACCEPT_ALL — no hardware ID filtering; we handle
    //   frame selection in software inside handle_can_frame().
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        // Log the ESP-IDF error code; servo control still works without CAN.
        Serial.printf("ERROR:TWAI install failed (esp_err=%d)\r\n", err);
    } else {
        err = twai_start();
        if (err != ESP_OK) {
            Serial.printf("ERROR:TWAI start failed (esp_err=%d)\r\n", err);
        }
    }

    Serial.println("CUSF Actuation Board v3 ready");
    Serial.println("INFO:Waiting for CAN heartbeat from instrumentation board");
    Serial.println("INFO:FAULT:1:0 will clear once first 0x010 frame arrives");
}


// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════

/*
 * Four things happen every iteration, in this order:
 *
 * 1. CAN RX drain — poll_can() dispatches every frame waiting in the TWAI
 *    RX queue. Running this every iteration (not on a timer) minimises the
 *    delay between a CAN frame arriving and the GUI seeing the SENSOR: line.
 *
 * 2. Serial command processing — read characters from USB serial into a
 *    buffer; when a newline arrives, parse and execute the complete command.
 *
 * 3. CAN health report (every 200 ms) — emit FAULT:1:0 or FAULT:1:1 and
 *    run the 3-second heartbeat watchdog. The 200 ms period also keeps the
 *    GUI's 1500 ms link-stale watchdog from triggering when no sensors are
 *    sending (e.g. before the instrumentation board boots).
 *
 * 4. CAN heartbeat send (every 1000 ms) — transmit a 0x020 frame so the
 *    instrumentation board can detect if the actuation board goes offline.
 */
void loop() {
    unsigned long now = millis();

    // ── 1. Drain CAN RX queue ────────────────────────────────────────────────
    poll_can();

    // ── 2. Process incoming serial commands ──────────────────────────────────
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            handle_command(serial_buffer);
            serial_buffer = "";
        } else if (c != '\r') {
            // Accumulate printable characters; skip carriage returns
            serial_buffer += c;
        }
    }

    // ── 3. Periodic CAN health report ────────────────────────────────────────
    if (now - last_can_health_report_ms >= CAN_HEALTH_REPORT_MS) {
        last_can_health_report_ms = now;
        report_can_health();
    }

    // ── 4. Periodic CAN heartbeat send ───────────────────────────────────────
    if (now - last_heartbeat_send_ms >= HEARTBEAT_SEND_MS) {
        last_heartbeat_send_ms = now;
        send_heartbeat();
    }
}
