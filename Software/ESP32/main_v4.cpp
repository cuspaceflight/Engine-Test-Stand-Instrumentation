/*
 * main_v3.cpp — CUSF Actuation Board v2 Firmware  (v3.1)
 *
 * Runs on: ESP32-S3-DevKitC-1
 * Talks to:
 *   Laptop GUI      — native USB serial, ASCII protocol
 *   Instr. board    — CAN bus at 500 kbps via SN65HVD230 transceiver
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * v3.1 SAFETY CHANGES vs the version you uploaded (all reviewed below):
 *   [S1] Operator-link fail-safe. If the firmware does not hear from the GUI
 *        within GUI_LINK_TIMEOUT_MS, it drives every servo to its configured
 *        SAFE position and LATCHES a fail-safe state. Servo move commands are
 *        rejected until an explicit RESET re-arms it. This closes the single
 *        biggest gap: previously a crashed/yanked laptop left valves frozen
 *        wherever they were last commanded. REQUIRES the v3.1 GUI (it PINGs).
 *   [S2] Atomic ABORT / RESET commands. ABORT = one frame → all servos safe +
 *        latch. RESET = re-arm. Replaces four separate SRV:500 sends.
 *   [S3] CAN bus-off detection + recovery. The previous comment claimed TWAI
 *        recovers automatically — it does NOT. ESP-IDF requires an explicit
 *        twai_initiate_recovery() + twai_start(). Without this the board goes
 *        permanently CAN-silent after any bus-off (e.g. a transient short or
 *        EMI burst near the stand). Now handled.
 *   [S4] Larger CAN RX queue (32) so loop jitter does not drop sensor frames.
 *   [S5] Optional hardware task watchdog: a hung loop now resets the chip,
 *        and because setup() drives servos safe, a reset fails the stand safe.
 *
 * The servo/LEDC code, the CAN decode path, and the USB protocol are otherwise
 * UNCHANGED. See CODE_OVERVIEW.md for the full change list and rationale.
 *
 * ⚠ VERSION NOTE — READ BEFORE BUILDING:
 *   This firmware targets Arduino-ESP32 core 2.0.x:
 *     - LEDC uses ledcSetup() / ledcAttachPin() / ledcWrite(channel, duty)
 *     - esp_task_wdt_init(timeout_s, panic) takes two scalar args
 *   Core 3.0.x REMOVED that LEDC API (use ledcAttach()/ledcWrite(pin,duty))
 *   and CHANGED esp_task_wdt_init() to take a config struct. If your build
 *   fails to compile, you are almost certainly on core 3.x — either pin the
 *   platform to a 2.0.x-providing release in platformio.ini (recommended for
 *   reproducibility) or migrate those two APIs. Confirm your installed core
 *   from the build log line: framework-arduinoespressif32 @ x.y.z
 *
 * Build with PlatformIO:
 *   pio run -e esp32s3_v3              (compile  ← FIRST validation gate)
 *   pio run -e esp32s3_v3 -t upload    (flash)
 *   pio run -e esp32s3_v3 -t monitor   (open serial monitor)
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"     // ESP-IDF CAN (TWAI) driver — bundled, no extra library

// [S5] Hardware task watchdog. Set to 0 if it will not compile on your core
//      version (the API differs on core 3.x — see VERSION NOTE above).
//      MUST be considered part of the safety case for any live test.
#define ENABLE_TASK_WDT  1
#if ENABLE_TASK_WDT
  #include "esp_task_wdt.h"
#endif

// [S1] Operator-link fail-safe. MUST be 1 for any test involving propellant or
//      pressurisation. Set to 0 ONLY for isolated bench servo testing with a
//      plain serial terminal that cannot send PING (otherwise the board will
//      correctly latch safe ~1.5 s after the last command).
#define ENABLE_LINK_FAILSAFE  1

#include "pins_v3.h"
#include "calibration.h"


// ═══════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════

// Current servo positions in microseconds. Index 0 is unused —
// servos are 1-indexed throughout to match the J1–J4 labelling on the PCB.
int servo_pulse[5] = {0, SERVO_MIN_US, SERVO_MIN_US, SERVO_MIN_US, SERVO_MIN_US};

// ════════════════════════════════════════════════════════════════════════════
//  [S1/S2] SAFE STATE TABLE — SAFETY CRITICAL — REVIEW PER CHANNEL
// ════════════════════════════════════════════════════════════════════════════
//  The "safe" position of a valve is NOT generic. It is defined by the
//  plumbing P&ID:
//      - A main FUEL or OXIDISER valve is safe when CLOSED  (500 µs here).
//      - A VENT valve is safe when OPEN                     (2500 µs here).
//  A single "all closed" default is therefore an ASSUMPTION. This table is what
//  every servo is driven to on boot, on operator-link loss, and on ABORT.
//  The defaults assume all four channels are normally-closed valves. CONFIRM
//  EACH ENTRY against the actual plumbing before any pressurised test.
//
//  NOTE: the GUI's ABORT optimistically shows "all closed". If you change any
//  entry below to a non-closed safe state, update the GUI display assumption
//  to match (see CODE_OVERVIEW.md).
// ════════════════════════════════════════════════════════════════════════════
const int SERVO_SAFE_US[5] = {
    0,            // [0] unused
    SERVO_MIN_US, // CH1 — TODO confirm against P&ID
    SERVO_MIN_US, // CH2 — TODO confirm against P&ID
    SERVO_MIN_US, // CH3 — TODO confirm against P&ID
    SERVO_MIN_US, // CH4 — TODO confirm against P&ID
};

// ── CAN health ──────────────────────────────────────────────────────────────
// Start in the faulted state. can_fault is cleared when the first heartbeat
// arrives from the instrumentation board and set again if heartbeats stop.
bool can_fault = true;
unsigned long last_instr_hb_ms = 0;  // millis() when last 0x010 frame arrived

// How long without an instrumentation heartbeat before declaring a fault.
const unsigned long CAN_WATCHDOG_MS = 3000;

// ── [S1] Operator-link (USB/GUI) watchdog + latched fail-safe ────────────────
// last_gui_cmd_ms advances on EVERY line received from the GUI (any host line
// means a host is connected and talking). The v3.1 GUI also sends PING ~2 Hz so
// the link stays alive even when the operator is not clicking. If the gap
// exceeds GUI_LINK_TIMEOUT_MS the operator link is presumed lost.
unsigned long last_gui_cmd_ms = 0;
const unsigned long GUI_LINK_TIMEOUT_MS = 1500;   // ~3 missed 500 ms pings

// link_failsafe latches TRUE on boot, on ABORT, and on link loss. While true,
// servo MOVE commands are rejected. Cleared only by an explicit RESET while the
// link is alive — we do NOT auto-resume actuation after a dropout, because for
// propellant valves silently re-opening on link recovery is the worse hazard.
bool link_failsafe = true;

// Serial input buffer — accumulates characters until a newline arrives
String serial_buffer = "";

// ── Periodic task timers ────────────────────────────────────────────────────
unsigned long last_can_health_report_ms = 0;
const unsigned long CAN_HEALTH_REPORT_MS = 200;  // Keep GUI watchdog satisfied

unsigned long last_heartbeat_send_ms = 0;
const unsigned long HEARTBEAT_SEND_MS = 1000;    // Send our 0x020 once per second

// Firmware version embedded in the CAN heartbeat payload
const uint8_t FW_VERSION_MAJOR = 3;
const uint8_t FW_VERSION_MINOR = 1;


// Forward declarations for helpers defined lower down
void emit_state();
void enter_failsafe(const char *reason);


// ═══════════════════════════════════════════════════════════════
// SERVO FUNCTIONS  (unchanged from your v3 — LEDC core 2.x API)
// ═══════════════════════════════════════════════════════════════

/*
 * Convert a pulse width in microseconds to a LEDC duty value.
 *   At 50 Hz, one full period is 20 000 µs. With 14-bit resolution the duty
 *   range is 0–16 383 counts, so 1 µs maps to 16 383 / 20 000 ≈ 0.8192 counts.
 */
uint32_t microseconds_to_duty(int us) {
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;         // 20 000 µs at 50 Hz
    uint32_t max_duty  = (1u << SERVO_RESOLUTION) - 1;    // 16 383 at 14-bit
    return (uint32_t)((float)us / (float)period_us * (float)max_duty);
}

/*
 * Set servo [channel] (1–4) to [pulse_us]. pulse_us must be one of the three
 * GUI presets (500 / 1500 / 2500). LEDC channel index = servo channel − 1.
 */
void set_servo(int channel, int pulse_us) {
    if (channel < 1 || channel > 4) return;
    if (pulse_us != SERVO_MIN_US && pulse_us != SERVO_MID_US && pulse_us != SERVO_MAX_US) return;

    servo_pulse[channel] = pulse_us;
    ledcWrite(channel - 1, microseconds_to_duty(pulse_us));
}

/*
 * [S2] Drive every servo to its configured SAFE position (SERVO_SAFE_US).
 * Used by enter_failsafe(). Writes the LEDC duty directly so it works even
 * when the safe position is not one of the three presets set_servo() allows.
 */
void drive_all_servos_safe() {
    for (int ch = 1; ch <= 4; ch++) {
        servo_pulse[ch] = SERVO_SAFE_US[ch];
        ledcWrite(ch - 1, microseconds_to_duty(SERVO_SAFE_US[ch]));
    }
}


// ═══════════════════════════════════════════════════════════════
// [S1/S2] FAIL-SAFE STATE MACHINE
// ═══════════════════════════════════════════════════════════════

/*
 * Emit the current arm/fail-safe state to the GUI. Sent on every transition
 * and once per second (so a GUI connecting mid-stream learns the state). Uses
 * a dedicated STATE: prefix rather than ERROR:/FAULT: so it never disturbs the
 * GUI's OK/ERROR acknowledgement bookkeeping.
 */
void emit_state() {
    if (link_failsafe) {
        Serial.println("STATE:FAILSAFE:active");
    } else {
        Serial.println("STATE:ARMED");
    }
}

/*
 * Enter the latched fail-safe state: drive all servos safe, latch, notify.
 * Idempotent — calling it while already latched just re-drives the safe state.
 */
void enter_failsafe(const char *reason) {
    drive_all_servos_safe();
    bool was_armed = !link_failsafe;
    link_failsafe = true;
    if (was_armed) {
        Serial.print("WARNING:FAILSAFE entered (");
        Serial.print(reason);
        Serial.println(") — servos driven safe, RESET required to re-arm");
    }
    Serial.print("STATE:FAILSAFE:");
    Serial.println(reason);
}

/*
 * Re-arm (clear the latch). Only honoured while the operator link is alive,
 * which by construction it is whenever a RESET command has just been received.
 */
void arm_system() {
    link_failsafe = false;
    Serial.println("STATE:ARMED");
    Serial.println("INFO:System armed — servo commands accepted");
}


// ═══════════════════════════════════════════════════════════════
// CAN BUS FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/*
 * Decode a 4× int16 little-endian CAN payload and emit SENSOR: lines.
 * Used for pressure (0x100/0x101) and thermocouple (0x102) frames.
 * value = raw * scale[ch] + offset[ch]; defaults pass raw counts through.
 */
void emit_four_int16(const twai_message_t &msg,
                     const char *sensor_prefix,
                     int first_channel,
                     const float *scale,
                     const float *offset) {
    for (int i = 0; i < 4; i++) {
        int16_t raw;
        // memcpy avoids UB from reading a possibly-unaligned int16 from bytes.
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
 *   0x010  Instrumentation heartbeat — reset watchdog, clear fault if set
 *   0x100  Pressure channels 1–4
 *   0x101  Pressure channels 5–8
 *   0x102  Thermocouple channels 1–4
 *   0x103  Load cell (int32 reading + status byte)
 *   other  Ignored (includes our own 0x020 echoed on RX)
 */
void handle_can_frame(const twai_message_t &msg) {
    switch (msg.identifier) {

        // ── Instrumentation board heartbeat ──────────────────────────────
        case 0x010: {
            last_instr_hb_ms = millis();
            if (can_fault) {
                can_fault = false;
                Serial.println("FAULT:1:1");   // notify recovery immediately
            }
            break;
        }

        // ── Pressure channels 1–4 ────────────────────────────────────────
        case 0x100:
            if (msg.data_length_code < 8) break;
            emit_four_int16(msg, "PRESS", 1, PRESS_SCALE, PRESS_OFFSET);
            break;

        // ── Pressure channels 5–8 ────────────────────────────────────────
        case 0x101:
            if (msg.data_length_code < 8) break;
            emit_four_int16(msg, "PRESS", 5, PRESS_SCALE, PRESS_OFFSET);
            break;

        // ── Thermocouple channels 1–4 ────────────────────────────────────
        case 0x102:
            if (msg.data_length_code < 8) break;
            emit_four_int16(msg, "TEMP", 1, TEMP_SCALE, TEMP_OFFSET);
            break;

        // ── Load cell ────────────────────────────────────────────────────
        case 0x103: {
            if (msg.data_length_code < 5) break;
            int32_t raw;
            memcpy(&raw, &msg.data[0], sizeof(int32_t));
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
            break;   // unrecognised / our own echoed heartbeat
    }
}

/*
 * Drain the TWAI RX queue, dispatching every waiting frame. Non-blocking
 * (0-tick timeout). Draining fully each loop minimises CAN→USB latency.
 */
void poll_can() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        handle_can_frame(msg);
    }
}

/*
 * Send the actuation board's CAN heartbeat (ID 0x020). 8-byte LE payload:
 *   [0..3] uptime_ms u32 | [4..5] fault_flags u16 | [6] fw_major | [7] fw_minor
 *   fault_flags bit 0 = instrumentation heartbeat missing.
 */
void send_heartbeat() {
    twai_message_t hb = {};      // zero-init; standard 11-bit ID, no RTR
    hb.identifier       = 0x020;
    hb.data_length_code = 8;

    uint32_t uptime = (uint32_t)millis();
    memcpy(&hb.data[0], &uptime, 4);
    uint16_t fault_flags = can_fault ? 0x0001u : 0x0000u;
    memcpy(&hb.data[4], &fault_flags, 2);
    hb.data[6] = FW_VERSION_MAJOR;
    hb.data[7] = FW_VERSION_MINOR;

    esp_err_t err = twai_transmit(&hb, pdMS_TO_TICKS(10));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        Serial.printf("WARNING:CAN TX error %d\r\n", err);
    }
}

/*
 * [S3] CAN controller-state service. ESP-IDF TWAI does NOT auto-recover from
 * bus-off (the previous comment was wrong). On bus-off we must explicitly
 * initiate recovery; recovery ends in the STOPPED state, from which we must
 * call twai_start() again. Checked here on the 200 ms cadence.
 */
void service_can_state() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    if (status.state == TWAI_STATE_BUS_OFF) {
        Serial.println("WARNING:CAN bus-off — initiating recovery");
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        // Reached after a recovery completes (or if never started). Restart.
        if (twai_start() == ESP_OK) {
            Serial.println("INFO:CAN restarted after recovery");
        }
    }
}

/*
 * Report CAN bus health to the GUI and run the heartbeat watchdog.
 * Emitting FAULT:1 every tick (not only on change) is deliberate: it is the
 * steady stream that keeps the GUI's link-stale watchdog from tripping.
 */
void report_can_health() {
    if (!can_fault && (millis() - last_instr_hb_ms > CAN_WATCHDOG_MS)) {
        can_fault = true;
        Serial.println("WARNING:CAN heartbeat from instrumentation board lost");
    }
    Serial.println(can_fault ? "FAULT:1:0" : "FAULT:1:1");
}


// ═══════════════════════════════════════════════════════════════
// [S1] OPERATOR-LINK WATCHDOG
// ═══════════════════════════════════════════════════════════════

/*
 * If the GUI has gone quiet for longer than GUI_LINK_TIMEOUT_MS, presume the
 * operator link is lost and fail safe. Only acts on the armed→failsafe edge.
 */
void check_link_watchdog() {
#if ENABLE_LINK_FAILSAFE
    if (!link_failsafe && (millis() - last_gui_cmd_ms > GUI_LINK_TIMEOUT_MS)) {
        enter_failsafe("linkloss");
    }
#endif
}


// ═══════════════════════════════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════════════════════════════

/*
 * Parse and execute one newline-terminated ASCII command from the GUI.
 *   SRVn:500|1500|2500   move servo n (rejected while in fail-safe)
 *   STATUS               report servo positions + CAN health + arm state
 *   PING                 liveness keepalive — SILENT (no reply) by design
 *   ABORT                drive all servos safe + latch fail-safe        [S2]
 *   RESET                re-arm after abort/link-loss                   [S2]
 *   SOLn:...             rejected — no solenoid driver on this board
 * Accepted commands reply "OK"; rejected/malformed reply "ERROR:<reason>".
 */
void handle_command(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // [S1] Any line from the host proves the operator link is alive.
    last_gui_cmd_ms = millis();

    // ── PING ──────────────────────────────────────────────────────────────
    // Silent on purpose: it must not emit OK, or it would pop an unrelated
    // command off the GUI's FIFO ack queue. The timestamp update above is its
    // entire effect.
    if (cmd == "PING") {
        return;
    }

    // ── ABORT ─────────────────────────────────────────────────────────────
    if (cmd == "ABORT") {
        enter_failsafe("abort");
        Serial.println("OK");
        return;
    }

    // ── RESET (re-arm) ──────────────────────────────────────────────────────
    if (cmd == "RESET") {
        arm_system();
        Serial.println("OK");
        return;
    }

    // ── Servo commands ──────────────────────────────────────────────────────
    // Format: "SRVn:pppp" — colon always at index 4 (SRV + digit + colon).
    if (cmd.startsWith("SRV") && cmd.indexOf(':') == 4) {
        int channel = cmd.charAt(3) - '0';
        int pulse   = cmd.substring(5).toInt();

        // [S1] Refuse to move servos while latched safe.
        if (link_failsafe) {
            Serial.println("ERROR:FAILSAFE active — send RESET to re-arm before moving servos");
            return;
        }

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
    else if (cmd.startsWith("SOL")) {
        Serial.println("ERROR:Solenoid driver not fitted on v2 board");
    }

    // ── Status request ──────────────────────────────────────────────────────
    else if (cmd == "STATUS") {
        Serial.print("STATUS:");
        for (int i = 1; i <= 4; i++) {
            Serial.print("SRV");
            Serial.print(i);
            Serial.print(":");
            Serial.print(servo_pulse[i]);
            if (i < 4) Serial.print(":");
        }
        Serial.print(":CAN:");
        Serial.print(can_fault ? "FAULT" : "OK");
        Serial.print(":STATE:");
        Serial.println(link_failsafe ? "FAILSAFE" : "ARMED");
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
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) {
        delay(10);
    }

    // ── Servo PWM (LEDC core-2.x API — see VERSION NOTE) ─────────────────────
    const int servo_pins[4] = {PIN_SERVO1, PIN_SERVO2, PIN_SERVO3, PIN_SERVO4};
    for (int ch = 0; ch < 4; ch++) {
        ledcSetup(ch, SERVO_FREQ_HZ, SERVO_RESOLUTION);
        ledcAttachPin(servo_pins[ch], ch);
    }
    // Power up with every servo in its configured SAFE position. Combined with
    // the task watchdog [S5], any reset therefore leaves the stand safe.
    drive_all_servos_safe();

    // ── Solenoid expansion header safe-state (J6, future daughterboard) ──────
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
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;   // [S4] tolerate loop jitter at high sensor rates
    g_config.tx_queue_len = 16;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("ERROR:TWAI install failed (esp_err=%d)\r\n", err);
    } else {
        err = twai_start();
        if (err != ESP_OK) {
            Serial.printf("ERROR:TWAI start failed (esp_err=%d)\r\n", err);
        }
    }

    // ── [S5] Hardware task watchdog ──────────────────────────────────────────
#if ENABLE_TASK_WDT
    // core-2.x signature: esp_task_wdt_init(timeout_seconds, panic_on_timeout)
    esp_task_wdt_init(3, true);   // reset the chip if loop() stalls > 3 s
    esp_task_wdt_add(NULL);       // subscribe the Arduino loop task
#endif

    // ── [S1] Operator-link watchdog + initial fail-safe latch ────────────────
    last_gui_cmd_ms = millis();   // grace period before the first timeout check
    link_failsafe   = true;       // boot latched-safe; operator must RESET to arm

    Serial.println("CUSF Actuation Board v3.1 ready");
    Serial.println("INFO:Booted in FAILSAFE — send RESET (GUI 'Re-arm') to enable servos");
    Serial.println("INFO:Waiting for CAN heartbeat from instrumentation board");
    emit_state();
}


// ═══════════════════════════════════════════════════════════════
// MAIN LOOP  (non-blocking — no delay() anywhere)
// ═══════════════════════════════════════════════════════════════

void loop() {
    unsigned long now = millis();

    // 1. Drain CAN RX queue
    poll_can();

    // 2. Process incoming serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            handle_command(serial_buffer);
            serial_buffer = "";
        } else if (c != '\r') {
            serial_buffer += c;
            // Guard against an unterminated flood filling RAM
            if (serial_buffer.length() > 128) serial_buffer = "";
        }
    }

    // 3. [S1] Operator-link watchdog
    check_link_watchdog();

    // 4. Periodic CAN health report + [S3] bus-off service (every 200 ms)
    if (now - last_can_health_report_ms >= CAN_HEALTH_REPORT_MS) {
        last_can_health_report_ms = now;
        service_can_state();
        report_can_health();
    }

    // 5. Periodic CAN heartbeat send + arm-state echo (every 1000 ms)
    if (now - last_heartbeat_send_ms >= HEARTBEAT_SEND_MS) {
        last_heartbeat_send_ms = now;
        send_heartbeat();
        emit_state();   // 1 Hz STATE so a mid-stream GUI learns arm state
    }

    // 6. [S5] Feed the task watchdog (only reached if the loop is healthy)
#if ENABLE_TASK_WDT
    esp_task_wdt_reset();
#endif
}