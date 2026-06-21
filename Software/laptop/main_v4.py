"""
main_v3.py — CUSF Static Fire Test Stand Ground Station GUI.  (v3.1)

Run with:  python main_v3.py            (real hardware — DEFAULT)
           python main_v3.py --mock     (fake telemetry for UI testing only)

Layout:
    Top bar:        Port + Connect/Disconnect + link/arm status + Re-arm + ABORT
    Left column:    Solenoid controls (inert on v2) + Servo controls
    Right column:   Sensor display + Log panel

v3.1 changes (see CODE_OVERVIEW.md):
    - Mock telemetry is now OFF by default and only enabled with --mock.
      (Previously it ran on every launch, overlaying FAKE data on a real test.)
    - Timing uses a monotonic clock so an NTP/clock step cannot corrupt the
      ack/stale watchdogs mid-test.
    - The GUI sends PING ~2 Hz so the firmware's operator-link fail-safe knows
      the GUI is alive even when the operator is not clicking. If this GUI
      stops (crash / freeze / cable pull) the firmware fails the stand safe.
    - ABORT sends a single atomic ABORT command (firmware drives all servos
      safe and latches). A latched board rejects servo moves until Re-arm.
    - Re-arm button sends RESET; an ARM-state indicator reflects the firmware's
      STATE:ARMED / STATE:FAILSAFE messages.

    DEPLOY TOGETHER: the v3.1 firmware requires this v3.1 GUI (it expects the
    PING keepalive). Do not mix a v3.1 firmware with an older GUI.
"""
import os
import csv
import time
import datetime
import customtkinter as ctk

from serial_comms_v3 import find_ports, SerialConnection
from protocol_v4 import (
    Command,
    SolenoidCommand, ServoCommand, StatusCommand,
    PingCommand, AbortCommand, ResetCommand,
    Message, FaultMessage, SensorMessage, AckMessage, ErrorMessage,
    StatusMessage, StateMessage, UnknownMessage,
)


# ─── Appearance ──────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ─── Tunables ────────────────────────────────────────────────────────────────
LINK_STALE_TIMEOUT_MS = 1500     # No msg for this long → "STALE"
LINK_CHECK_INTERVAL_MS = 200     # Watchdog tick rate
ACK_TIMEOUT_MS = 500             # Pending command must ack within this
ACK_CHECK_INTERVAL_MS = 100      # Pending-command sweeper rate
PING_INTERVAL_MS = 500           # Keepalive to the firmware link watchdog (~2 Hz)

SERVO_LABEL = {500: "Closed", 1500: "Centre", 2500: "Open"}
SERVO_COLOURS = {
    500:  ("#8B0000", "#A52A2A"),   # red
    1500: ("#B8860B", "#DAA520"),   # gold
    2500: ("#006400", "#228B22"),   # green
}


class TestStandGUI(ctk.CTk):
    """Main application window."""

    # ═══════════════════════════════════════════════════════════════
    # CONSTRUCTION
    # ═══════════════════════════════════════════════════════════════

    def __init__(self):
        super().__init__()

        self.title("CUSF Static Fire — Ground Station")
        self.geometry("1240x820")

        # ── State ─────────────────────────────────────────────────
        self.serial: SerialConnection | None = None
        self.sol_states = {1: False, 2: False}
        self.servo_states = {1: 500, 2: 500, 3: 500, 4: 500}

        # Pending commands awaiting an ACK from the ESP32.
        self._pending: list[dict] = []
        self._next_pending_id = 0

        # Watchdog / arm state
        self._last_msg_time_ms: int | None = None
        self._link_state = "disconnected"   # disconnected | live | stale
        self._armed_state = "unknown"       # unknown | ARMED | FAILSAFE

        # ── CSV logging ───────────────────────────────────────────
        os.makedirs("logs", exist_ok=True)
        log_filename = datetime.datetime.now().strftime(
            "logs/log_%Y%m%d_%H%M%S.csv"
        )
        self._log_fh = open(log_filename, "w", newline="")
        self._log_csv = csv.writer(self._log_fh)
        self._log_csv.writerow(["timestamp_utc", "message"])
        self._log_fh.flush()

        # ── Build the GUI ─────────────────────────────────────────
        self._build_top_bar()
        self._build_main_layout()
        self._build_solenoid_controls()
        self._build_servo_controls()
        self._build_sensor_display()
        self._build_log_panel()

        # ── Hooks ─────────────────────────────────────────────────
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Start the watchdog, ack-timeout sweeper, and link keepalive
        self.after(LINK_CHECK_INTERVAL_MS, self._link_watchdog_tick)
        self.after(ACK_CHECK_INTERVAL_MS, self._ack_watchdog_tick)
        self.after(PING_INTERVAL_MS, self._ping_tick)

    # ═══════════════════════════════════════════════════════════════
    # GUI CONSTRUCTION
    # ═══════════════════════════════════════════════════════════════

    def _build_top_bar(self):
        """Connection controls + link status + arm status + Re-arm + ABORT."""
        bar = ctk.CTkFrame(self)
        bar.pack(fill="x", padx=10, pady=(10, 5))

        ctk.CTkLabel(bar, text="Port:").pack(side="left", padx=(10, 5))

        self.port_var = ctk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(
            bar, variable=self.port_var, values=[""], width=180
        )
        self.port_menu.pack(side="left", padx=5)

        ctk.CTkButton(
            bar, text="Refresh", width=80, command=self.refresh_ports
        ).pack(side="left", padx=5)

        self.connect_btn = ctk.CTkButton(
            bar, text="Connect", width=110, command=self.toggle_connection
        )
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ctk.CTkLabel(
            bar, text="● Disconnected", text_color="red", width=130
        )
        self.status_label.pack(side="left", padx=10)

        # Arm / fail-safe state reported by the firmware (STATE: messages)
        self.arm_label = ctk.CTkLabel(
            bar, text="ARM: ---", text_color="gray", width=130,
            font=("", 13, "bold"),
        )
        self.arm_label.pack(side="left", padx=10)

        # ── ABORT button — far right, large, unmissable ──
        self.abort_btn = ctk.CTkButton(
            bar, text="⏻  ABORT  ⏻", width=180, height=48,
            font=("", 18, "bold"),
            fg_color="#B00020", hover_color="#D32F2F",
            text_color="white",
            command=self.abort,
        )
        self.abort_btn.pack(side="right", padx=10, pady=4)

        # ── Re-arm — left of ABORT. Clears a latched fail-safe. ──
        self.rearm_btn = ctk.CTkButton(
            bar, text="Re-arm", width=100, height=48,
            font=("", 14, "bold"),
            fg_color="#1B5E20", hover_color="#2E7D32",
            command=self.rearm,
        )
        self.rearm_btn.pack(side="right", padx=(0, 4), pady=4)

        self.refresh_ports()

    def _build_main_layout(self):
        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.left_col = ctk.CTkFrame(self.main_frame)
        self.left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        self.right_col = ctk.CTkFrame(self.main_frame)
        self.right_col.pack(side="right", fill="both", expand=True, padx=(5, 0))

    def _build_solenoid_controls(self):
        """Solenoid toggle buttons + CAN bus health indicator.

        The MPQ6610 chips are not fitted on the v2 board; the buttons send a
        command that the ESP32 rejects (and the GUI reverts). The FAULT:1 line
        carries CAN-link health in v3, shown next to SOL 1.
        """
        frame = ctk.CTkFrame(self.left_col)
        frame.pack(fill="x", padx=10, pady=(10, 5))

        ctk.CTkLabel(frame, text="Solenoids", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.sol_buttons: dict[int, ctk.CTkButton] = {}
        self.fault_labels: dict[int, ctk.CTkLabel] = {}

        for ch in (1, 2):
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=4)

            btn = ctk.CTkButton(
                row, text=f"SOL {ch}: OFF", width=160,
                fg_color="gray30", hover_color="gray40",
                command=lambda c=ch: self.toggle_solenoid(c),
            )
            btn.pack(side="left")
            self.sol_buttons[ch] = btn

            if ch == 1:
                fault = ctk.CTkLabel(
                    row, text="● CAN Bus ---", text_color="gray", width=160
                )
            else:
                fault = ctk.CTkLabel(
                    row, text="—", text_color="gray50", width=160
                )
            fault.pack(side="left", padx=15)
            self.fault_labels[ch] = fault

        ctk.CTkLabel(
            frame,
            text="(Solenoid driver on future expansion board — controls inactive until fitted)",
            text_color="gray", font=("", 10),
        ).pack(anchor="w", padx=10, pady=(0, 10))

    def _build_servo_controls(self):
        """4 servos × 3 preset buttons + 3 global presets."""
        frame = ctk.CTkFrame(self.left_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Servos", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        # Global presets
        preset_row = ctk.CTkFrame(frame, fg_color="transparent")
        preset_row.pack(fill="x", padx=10, pady=(0, 8))

        for pulse, label in ((500, "All Closed"),
                             (1500, "All Centre"),
                             (2500, "All Open")):
            fg, hover = SERVO_COLOURS[pulse]
            ctk.CTkButton(
                preset_row, text=label, width=120,
                fg_color=fg, hover_color=hover,
                command=lambda p=pulse: self.set_all_servos(p),
            ).pack(side="left", padx=3)

        # Per-servo buttons
        self.servo_buttons: dict[tuple, ctk.CTkBaseClass] = {}

        for ch in (1, 2, 3, 4):
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=2)

            ctk.CTkLabel(row, text=f"Servo {ch}:", width=70).pack(side="left")

            for pulse in (500, 1500, 2500):
                btn = ctk.CTkButton(
                    row, text=SERVO_LABEL[pulse], width=90,
                    fg_color="gray30", hover_color="gray40",
                    command=lambda c=ch, p=pulse: self.set_servo(c, p),
                )
                btn.pack(side="left", padx=3)
                self.servo_buttons[(ch, pulse)] = btn

            status = ctk.CTkLabel(
                row, text="500 µs", width=70, text_color="gray"
            )
            status.pack(side="left", padx=8)
            self.servo_buttons[(ch, "label")] = status

        # Highlight initial state (all closed)
        for ch in (1, 2, 3, 4):
            self._update_servo_buttons(ch, 500)

    def _build_sensor_display(self):
        """8 pressure + 4 temperature + 1 force sensor displays."""
        frame = ctk.CTkScrollableFrame(self.right_col, label_text="Sensors")
        frame.pack(fill="x", padx=10, pady=5)

        self.sensor_labels: dict[str, ctk.CTkLabel] = {}

        sections = [
            ("Pressure (bar)", [(f"PRESS{i}", f"P{i}") for i in range(1, 9)]),
            ("Temperature (°C)", [(f"TEMP{i}", f"T{i}") for i in range(1, 5)]),
            ("Force (N)", [("FORCE", "Thrust")]),
        ]

        for section_title, items in sections:
            ctk.CTkLabel(
                frame, text=section_title,
                font=("", 13, "bold"), text_color="gray70",
            ).pack(anchor="w", padx=8, pady=(8, 2))

            for key, name in items:
                row = ctk.CTkFrame(frame, fg_color="transparent")
                row.pack(fill="x", padx=10, pady=1)

                ctk.CTkLabel(row, text=f"{name}:", width=80).pack(side="left")
                val = ctk.CTkLabel(
                    row, text="---", font=("", 14, "bold"), width=90,
                    anchor="e",
                )
                val.pack(side="left")
                self.sensor_labels[key] = val

    def _build_log_panel(self):
        frame = ctk.CTkFrame(self.right_col)
        frame.pack(fill="both", expand=True, padx=10, pady=5)

        ctk.CTkLabel(frame, text="Log", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )
        self.log_box = ctk.CTkTextbox(frame, height=240, state="disabled")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    # ═══════════════════════════════════════════════════════════════
    # CONNECTION
    # ═══════════════════════════════════════════════════════════════

    def refresh_ports(self):
        ports = find_ports(esp32_only=True)
        if ports:
            self.port_menu.configure(values=ports)
            # Preserve the current selection if it is still present
            if self.port_var.get() not in ports:
                self.port_var.set(ports[0])
        else:
            self.port_menu.configure(values=["No ports found"])
            self.port_var.set("No ports found")

    def toggle_connection(self):
        if self.serial and self.serial.is_connected:
            self._do_disconnect("user")
            return

        port = self.port_var.get()
        if not port or port == "No ports found":
            self.log("No port selected")
            return

        self.serial = SerialConnection(port)
        self.serial.set_callback(self._on_serial_receive)

        if self.serial.connect():
            self.connect_btn.configure(text="Disconnect")
            self._set_link_state("live")
            self._last_msg_time_ms = self._now_ms()
            self.log(f"Connected to {port}")
            # Ask the board for its current state so the indicator updates
            # immediately rather than waiting for the 1 Hz STATE message.
            self.serial.send(StatusCommand())
            self.log("Board boots in FAILSAFE — click Re-arm to enable servos")
        else:
            self.serial = None
            self._set_link_state("disconnected")
            self.status_label.configure(text="● Failed", text_color="orange")
            self.log(f"Failed to connect to {port}")

    def _do_disconnect(self, reason: str):
        if self.serial:
            self.serial.disconnect()
            self.serial = None
        self.connect_btn.configure(text="Connect")
        self._set_link_state("disconnected")
        self._set_arm_state("unknown")
        self._pending.clear()
        self.fault_labels[1].configure(text="● CAN Bus ---", text_color="gray")
        self.log(f"Disconnected ({reason})")

    def _set_link_state(self, state: str):
        self._link_state = state
        if state == "live":
            self.status_label.configure(text="● Connected", text_color="green")
        elif state == "stale":
            self.status_label.configure(text="⚠ STALE", text_color="orange")
        else:  # disconnected
            self.status_label.configure(text="● Disconnected", text_color="red")

    def _set_arm_state(self, state: str, reason: str = ""):
        self._armed_state = state
        if state == "ARMED":
            self.arm_label.configure(text="ARM: ARMED", text_color="#2ECC71")
        elif state == "FAILSAFE":
            self.arm_label.configure(text="ARM: FAILSAFE", text_color="#E74C3C")
        else:
            self.arm_label.configure(text="ARM: ---", text_color="gray")

    # ═══════════════════════════════════════════════════════════════
    # SERIAL DATA HANDLING
    # ═══════════════════════════════════════════════════════════════

    def _on_serial_receive(self, msg: Message):
        """Background thread → bounce to main thread."""
        self.after(0, self._handle_message, msg)

    def _handle_message(self, msg: Message):
        """Main thread. Update the GUI from one received Message."""
        # Any message resets the link watchdog
        self._last_msg_time_ms = self._now_ms()
        if self._link_state == "stale":
            self._set_link_state("live")
            self.log("Link recovered")

        if isinstance(msg, FaultMessage):
            ch = msg.channel
            if ch == 1:
                # Channel 1 carries CAN bus health to the instrumentation board.
                if msg.ok:
                    self.fault_labels[1].configure(
                        text="● CAN Bus OK", text_color="green"
                    )
                else:
                    self.fault_labels[1].configure(
                        text="● CAN Fault!", text_color="red"
                    )
            # Channel 2 is no longer used in v3 — ignore silently.

        elif isinstance(msg, SensorMessage):
            if msg.name in self.sensor_labels:
                self.sensor_labels[msg.name].configure(
                    text=f"{msg.value:.2f}"
                )
            # Sensor lines are too frequent to log every one.

        elif isinstance(msg, StateMessage):
            # Arm / fail-safe state from the firmware. Log only on change.
            prev = self._armed_state
            self._set_arm_state(msg.state, msg.reason)
            if msg.state != prev:
                if msg.state == "FAILSAFE":
                    self.log(
                        f"⚠ Board FAILSAFE ({msg.reason or 'latched'}) "
                        f"— click Re-arm to enable servos"
                    )
                elif msg.state == "ARMED":
                    self.log("Board ARMED — servos enabled")

        elif isinstance(msg, AckMessage):
            # Resolve the OLDEST pending command (FIFO).
            if self._pending:
                self._pending.pop(0)

        elif isinstance(msg, ErrorMessage):
            self.log(f"ESP32 ERROR: {msg.message}")
            # An error effectively acks the oldest pending command too
            # (the ESP32 finished processing it, just unsuccessfully).
            if self._pending:
                pending = self._pending.pop(0)
                self._revert(pending)

        elif isinstance(msg, StatusMessage):
            self.log(f"STATUS: {msg.raw}")

        elif isinstance(msg, UnknownMessage):
            if msg.raw:
                self.log(f"<unparsed> {msg.raw}")

    # ═══════════════════════════════════════════════════════════════
    # COMMAND SEND + ACK TRACKING
    # ═══════════════════════════════════════════════════════════════

    def _send_tracked(self, cmd: Command, on_revert=None) -> bool:
        """Send a command and remember it until we see an OK or it times out.

        on_revert: optional callable(cmd) invoked if the command fails to ack
        (timeout or error) — used to undo optimistic UI updates.
        Returns True if the command was sent on the wire.
        """
        if not self.serial or not self.serial.is_connected:
            self.log(f"Cannot send {cmd.to_str().strip()} — not connected")
            if on_revert:
                on_revert(cmd)
            return False

        if not self.serial.send(cmd):
            self.log(f"Send failed: {cmd.to_str().strip()}")
            if on_revert:
                on_revert(cmd)
            return False

        self._pending.append({
            "id": self._next_pending_id,
            "cmd": cmd,
            "sent_ms": self._now_ms(),
            "on_revert": on_revert,
        })
        self._next_pending_id += 1
        return True

    def _revert(self, pending: dict):
        if pending["on_revert"] is not None:
            pending["on_revert"](pending["cmd"])

    def _ack_watchdog_tick(self):
        """Periodically expire pending commands that never got their OK."""
        now = self._now_ms()
        still_pending = []
        for p in self._pending:
            if now - p["sent_ms"] > ACK_TIMEOUT_MS:
                cmd_str = p["cmd"].to_str().strip()
                self.log(f"⚠ ACK timeout for {cmd_str}")
                self._revert(p)
            else:
                still_pending.append(p)
        self._pending = still_pending

        self.after(ACK_CHECK_INTERVAL_MS, self._ack_watchdog_tick)

    # ═══════════════════════════════════════════════════════════════
    # LINK KEEPALIVE  (so the firmware's operator-link watchdog stays fed)
    # ═══════════════════════════════════════════════════════════════

    def _ping_tick(self):
        """Send a PING to the board ~2 Hz while connected.

        This is what tells the FIRMWARE that the GUI is still alive. If this
        GUI crashes, freezes, or the cable is pulled, the PINGs stop and the
        firmware fails the stand safe after its own timeout.
        """
        if self.serial is not None and self.serial.is_connected:
            try:
                self.serial.send(PingCommand())   # untracked — firmware is silent
            except Exception:
                pass
        self.after(PING_INTERVAL_MS, self._ping_tick)

    # ═══════════════════════════════════════════════════════════════
    # CONNECTION-LOSS WATCHDOG
    # ═══════════════════════════════════════════════════════════════

    def _link_watchdog_tick(self):
        """Mark the link STALE if no message arrived recently.

        The firmware sends FAULT:1 every 200 ms, so 1.5 s of silence
        reliably indicates that the USB connection has been lost.
        """
        if self.serial is not None:
            # Did the reader thread die?
            if not self.serial.is_connected:
                self._do_disconnect("link lost")

            elif self._last_msg_time_ms is not None:
                idle = self._now_ms() - self._last_msg_time_ms
                if idle > LINK_STALE_TIMEOUT_MS and self._link_state == "live":
                    self._set_link_state("stale")
                    self.log(f"⚠ No data from ESP32 for {idle} ms")

        self.after(LINK_CHECK_INTERVAL_MS, self._link_watchdog_tick)

    # ═══════════════════════════════════════════════════════════════
    # CONTROL ACTIONS
    # ═══════════════════════════════════════════════════════════════

    def toggle_solenoid(self, channel: int):
        """Optimistic update + revert on no-ack.

        The v2 board has no solenoid driver, so the ESP32 returns an ERROR
        immediately; the ErrorMessage handler pops the pending command and
        reverts the optimistic state change.
        """
        new_state = not self.sol_states[channel]
        self.sol_states[channel] = new_state
        self._paint_solenoid(channel, new_state)

        cmd = SolenoidCommand(channel, new_state)

        def revert(_cmd):
            self.sol_states[channel] = not new_state
            self._paint_solenoid(channel, not new_state)
            self.log(f"SOL {channel} reverted (no ack)")

        sent = self._send_tracked(cmd, on_revert=revert)
        if sent:
            self.log(f"SOL {channel} → {'ON' if new_state else 'OFF'}")

    def _paint_solenoid(self, channel: int, on: bool):
        if on:
            self.sol_buttons[channel].configure(
                text=f"SOL {channel}: ON",
                fg_color="green", hover_color="darkgreen",
            )
        else:
            self.sol_buttons[channel].configure(
                text=f"SOL {channel}: OFF",
                fg_color="gray30", hover_color="gray40",
            )

    def set_servo(self, channel: int, pulse_us: int):
        """Optimistic update + revert on no-ack.

        If the board is in FAILSAFE it replies ERROR; the revert path then
        restores the previous button state and the ERROR is logged.
        """
        previous = self.servo_states[channel]
        self.servo_states[channel] = pulse_us
        self._update_servo_buttons(channel, pulse_us)

        cmd = ServoCommand(channel, pulse_us)

        def revert(_cmd):
            self.servo_states[channel] = previous
            self._update_servo_buttons(channel, previous)
            self.log(f"Servo {channel} reverted (no ack / refused)")

        sent = self._send_tracked(cmd, on_revert=revert)
        if sent:
            self.log(
                f"Servo {channel} → {SERVO_LABEL[pulse_us]} ({pulse_us} µs)"
            )

    def set_all_servos(self, pulse_us: int):
        for ch in (1, 2, 3, 4):
            self.set_servo(ch, pulse_us)

    def _update_servo_buttons(self, channel: int, active_pulse: int):
        for pulse in (500, 1500, 2500):
            btn = self.servo_buttons[(channel, pulse)]
            if pulse == active_pulse:
                fg, hover = SERVO_COLOURS[pulse]
                btn.configure(fg_color=fg, hover_color=hover)
            else:
                btn.configure(fg_color="gray30", hover_color="gray40")

        self.servo_buttons[(channel, "label")].configure(
            text=f"{active_pulse} µs"
        )

    # ═══════════════════════════════════════════════════════════════
    # ABORT  /  RE-ARM
    # ═══════════════════════════════════════════════════════════════

    def abort(self):
        """Atomic abort — one ABORT command; firmware drives all servos safe.

        Optimistic UI shows everything closed, which matches the firmware's
        default all-closed safe state. (If you reconfigure a channel's safe
        state to OPEN — e.g. a vent — update this assumption to match.)
        The ABORT command is NOT reverted on failure: an abort that did not
        reach the board must never re-open anything in the GUI.
        """
        self.log("⏻ ABORT triggered")

        for ch in (1, 2):
            if self.sol_states[ch]:
                self.sol_states[ch] = False
                self._paint_solenoid(ch, False)

        for ch in (1, 2, 3, 4):
            self.servo_states[ch] = 500
            self._update_servo_buttons(ch, 500)

        self._send_tracked(AbortCommand(), on_revert=self._abort_failure_log)

    def _abort_failure_log(self, cmd: Command):
        self.log(f"⚠ ABORT command not acknowledged: {cmd.to_str().strip()}")

    def rearm(self):
        """Clear a latched fail-safe so servo commands are accepted again."""
        self.log("Re-arm requested")
        self._send_tracked(
            ResetCommand(),
            on_revert=lambda c: self.log("⚠ Re-arm not acknowledged"),
        )

    # ═══════════════════════════════════════════════════════════════
    # LOGGING
    # ═══════════════════════════════════════════════════════════════

    def log(self, message: str):
        """Append a timestamped message to the on-screen log + CSV file.

        Screen shows local wall-clock for the operator's convenience; the CSV
        records UTC so it correlates cleanly with the instrumentation log.
        """
        now_local = datetime.datetime.now()
        screen_ts = now_local.strftime("%H:%M:%S.%f")[:-3]
        utc_ts = datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="milliseconds"
        )

        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{screen_ts}] {message}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

        try:
            self._log_csv.writerow([utc_ts, message])
            self._log_fh.flush()
        except (ValueError, OSError):
            pass   # file closed during shutdown — ignore

    # ═══════════════════════════════════════════════════════════════
    # SHUTDOWN
    # ═══════════════════════════════════════════════════════════════

    def _on_close(self):
        """Tidy shutdown — disconnect serial, close log file, exit."""
        try:
            if self.serial and self.serial.is_connected:
                self.serial.disconnect()
        except Exception:
            pass

        try:
            self._log_fh.close()
        except Exception:
            pass

        self.destroy()

    # ═══════════════════════════════════════════════════════════════
    # HELPERS
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _now_ms() -> int:
        # Monotonic: immune to wall-clock/NTP steps that would otherwise
        # corrupt the ack and stale-link timers mid-test.
        return int(time.monotonic() * 1000)


# ═══════════════════════════════════════════════════════════════════
# MOCK DATA (test the GUI without an ESP32) — enabled only with --mock
# ═══════════════════════════════════════════════════════════════════

def add_mock_data(app: TestStandGUI):
    """Inject fake messages every 200 ms. UI testing / screenshots only."""
    import random

    def tick():
        for i in range(1, 9):
            app._handle_message(SensorMessage(
                f"PRESS{i}", round(2.5 + random.gauss(0, 0.1), 2)
            ))
        for i in range(1, 5):
            app._handle_message(SensorMessage(
                f"TEMP{i}", round(22 + i * 5 + random.gauss(0, 0.5), 1)
            ))
        app._handle_message(SensorMessage(
            "FORCE", round(max(0, 140 + random.gauss(0, 5)), 1)
        ))
        app._handle_message(FaultMessage(1, random.random() > 0.05))
        app._handle_message(StateMessage("ARMED", ""))
        app.after(200, tick)

    app._set_link_state("live")
    app.after(500, tick)


# ═══════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CUSF Ground Station GUI")
    parser.add_argument(
        "--mock", action="store_true",
        help="Inject FAKE telemetry for UI testing. NEVER use for a real test.",
    )
    args = parser.parse_args()

    app = TestStandGUI()

    if args.mock:
        app.log("⚠ MOCK MODE — telemetry is FAKE. Do not use for a real test.")
        add_mock_data(app)

    app.mainloop()