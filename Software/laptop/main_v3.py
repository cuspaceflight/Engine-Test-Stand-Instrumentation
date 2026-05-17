"""
main_v3.py — CUSF Static Fire Test Stand Ground Station GUI.

Run with:  python main_v3.py

Layout:
    Top bar:        Port selection + Connect/Disconnect + ABORT button
    Left column:    Solenoid controls + Servo controls
    Right column:   Sensor display + Log panel

Controls:
    - 2 solenoid valve buttons (inactive on v2 board — no MPQ6610 driver;
      buttons return an error and revert automatically)
    - 4 servos with 3 preset buttons each (Closed/Centre/Open)
    - 3 global servo presets (All Closed / All Centre / All Open)
    - Big red ABORT button: closes all servos in one click

Displays:
    - CAN bus health indicator (channel 1 FAULT message, replaces old nFAULT)
    - 8 pressure + 4 temperature + 1 force sensor readings
    - Connection-loss watchdog (link is "stale" if no message in 1.5 s)
    - Timestamped log (screen + CSV file in logs/)

Architecture change vs v2:
    Sensor data now arrives via CAN bus from a separate instrumentation board.
    The ESP32 firmware decodes CAN frames and forwards them as SENSOR: lines
    over USB — the GUI receives identical messages to v1/v2. The FAULT:1:
    message now reports CAN link health rather than MPQ6610 nFAULT status.

Testing without hardware:
    add_mock_data(app) at the bottom injects fake data so you can test the
    GUI without an ESP32. Comment it out for real hardware use.
"""
import os
import csv
import datetime
import customtkinter as ctk

from serial_comms_v3 import find_ports, SerialConnection
from protocol_v3 import (
    Command,
    SolenoidCommand, ServoCommand, StatusCommand,
    Message, FaultMessage, SensorMessage, AckMessage, ErrorMessage,
    StatusMessage, UnknownMessage,
)


# ─── Appearance ──────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ─── Tunables ────────────────────────────────────────────────────────────────
LINK_STALE_TIMEOUT_MS = 1500     # No msg for this long → "STALE"
LINK_CHECK_INTERVAL_MS = 200     # Watchdog tick rate
ACK_TIMEOUT_MS = 500             # Pending command must ack within this
ACK_CHECK_INTERVAL_MS = 100      # Pending-command sweeper rate

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
        self.geometry("1200x820")

        # ── State ─────────────────────────────────────────────────
        self.serial: SerialConnection | None = None
        self.sol_states = {1: False, 2: False}
        self.servo_states = {1: 500, 2: 500, 3: 500, 4: 500}

        # Pending commands awaiting an ACK from the ESP32.
        # Each entry: {"id": int, "cmd": Command, "sent_ms": int,
        #              "on_revert": callable | None}
        self._pending: list[dict] = []
        self._next_pending_id = 0

        # Watchdog
        self._last_msg_time_ms: int | None = None
        self._link_state = "disconnected"   # disconnected | live | stale

        # ── CSV logging ───────────────────────────────────────────
        os.makedirs("logs", exist_ok=True)
        log_filename = datetime.datetime.now().strftime(
            "logs/log_%Y%m%d_%H%M%S.csv"
        )
        self._log_fh = open(log_filename, "w", newline="")
        self._log_csv = csv.writer(self._log_fh)
        self._log_csv.writerow(["timestamp", "message"])
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

        # Start the watchdog and ack-timeout sweeper
        self.after(LINK_CHECK_INTERVAL_MS, self._link_watchdog_tick)
        self.after(ACK_CHECK_INTERVAL_MS, self._ack_watchdog_tick)

    # ═══════════════════════════════════════════════════════════════
    # GUI CONSTRUCTION
    # ═══════════════════════════════════════════════════════════════

    def _build_top_bar(self):
        """Connection controls + status indicator + ABORT button."""
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
            bar, text="● Disconnected", text_color="red", width=140
        )
        self.status_label.pack(side="left", padx=15)

        # ── ABORT button — far right, large, unmissable ──
        self.abort_btn = ctk.CTkButton(
            bar, text="⏻  ABORT  ⏻", width=200, height=48,
            font=("", 18, "bold"),
            fg_color="#B00020", hover_color="#D32F2F",
            text_color="white",
            command=self.abort,
        )
        self.abort_btn.pack(side="right", padx=10, pady=4)

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

        The solenoid driver chips (MPQ6610) are not fitted on the v2 board.
        Pressing a solenoid button sends the command to the ESP32, which
        returns an ERROR immediately, and the GUI reverts the button — so
        the buttons are inert until a future expansion daughterboard is added.

        The FAULT:1 line from the ESP32 carries CAN bus health in v3 (not
        MPQ6610 nFAULT as in v1). It is displayed next to SOL 1's button
        since that label area is free. SOL 2's fault label is unused.
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
                # Repurposed in v3: shows CAN link health to the
                # instrumentation board (updated by FAULT:1: messages).
                fault = ctk.CTkLabel(
                    row, text="● CAN Bus ---", text_color="gray", width=160
                )
            else:
                # Channel 2 nFAULT no longer used; show a placeholder.
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
        self._pending.clear()
        # Reset CAN health indicator when disconnected
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

    # ═══════════════════════════════════════════════════════════════
    # SERIAL DATA HANDLING
    # ═══════════════════════════════════════════════════════════════

    def _on_serial_receive(self, msg: Message):
        """Background thread → bounce to main thread."""
        self.after(0, self._handle_message, msg)

    def _handle_message(self, msg: Message):
        """Main thread. Update GUI."""
        # Any message resets the watchdog and the link state
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
                    self.log("CAN bus to instrumentation board is down!")
            # Channel 2 is no longer used in v3 — ignore silently.

        elif isinstance(msg, SensorMessage):
            if msg.name in self.sensor_labels:
                self.sensor_labels[msg.name].configure(
                    text=f"{msg.value:.2f}"
                )
            # Sensor lines are too frequent to log every one.
            # Uncomment if you want full sensor logging:
            # self.log(f"{msg.name}: {msg.value}")

        elif isinstance(msg, AckMessage):
            # Resolve the OLDEST pending command (FIFO).
            if self._pending:
                self._pending.pop(0)

        elif isinstance(msg, ErrorMessage):
            self.log(f"ESP32 ERROR: {msg.message}")
            # An error effectively acks the oldest pending command too
            # (the ESP32 has finished processing it, just unsuccessfully).
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

        Args:
            cmd: The Command to send.
            on_revert: Optional callable(cmd) called if the command
                fails to ack (timeout or error). Used to undo
                optimistic UI updates.

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

        The v2 board has no solenoid driver, so the ESP32 returns an
        ERROR immediately. The ErrorMessage handler pops the pending
        command and calls revert, which undoes the optimistic state change.
        The round-trip is fast enough that the button barely flickers.
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
        """Optimistic update + revert on no-ack."""
        previous = self.servo_states[channel]
        self.servo_states[channel] = pulse_us
        self._update_servo_buttons(channel, pulse_us)

        cmd = ServoCommand(channel, pulse_us)

        def revert(_cmd):
            self.servo_states[channel] = previous
            self._update_servo_buttons(channel, previous)
            self.log(f"Servo {channel} reverted (no ack)")

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
    # ABORT
    # ═══════════════════════════════════════════════════════════════

    def abort(self):
        """Close everything immediately — ALL servos to closed (500 µs).

        Solenoid commands are sent too (in case a future daughterboard is
        connected) but are not reverted on failure: an abort that didn't
        reach the board should NOT re-open the valve in the GUI.
        """
        self.log("⏻ ABORT triggered")

        # Solenoids OFF
        for ch in (1, 2):
            if self.sol_states[ch]:
                self.sol_states[ch] = False
                self._paint_solenoid(ch, False)
                cmd = SolenoidCommand(ch, False)
                self._send_tracked(cmd, on_revert=self._abort_failure_log)

        # Servos all to closed (500 µs)
        for ch in (1, 2, 3, 4):
            if self.servo_states[ch] != 500:
                self.servo_states[ch] = 500
                self._update_servo_buttons(ch, 500)
                cmd = ServoCommand(ch, 500)
                self._send_tracked(cmd, on_revert=self._abort_failure_log)

    def _abort_failure_log(self, cmd: Command):
        self.log(f"⚠ ABORT command not acknowledged: {cmd.to_str().strip()}")

    # ═══════════════════════════════════════════════════════════════
    # LOGGING
    # ═══════════════════════════════════════════════════════════════

    def log(self, message: str):
        """Append a timestamped message to the on-screen log + CSV file."""
        now = datetime.datetime.now()
        screen_ts = now.strftime("%H:%M:%S.%f")[:-3]
        full_ts = now.isoformat(timespec="milliseconds")

        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{screen_ts}] {message}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

        try:
            self._log_csv.writerow([full_ts, message])
            self._log_fh.flush()
        except (ValueError, OSError):
            # File closed during shutdown — ignore
            pass

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
        return int(datetime.datetime.now().timestamp() * 1000)


# ═══════════════════════════════════════════════════════════════════
# MOCK DATA (test the GUI without an ESP32)
# ═══════════════════════════════════════════════════════════════════

def add_mock_data(app: TestStandGUI):
    """Inject fake messages every 200 ms. Useful for screenshots/demos."""
    import random

    def tick():
        # 8 pressure channels (bar — raw counts until calibration.h is filled)
        for i in range(1, 9):
            app._handle_message(SensorMessage(
                f"PRESS{i}", round(2.5 + random.gauss(0, 0.1), 2)
            ))
        # 4 temperature channels (°C)
        for i in range(1, 5):
            app._handle_message(SensorMessage(
                f"TEMP{i}", round(22 + i * 5 + random.gauss(0, 0.5), 1)
            ))
        # 1 force channel (N)
        app._handle_message(SensorMessage(
            "FORCE", round(max(0, 140 + random.gauss(0, 5)), 1)
        ))
        # CAN bus health — channel 1. Simulate 5% chance of a brief fault.
        app._handle_message(FaultMessage(1, random.random() > 0.05))
        # Channel 2 is unused in v3; still emitted here to confirm it is
        # silently ignored by _handle_message.
        app._handle_message(FaultMessage(2, True))

        app.after(200, tick)

    # Force the link to "live" so the GUI watchdog doesn't go stale
    app._set_link_state("live")
    app.after(500, tick)


# ═══════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = TestStandGUI()

    # ──────────────────────────────────────────────────────────────
    # Comment out the next line when using real hardware:
    add_mock_data(app)
    # ──────────────────────────────────────────────────────────────

    app.mainloop()
