"""
main.py — CUSF Static Fire Test Stand Ground Station GUI.

Controls:
    - 2 solenoid valves (on/off toggle buttons)
    - 4 servos (3 preset buttons each: closed / centre / open)
    - 3 global servo presets (all closed / all centre / all open)

Displays:
    - nFAULT status per solenoid (green/red)
    - 8 pressure readings (bar)
    - 4 temperature readings (°C)
    - 1 force/thrust reading (N)
    - Timestamped log (screen + CSV file)

Run with:  python main.py
"""

import os
import datetime
import customtkinter as ctk
from serial_comms import find_ports, SerialConnection
from protocol import (
    SolenoidCommand, ServoCommand, StatusCommand,
    Message, FaultMessage, SensorMessage, AcknowledgementMessage, ErrorMessage
)

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class TestStandGUI(ctk.CTk):

    def __init__(self):
        super().__init__()

        self.title("CUSF Static Fire — Ground Station")
        self.geometry("1100x800")

        self.serial = None
        self.sol_states = {1: False, 2: False}
        self.servo_states = {1: 500, 2: 500, 3: 500, 4: 500}

        # File logging
        os.makedirs("logs", exist_ok=True)
        log_filename = datetime.datetime.now().strftime(
            "logs/log_%Y%m%d_%H%M%S.csv"
        )
        self.log_file = open(log_filename, "w", encoding="utf-8")
        self.log_file.write("timestamp,type,data\n")

        # Build GUI
        self._build_connection_bar()
        self._build_main_layout()
        self._build_solenoid_controls()
        self._build_servo_controls()
        self._build_sensor_display()
        self._build_log_panel()

    # ═══════════════════════════════════════════════════════════
    # GUI CONSTRUCTION
    # ═══════════════════════════════════════════════════════════

    def _build_connection_bar(self):
        """Top bar: port selection, connect/disconnect, status."""
        frame = ctk.CTkFrame(self)
        frame.pack(fill="x", padx=10, pady=(10, 5))

        ctk.CTkLabel(frame, text="Port:").pack(side="left", padx=5)

        self.port_var = ctk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(
            frame, variable=self.port_var, values=[""]
        )
        self.port_menu.pack(side="left", padx=5)

        ctk.CTkButton(
            frame, text="Refresh", width=80, command=self.refresh_ports
        ).pack(side="left", padx=5)

        self.connect_btn = ctk.CTkButton(
            frame, text="Connect", width=100, command=self.toggle_connection
        )
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ctk.CTkLabel(
            frame, text="● Disconnected", text_color="red"
        )
        self.status_label.pack(side="left", padx=15)

        self.refresh_ports()

    def _build_main_layout(self):
        """Two-column layout: controls on left, sensors + log on right."""
        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.left_col = ctk.CTkFrame(self.main_frame)
        self.left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        self.right_col = ctk.CTkFrame(self.main_frame)
        self.right_col.pack(side="right", fill="both", expand=True, padx=(5, 0))

    def _build_solenoid_controls(self):
        """2 solenoid toggle buttons with nFAULT indicators."""
        frame = ctk.CTkFrame(self.left_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Solenoids", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.sol_buttons = {}
        self.fault_labels = {}

        for ch in [1, 2]:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=3)

            btn = ctk.CTkButton(
                row, text=f"SOL {ch}: OFF", width=150,
                fg_color="gray30", hover_color="gray40",
                command=lambda c=ch: self.toggle_solenoid(c)
            )
            btn.pack(side="left")
            self.sol_buttons[ch] = btn

            fault = ctk.CTkLabel(
                row, text="● nFAULT OK", text_color="green"
            )
            fault.pack(side="left", padx=15)
            self.fault_labels[ch] = fault

    def _build_servo_controls(self):
        """4 servos × 3 preset buttons + 3 global presets.

        Each servo gets: [Closed (500)] [Centre (1500)] [Open (2500)]
        The active position is highlighted in colour.
        """
        frame = ctk.CTkFrame(self.left_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Servos", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        # Global presets
        preset_row = ctk.CTkFrame(frame, fg_color="transparent")
        preset_row.pack(fill="x", padx=10, pady=(0, 8))

        ctk.CTkButton(
            preset_row, text="All Closed", width=110,
            fg_color="#8B0000", hover_color="#A52A2A",
            command=lambda: self.set_all_servos(500)
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            preset_row, text="All Centre", width=110,
            fg_color="#B8860B", hover_color="#DAA520",
            command=lambda: self.set_all_servos(1500)
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            preset_row, text="All Open", width=110,
            fg_color="#006400", hover_color="#228B22",
            command=lambda: self.set_all_servos(2500)
        ).pack(side="left", padx=3)

        # Per-servo buttons
        self.servo_buttons = {}  # {(channel, pulse): button_widget}

        presets = [
            (500, "Closed", "#8B0000", "#A52A2A"),
            (1500, "Centre", "#B8860B", "#DAA520"),
            (2500, "Open", "#006400", "#228B22"),
        ]

        for ch in [1, 2, 3, 4]:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=2)

            ctk.CTkLabel(row, text=f"Servo {ch}:", width=70).pack(side="left")

            for pulse, label, color, hover in presets:
                btn = ctk.CTkButton(
                    row, text=label, width=90,
                    fg_color="gray30", hover_color="gray40",
                    command=lambda c=ch, p=pulse: self.set_servo(c, p)
                )
                btn.pack(side="left", padx=3)
                self.servo_buttons[(ch, pulse)] = btn

            # Status label showing current position
            status = ctk.CTkLabel(row, text="500 µs", width=60, text_color="gray")
            status.pack(side="left", padx=8)
            # Store with a special key
            self.servo_buttons[(ch, "label")] = status

        # Highlight the initial state (all closed)
        for ch in [1, 2, 3, 4]:
            self._update_servo_buttons(ch, 500)

    def _build_sensor_display(self):
        """8 pressure + 4 temperature + 1 force sensor displays."""
        frame = ctk.CTkFrame(self.right_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Sensors", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.sensor_labels = {}

        # Use a scrollable frame since we have 13 sensors
        sensor_scroll = ctk.CTkScrollableFrame(frame, height=300)
        sensor_scroll.pack(fill="x", padx=10, pady=(0, 10))

        # Pressure sensors (8)
        ctk.CTkLabel(
            sensor_scroll, text="Pressure", font=("", 13, "bold"),
            text_color="gray"
        ).pack(anchor="w", pady=(5, 2))

        for i in range(1, 9):
            key = f"PRESS{i}"
            row = ctk.CTkFrame(sensor_scroll, fg_color="transparent")
            row.pack(fill="x", pady=1)
            ctk.CTkLabel(row, text=f"  P{i}:", width=50).pack(side="left")
            val = ctk.CTkLabel(row, text="---", font=("", 13, "bold"), width=70)
            val.pack(side="left")
            ctk.CTkLabel(row, text="bar", text_color="gray").pack(side="left")
            self.sensor_labels[key] = val

        # Temperature sensors (4)
        ctk.CTkLabel(
            sensor_scroll, text="Temperature", font=("", 13, "bold"),
            text_color="gray"
        ).pack(anchor="w", pady=(10, 2))

        for i in range(1, 5):
            key = f"TEMP{i}"
            row = ctk.CTkFrame(sensor_scroll, fg_color="transparent")
            row.pack(fill="x", pady=1)
            ctk.CTkLabel(row, text=f"  T{i}:", width=50).pack(side="left")
            val = ctk.CTkLabel(row, text="---", font=("", 13, "bold"), width=70)
            val.pack(side="left")
            ctk.CTkLabel(row, text="°C", text_color="gray").pack(side="left")
            self.sensor_labels[key] = val

        # Force sensor (1)
        ctk.CTkLabel(
            sensor_scroll, text="Force", font=("", 13, "bold"),
            text_color="gray"
        ).pack(anchor="w", pady=(10, 2))

        row = ctk.CTkFrame(sensor_scroll, fg_color="transparent")
        row.pack(fill="x", pady=1)
        ctk.CTkLabel(row, text="  F:", width=50).pack(side="left")
        val = ctk.CTkLabel(row, text="---", font=("", 13, "bold"), width=70)
        val.pack(side="left")
        ctk.CTkLabel(row, text="N", text_color="gray").pack(side="left")
        self.sensor_labels["FORCE"] = val

    def _build_log_panel(self):
        """Scrolling log with timestamps."""
        frame = ctk.CTkFrame(self.right_col)
        frame.pack(fill="both", expand=True, padx=10, pady=5)

        ctk.CTkLabel(frame, text="Log", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.log_box = ctk.CTkTextbox(frame, height=200, state="disabled")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    # ═══════════════════════════════════════════════════════════
    # CONNECTION
    # ═══════════════════════════════════════════════════════════

    def refresh_ports(self):
        ports = find_ports()
        if ports:
            self.port_menu.configure(values=ports)
            self.port_var.set(ports[0])
        else:
            self.port_menu.configure(values=["No ports found"])
            self.port_var.set("No ports found")

    def toggle_connection(self):
        if self.serial and self.serial.is_connected:
            self.serial.disconnect()
            self.serial = None
            self.connect_btn.configure(text="Connect")
            self.status_label.configure(
                text="● Disconnected", text_color="red"
            )
            self.log("Disconnected from ESP32")
        else:
            port = self.port_var.get()
            self.serial = SerialConnection(port)
            self.serial.set_callback(self.on_serial_receive)

            if self.serial.connect():
                self.connect_btn.configure(text="Disconnect")
                self.status_label.configure(
                    text="● Connected", text_color="green"
                )
                self.log(f"Connected to {port}")
            else:
                self.serial = None
                self.status_label.configure(
                    text="● Failed", text_color="orange"
                )
                self.log(f"Failed to connect to {port}")

    # ═══════════════════════════════════════════════════════════
    # SERIAL DATA HANDLING
    # ═══════════════════════════════════════════════════════════

    def on_serial_receive(self, msg: Message):
        """Called from background thread. Schedules GUI update on main thread."""
        self.after(0, self._handle_message, msg)

    def _handle_message(self, msg: Message):
        """Process a received message. Runs on main thread."""
        if isinstance(msg, FaultMessage):
            ch = msg.channel
            if ch in self.fault_labels:
                if msg.ok:
                    self.fault_labels[ch].configure(
                        text="● nFAULT OK", text_color="green"
                    )
                else:
                    self.fault_labels[ch].configure(
                        text="● FAULT!", text_color="red"
                    )
                    self.log(f"FAULT on solenoid {ch}!")

        elif isinstance(msg, SensorMessage):
            if msg.name in self.sensor_labels:
                self.sensor_labels[msg.name].configure(
                    text=f"{msg.value:.1f}"
                )
            self.log(f"{msg.name}: {msg.value}")

        elif isinstance(msg, ErrorMessage):
            self.log(f"ERROR: {msg.message}")

        elif isinstance(msg, AcknowledgementMessage):
            pass

    # ═══════════════════════════════════════════════════════════
    # VALVE CONTROLS
    # ═══════════════════════════════════════════════════════════

    def toggle_solenoid(self, channel: int):
        """Toggle solenoid on/off."""
        self.sol_states[channel] = not self.sol_states[channel]
        on = self.sol_states[channel]

        if on:
            self.sol_buttons[channel].configure(
                text=f"SOL {channel}: ON",
                fg_color="green", hover_color="darkgreen"
            )
        else:
            self.sol_buttons[channel].configure(
                text=f"SOL {channel}: OFF",
                fg_color="gray30", hover_color="gray40"
            )

        if self.serial and self.serial.is_connected:
            self.serial.send(SolenoidCommand(channel, on))

        self.log(f"SOL {channel} → {'ON' if on else 'OFF'}")

    def set_servo(self, channel: int, pulse_us: int):
        """Set one servo to a preset position."""
        self.servo_states[channel] = pulse_us
        self._update_servo_buttons(channel, pulse_us)

        if self.serial and self.serial.is_connected:
            self.serial.send(ServoCommand(channel, pulse_us))

        label = {500: "Closed", 1500: "Centre", 2500: "Open"}[pulse_us]
        self.log(f"Servo {channel} → {label} ({pulse_us} µs)")

    def set_all_servos(self, pulse_us: int):
        """Set all 4 servos to the same position."""
        for ch in [1, 2, 3, 4]:
            self.set_servo(ch, pulse_us)

    def _update_servo_buttons(self, channel: int, active_pulse: int):
        """Highlight the active button for a servo channel, grey out the rest."""
        colours = {
            500: ("#8B0000", "#A52A2A"),      # Dark red
            1500: ("#B8860B", "#DAA520"),      # Dark gold
            2500: ("#006400", "#228B22"),      # Dark green
        }

        for pulse in [500, 1500, 2500]:
            btn = self.servo_buttons[(channel, pulse)]
            if pulse == active_pulse:
                fg, hover = colours[pulse]
                btn.configure(fg_color=fg, hover_color=hover)
            else:
                btn.configure(fg_color="gray30", hover_color="gray40")

        # Update the status label
        self.servo_buttons[(channel, "label")].configure(
            text=f"{active_pulse} µs"
        )

    # ═══════════════════════════════════════════════════════════
    # LOGGING
    # ═══════════════════════════════════════════════════════════

    def log(self, message: str):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{timestamp}] {message}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

        full_timestamp = datetime.datetime.now().isoformat()
        self.log_file.write(f"{full_timestamp},{message}\n")
        self.log_file.flush()


# ═══════════════════════════════════════════════════════════════
# MOCK DATA (testing without ESP32)
# ═══════════════════════════════════════════════════════════════

def add_mock_data(app: TestStandGUI):
    """Inject fake data for GUI testing. No ESP32 needed."""
    import random

    def send_fake():
        # 8 pressure sensors
        for i in range(1, 9):
            app._handle_message(SensorMessage(
                f"PRESS{i}", round(2.5 + random.gauss(0, 0.15), 2)
            ))

        # 4 temperature sensors
        for i in range(1, 5):
            app._handle_message(SensorMessage(
                f"TEMP{i}", round(22.0 + i * 5 + random.gauss(0, 0.5), 1)
            ))

        # 1 force sensor
        app._handle_message(SensorMessage(
            "FORCE", round(max(0, 140 + random.gauss(0, 5)), 1)
        ))

        # nFAULT: usually OK, occasional fault on channel 1
        app._handle_message(FaultMessage(1, random.random() > 0.05))
        app._handle_message(FaultMessage(2, True))

        app.after(200, send_fake)

    app.after(1000, send_fake)


# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = TestStandGUI()

    # Uncomment to test with fake data (no ESP32 needed):
    # add_mock_data(app)

    app.mainloop()