"""
main.py — CUSF Static Fire Test Stand Ground Station GUI.

This is the laptop application that controls the test stand.
It connects to the ESP32-S3 on the actuation board over USB serial
and provides:
    - Solenoid valve controls (on/off buttons)
    - Servo position controls (sliders, 500–2500µs)
    - nFAULT status indicators (green/red per solenoid channel)
    - Live sensor readings (pressure, temperature, thrust)
    - Timestamped logging to screen and CSV file

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


# ─── App appearance ──────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class TestStandGUI(ctk.CTk):
    """Main application window for the test stand ground station."""

    def __init__(self):
        super().__init__()

        self.title("CUSF Static Fire — Ground Station")
        self.geometry("1050x750")

        # Serial connection (None until user clicks Connect)
        self.serial = None

        # Track solenoid on/off states
        self.sol_states = {1: False, 2: False}

        # ─── File logging setup ──────────────────────────────
        os.makedirs("logs", exist_ok=True)
        log_filename = datetime.datetime.now().strftime(
            "logs/log_%Y%m%d_%H%M%S.csv"
        )
        self.log_file = open(log_filename, "w")
        self.log_file.write("timestamp,type,data\n")

        # ─── Build the GUI ───────────────────────────────────
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
        """Top bar: port selection, connect/disconnect, status indicator."""
        self.top_frame = ctk.CTkFrame(self)
        self.top_frame.pack(fill="x", padx=10, pady=(10, 5))

        # Port dropdown
        ctk.CTkLabel(self.top_frame, text="Port:").pack(side="left", padx=5)

        self.port_var = ctk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(
            self.top_frame, variable=self.port_var, values=[""]
        )
        self.port_menu.pack(side="left", padx=5)

        # Refresh button — rescans for serial ports
        self.refresh_btn = ctk.CTkButton(
            self.top_frame, text="Refresh", width=80,
            command=self.refresh_ports
        )
        self.refresh_btn.pack(side="left", padx=5)

        # Connect/Disconnect button
        self.connect_btn = ctk.CTkButton(
            self.top_frame, text="Connect", width=100,
            command=self.toggle_connection
        )
        self.connect_btn.pack(side="left", padx=5)

        # Connection status indicator
        self.status_label = ctk.CTkLabel(
            self.top_frame, text="● Disconnected", text_color="red"
        )
        self.status_label.pack(side="left", padx=15)

        # Populate the port dropdown on startup
        self.refresh_ports()

    def _build_main_layout(self):
        """Create the two-column layout: controls on left, data on right."""
        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Left column: valve controls
        self.left_col = ctk.CTkFrame(self.main_frame)
        self.left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # Right column: sensors + log
        self.right_col = ctk.CTkFrame(self.main_frame)
        self.right_col.pack(side="right", fill="both", expand=True, padx=(5, 0))

    def _build_solenoid_controls(self):
        """Solenoid on/off buttons with nFAULT indicators.

        Each solenoid channel gets:
        - A toggle button (grey=off, green=on)
        - An nFAULT status dot (green=ok, red=fault)

        The nFAULT status is updated by incoming FaultMessage data
        from the ESP32, which reads the MPQ6610's nFAULT pin.
        """
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

            # Toggle button
            btn = ctk.CTkButton(
                row, text=f"SOL {ch}: OFF", width=150,
                fg_color="gray30", hover_color="gray40",
                command=lambda c=ch: self.toggle_solenoid(c)
            )
            btn.pack(side="left")
            self.sol_buttons[ch] = btn

            # nFAULT indicator
            fault = ctk.CTkLabel(
                row, text="● nFAULT OK", text_color="green"
            )
            fault.pack(side="left", padx=15)
            self.fault_labels[ch] = fault

    def _build_servo_controls(self):
        """Servo position sliders (500–2500µs).

        Each slider controls one FT5330M servo. The pulse width maps
        to the servo's angular position:
            500µs  = 0° (fully closed)
            1500µs = 90° (centre)
            2500µs = 180° (fully open)

        Moving the slider sends a ServoCommand to the ESP32, which
        updates the LEDC PWM output on the corresponding GPIO pin.
        """
        frame = ctk.CTkFrame(self.left_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Servos", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        # Preset buttons
        preset_row = ctk.CTkFrame(frame, fg_color="transparent")
        preset_row.pack(fill="x", padx=10, pady=(0, 5))

        ctk.CTkButton(
            preset_row, text="All Closed (500)", width=120,
            command=lambda: self.set_all_servos(500)
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            preset_row, text="All Centre (1500)", width=120,
            command=lambda: self.set_all_servos(1500)
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            preset_row, text="All Open (2500)", width=120,
            command=lambda: self.set_all_servos(2500)
        ).pack(side="left", padx=3)

        # Individual sliders
        self.servo_sliders = {}
        self.servo_labels = {}

        for ch in [1, 2, 3, 4]:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=3)

            ctk.CTkLabel(row, text=f"Servo {ch}:", width=70).pack(side="left")

            label = ctk.CTkLabel(row, text="1500 µs", width=70)
            label.pack(side="right")
            self.servo_labels[ch] = label

            slider = ctk.CTkSlider(
                row, from_=500, to=2500, number_of_steps=200,
                command=lambda val, c=ch: self.on_servo_change(c, val)
            )
            slider.set(1500)
            slider.pack(side="left", fill="x", expand=True, padx=10)
            self.servo_sliders[ch] = slider

    def _build_sensor_display(self):
        """Live sensor readings from the instrumentation board.

        These values come from the ESP32, which polls the instrumentation
        board over the inter-board bus. The ESP32 sends SENSOR: lines
        periodically, and the GUI updates the display.
        """
        frame = ctk.CTkFrame(self.right_col)
        frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(frame, text="Sensors", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.sensor_labels = {}

        sensors = [
            ("PRESS1", "Pressure 1", "bar"),
            ("PRESS2", "Pressure 2", "bar"),
            ("TEMP1", "Temperature 1", "°C"),
            ("THRUST", "Thrust", "N"),
        ]

        for key, name, unit in sensors:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=2)

            ctk.CTkLabel(row, text=f"{name}:", width=130).pack(side="left")

            val_label = ctk.CTkLabel(
                row, text="---", font=("", 14, "bold"), width=80
            )
            val_label.pack(side="left")
            self.sensor_labels[key] = val_label

            ctk.CTkLabel(row, text=unit, text_color="gray").pack(side="left")

    def _build_log_panel(self):
        """Scrolling log panel + CSV file logging.

        Every sensor reading, fault, error, and command is logged with
        a timestamp. The log panel shows the last N messages on screen,
        and everything is also written to a CSV file in the logs/ folder.
        """
        frame = ctk.CTkFrame(self.right_col)
        frame.pack(fill="both", expand=True, padx=10, pady=5)

        ctk.CTkLabel(frame, text="Log", font=("", 16, "bold")).pack(
            anchor="w", padx=10, pady=(10, 5)
        )

        self.log_box = ctk.CTkTextbox(frame, height=250, state="disabled")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    # ═══════════════════════════════════════════════════════════
    # CONNECTION HANDLING
    # ═══════════════════════════════════════════════════════════

    def refresh_ports(self):
        """Scan for available serial ports and update the dropdown."""
        ports = find_ports()
        if ports:
            self.port_menu.configure(values=ports)
            self.port_var.set(ports[0])
        else:
            self.port_menu.configure(values=["No ports found"])
            self.port_var.set("No ports found")

    def toggle_connection(self):
        """Connect or disconnect from the ESP32."""
        if self.serial and self.serial.is_connected:
            # Currently connected → disconnect
            self.serial.disconnect()
            self.serial = None
            self.connect_btn.configure(text="Connect")
            self.status_label.configure(
                text="● Disconnected", text_color="red"
            )
            self.log("Disconnected from ESP32")
        else:
            # Currently disconnected → connect
            port = self.port_var.get()
            self.serial = SerialConnection(port)

            # Register our callback — the serial module will call
            # self.on_serial_receive() whenever the ESP32 sends data
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
    # SERIAL DATA HANDLING (the callback chain)
    # ═══════════════════════════════════════════════════════════

    def on_serial_receive(self, msg: Message):
        """Called by the serial reader thread when data arrives.

        THIS RUNS ON THE BACKGROUND THREAD — you must NOT update
        GUI widgets here. Instead, use self.after() to schedule
        the update on the main GUI thread.

        self.after(0, function, args) means:
            "As soon as possible, run this function on the main thread"
        """
        self.after(0, self._handle_message, msg)

    def _handle_message(self, msg: Message):
        """Process a received Message and update the GUI.

        THIS RUNS ON THE MAIN THREAD (scheduled by self.after),
        so it's safe to update widgets here.
        """
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
            self.log(f"SENSOR {msg.name}: {msg.value}")

        elif isinstance(msg, ErrorMessage):
            self.log(f"ERROR from ESP32: {msg.message}")

        elif isinstance(msg, AckMessage):
            pass  # Command acknowledged — nothing to show

    # ═══════════════════════════════════════════════════════════
    # VALVE CONTROLS
    # ═══════════════════════════════════════════════════════════

    def toggle_solenoid(self, channel: int):
        """Toggle a solenoid on/off and send the command to the ESP32."""
        self.sol_states[channel] = not self.sol_states[channel]
        on = self.sol_states[channel]

        # Update button appearance
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

        # Send command to ESP32
        if self.serial and self.serial.is_connected:
            self.serial.send(SolenoidCommand(channel, on))

        self.log(f"SOL {channel} → {'ON' if on else 'OFF'}")

    def on_servo_change(self, channel: int, value: float):
        """Called when a servo slider is moved."""
        pulse = int(value)
        self.servo_labels[channel].configure(text=f"{pulse} µs")

        # Send command to ESP32
        if self.serial and self.serial.is_connected:
            self.serial.send(ServoCommand(channel, pulse))

    def set_all_servos(self, pulse_us: int):
        """Set all 4 servos to the same position (preset button)."""
        for ch in [1, 2, 3, 4]:
            self.servo_sliders[ch].set(pulse_us)
            self.on_servo_change(ch, pulse_us)
        self.log(f"All servos → {pulse_us} µs")

    # ═══════════════════════════════════════════════════════════
    # LOGGING
    # ═══════════════════════════════════════════════════════════

    def log(self, message: str):
        """Log a message to both the GUI panel and the CSV file."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Append to the GUI log panel
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{timestamp}] {message}\n")
        self.log_box.see("end")     # Auto-scroll to the bottom
        self.log_box.configure(state="disabled")

        # Append to the CSV file
        full_timestamp = datetime.datetime.now().isoformat()
        self.log_file.write(f"{full_timestamp},{message}\n")
        self.log_file.flush()       # Write immediately, don't buffer


# ═══════════════════════════════════════════════════════════════
# MOCK DATA (for testing without the ESP32)
# ═══════════════════════════════════════════════════════════════

def add_mock_data(app: TestStandGUI):
    """Inject fake sensor/fault data into the GUI for testing.

    Call this from __main__ to test the GUI without an ESP32 connected.
    It simulates the ESP32 sending periodic sensor readings and
    occasional fault events.
    """
    import random

    def send_fake():
        # Simulate sensor readings with some noise
        app._handle_message(SensorMessage(
            "PRESS1", round(3.0 + random.gauss(0, 0.1), 2)
        ))
        app._handle_message(SensorMessage(
            "PRESS2", round(2.1 + random.gauss(0, 0.05), 2)
        ))
        app._handle_message(SensorMessage(
            "TEMP1", round(25.0 + random.gauss(0, 0.5), 1)
        ))
        app._handle_message(SensorMessage(
            "THRUST", round(max(0, 140 + random.gauss(0, 5)), 1)
        ))

        # nFAULT: usually OK, 5% chance of fault on channel 1
        app._handle_message(FaultMessage(1, random.random() > 0.05))
        app._handle_message(FaultMessage(2, True))

        # Repeat every 200ms
        app.after(200, send_fake)

    # Start after 1 second (give the GUI time to finish building)
    app.after(1000, send_fake)


# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = TestStandGUI()

    # ──────────────────────────────────────────────────────────
    # Uncomment the next line to test with fake data (no ESP32):
    # add_mock_data(app)
    # ──────────────────────────────────────────────────────────

    app.mainloop()