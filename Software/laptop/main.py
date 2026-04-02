"""
CUSF Static Fire Test Stand — Ground Station GUI
"""
import customtkinter as ctk
from serial_comms import find_ports, SerialConnection
from protocol import (
    SolenoidCommand, ServoCommand, StatusCommand, parse_response
)

# -- App setup --
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class TestStandGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("CUSF Test Stand")
        self.geometry("1000x700")

        self.serial = None  # SerialConnection instance

        # ---- Top bar: connection controls ----
        self.top_frame = ctk.CTkFrame(self)
        self.top_frame.pack(fill="x", padx=10, pady=(10, 5))

        ctk.CTkLabel(self.top_frame, text="Port:").pack(side="left", padx=5)

        self.port_var = ctk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(
            self.top_frame, variable=self.port_var, values=[""]
        )
        self.port_menu.pack(side="left", padx=5)

        self.refresh_btn = ctk.CTkButton(
            self.top_frame, text="Refresh", width=80,
            command=self.refresh_ports
        )
        self.refresh_btn.pack(side="left", padx=5)

        self.connect_btn = ctk.CTkButton(
            self.top_frame, text="Connect", width=100,
            command=self.toggle_connection
        )
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ctk.CTkLabel(
            self.top_frame, text="● Disconnected",
            text_color="red"
        )
        self.status_label.pack(side="left", padx=15)

        # Populate ports on startup
        self.refresh_ports()

    def refresh_ports(self):
        """Scan for available serial ports."""
        ports = find_ports()
        if ports:
            self.port_menu.configure(values=ports)
            self.port_var.set(ports[0])
        else:
            self.port_menu.configure(values=["No ports found"])
            self.port_var.set("No ports found")

    def toggle_connection(self):
        """Connect or disconnect."""
        if self.serial and self.serial.is_connected:
            self.serial.disconnect()
            self.serial = None
            self.connect_btn.configure(text="Connect")
            self.status_label.configure(
                text="● Disconnected", text_color="red"
            )
        else:
            port = self.port_var.get()
            self.serial = SerialConnection(port)
            self.serial.set_callback(self.on_serial_receive)
            if self.serial.connect():
                self.connect_btn.configure(text="Disconnect")
                self.status_label.configure(
                    text="● Connected", text_color="green"
                )
            else:
                self.status_label.configure(
                    text="● Failed", text_color="orange"
                )

    def on_serial_receive(self, line: str):
        """Called from the serial reader thread when a line arrives.
        We'll fill this in during later steps."""
        parsed = parse_response(line)
        print(f"Received: {parsed}")  # For now, just print


if __name__ == "__main__":
    app = TestStandGUI()
    app.mainloop()