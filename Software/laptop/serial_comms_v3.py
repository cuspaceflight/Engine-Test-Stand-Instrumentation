"""
serial_comms_v3.py — USB serial connection to the ESP32-S3.

The ESP32-S3-DevKitC-1 has native USB built into the chip. When plugged
in, it appears as a virtual serial port (e.g. COM3 on Windows,
/dev/ttyACM0 on Linux, /dev/tty.usbmodem* on Mac).

Reads incoming lines on a background thread so the GUI never blocks
waiting for serial data. Parsed Message objects are delivered to the
GUI via a callback.

IMPORTANT (threading):
    The callback runs on the BACKGROUND thread. Do not touch GUI
    widgets directly inside it — use self.after(0, ...) in the GUI
    to bounce the work onto the main thread.
"""
import time
import threading
import serial
import serial.tools.list_ports

from protocol_v4 import Command, Message, parse_response


# Espressif's USB Vendor ID — used to filter the port list to actual
# ESP32-S3 devices (instead of every Bluetooth virtual port on the laptop).
ESPRESSIF_VID = 0x303A


def find_ports(esp32_only: bool = True) -> list[str]:
    """List available serial ports.

    Args:
        esp32_only: If True, only return ports whose USB VID matches
            Espressif (0x303A). Falls back to all ports if no Espressif
            devices are found.

    Returns:
        List of port name strings, e.g. ["COM3"] or ["/dev/ttyACM0"].
    """
    all_ports = list(serial.tools.list_ports.comports())

    if esp32_only:
        esp_ports = [p.device for p in all_ports if p.vid == ESPRESSIF_VID]
        if esp_ports:
            return esp_ports

    return [p.device for p in all_ports]


class SerialConnection:
    """Manages a serial connection to the ESP32-S3.

    Usage:
        conn = SerialConnection("COM3")
        conn.set_callback(my_handler)
        conn.connect()
        conn.send(ServoCommand(1, 2500))
        ...
        conn.disconnect()
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._running = False
        self._on_receive = None

    def connect(self) -> bool:
        """Open the serial port and start the reader thread.

        Returns True on success, False on failure.
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,   # readline() returns after 100 ms if nothing
            )
            # ESP32-S3 resets when DTR toggles on connect; give it time
            # to boot before sending anything.
            time.sleep(0.5)

            self._running = True
            self._reader_thread = threading.Thread(
                target=self._read_loop,
                daemon=True,
                name="SerialReader",
            )
            self._reader_thread.start()
            return True

        except serial.SerialException as err:
            print(f"Failed to connect to {self.port}: {err}")
            self.ser = None
            return False

    def disconnect(self) -> None:
        """Stop the reader thread and close the port."""
        self._running = False

        if self._reader_thread is not None:
            self._reader_thread.join(timeout=0.5)
            self._reader_thread = None

        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def send(self, command: Command) -> bool:
        """Send a Command to the ESP32. Returns True on success."""
        if self.ser is None or not self.ser.is_open:
            return False
        try:
            self.ser.write(command.to_bytes())
            return True
        except serial.SerialException as err:
            print(f"Serial write failed: {err}")
            return False

    def set_callback(self, callback) -> None:
        """Register a function to be called for each incoming Message.

        The callback runs on the background reader thread. The GUI
        must not update widgets inside it — use self.after() to
        bounce to the main thread.
        """
        self._on_receive = callback

    @property
    def is_connected(self) -> bool:
        """True iff the port is open AND the reader thread is alive.

        Checking the thread is important: if _read_loop crashed, the
        thread is dead but self.ser may still claim to be open. Without
        this check the GUI thinks it's connected and silently sends
        commands into the void.
        """
        return (
            self.ser is not None
            and self.ser.is_open
            and self._reader_thread is not None
            and self._reader_thread.is_alive()
        )

    # ───────────────────────────────────────────────────────────
    # Background thread
    # ───────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Continuously read lines from the serial port (background thread).

        Always sets self._running = False before exiting, so the
        is_connected property correctly reports the dead state.
        """
        try:
            while self._running:
                try:
                    if self.ser is None or not self.ser.is_open:
                        break

                    if self.ser.in_waiting:
                        raw = self.ser.readline()
                        text = raw.decode("utf-8", errors="replace").strip()

                        if text and self._on_receive is not None:
                            msg = parse_response(text)
                            try:
                                self._on_receive(msg)
                            except Exception as cb_err:
                                # A buggy callback must never kill the reader
                                print(f"Callback error: {cb_err}")
                    else:
                        time.sleep(0.01)

                except serial.SerialException as err:
                    print(f"Serial connection lost: {err}")
                    break
                except OSError as err:
                    # USB cable yanked → OSError on Linux/Mac
                    print(f"Serial OS error: {err}")
                    break
        finally:
            # Ensure is_connected returns False once we exit, even if
            # someone forgot to call disconnect()
            self._running = False
