"""
protocol.py — Serial protocol between the laptop GUI and the ESP32-S3.

All messages are ASCII text terminated with newline ('\\n').
This module is the SINGLE SOURCE OF TRUTH for the wire format.
The ESP32 firmware (main.cpp) implements the matching contract.

GUI → ESP32 (Commands):
    SOL1:ON\\n / SOL1:OFF\\n        Energise / de-energise solenoid 1
    SOL2:ON\\n / SOL2:OFF\\n        Energise / de-energise solenoid 2
    SRV1:500\\n / 1500 / 2500       Servo 1 pulse width in µs
    SRV2:... / SRV3:... / SRV4:...
    STATUS\\n                       Request full status

ESP32 → GUI (Messages):
    OK\\n                          Command acknowledged
    ERROR:<text>\\n                Error description
    FAULT:<channel>:<0|1>\\n       nFAULT status (0=fault, 1=ok)
    SENSOR:<name>:<value>\\n       Sensor reading (PRESS1–8, TEMP1–4, FORCE)
    STATUS:SOL1:ON:SOL2:OFF:SRV1:1500:...\\n
"""
from dataclasses import dataclass


# ═══════════════════════════════════════════════════════════════
# COMMANDS  (GUI → ESP32)
# ═══════════════════════════════════════════════════════════════

class Command:
    """Base class for all commands sent to the ESP32."""

    def to_bytes(self) -> bytes:
        return self.to_str().encode("utf-8")

    def to_str(self) -> str:
        raise NotImplementedError("Subclasses must implement to_str()")


@dataclass
class SolenoidCommand(Command):
    """Energise or de-energise a solenoid via MPQ6610.

        ON  → EN=1, IN=1 (high-side MOSFET on, solenoid energised)
        OFF → EN=0       (Hi-Z, solenoid de-energised)
    """
    channel: int   # 1 or 2
    on: bool

    def __post_init__(self):
        if self.channel not in (1, 2):
            raise ValueError(f"Solenoid channel must be 1 or 2, got {self.channel}")

    def to_str(self) -> str:
        state = "ON" if self.on else "OFF"
        return f"SOL{self.channel}:{state}\n"


@dataclass
class ServoCommand(Command):
    """Move a servo to one of three preset positions.

    FT5330M servo at 50 Hz PWM:
        500 µs  = fully closed
        1500 µs = centre
        2500 µs = fully open
    """
    channel: int       # 1–4
    pulse_us: int      # 500, 1500, or 2500

    def __post_init__(self):
        if not 1 <= self.channel <= 4:
            raise ValueError(f"Servo channel must be 1–4, got {self.channel}")
        if self.pulse_us not in (500, 1500, 2500):
            raise ValueError(
                f"Pulse must be 500, 1500, or 2500, got {self.pulse_us}"
            )

    def to_str(self) -> str:
        return f"SRV{self.channel}:{self.pulse_us}\n"


@dataclass
class StatusCommand(Command):
    """Request the ESP32 to report current state of all channels."""

    def to_str(self) -> str:
        return "STATUS\n"


# ═══════════════════════════════════════════════════════════════
# MESSAGES  (ESP32 → GUI)
# ═══════════════════════════════════════════════════════════════

class Message:
    """Base class for all messages received from the ESP32."""
    pass


@dataclass
class FaultMessage(Message):
    """nFAULT pin status from MPQ6610. ok=False means fault detected."""
    channel: int
    ok: bool


@dataclass
class SensorMessage(Message):
    """Sensor reading from the instrumentation board.

    Names: PRESS1–PRESS8 (bar), TEMP1–TEMP4 (°C), FORCE (N).
    """
    name: str
    value: float


@dataclass
class AckMessage(Message):
    """ESP32 acknowledged a command (sent 'OK')."""
    pass


@dataclass
class ErrorMessage(Message):
    """Error reported by ESP32."""
    message: str


@dataclass
class StatusMessage(Message):
    """Full status report. Raw line preserved for the log."""
    raw: str


@dataclass
class UnknownMessage(Message):
    """Safety net for any line we couldn't parse."""
    raw: str


# ═══════════════════════════════════════════════════════════════
# PARSER  (raw serial line → typed Message)
# ═══════════════════════════════════════════════════════════════

def parse_response(line: str) -> Message:
    """Parse one line from the ESP32 into a typed Message.

    Robust to malformed input — never raises. Anything we can't
    parse becomes an UnknownMessage so the read loop keeps running.
    """
    line = line.strip()

    if not line:
        return UnknownMessage(raw="")

    # ── FAULT:<channel>:<0|1> ──
    if line.startswith("FAULT:"):
        try:
            parts = line.split(":")
            if len(parts) != 3:
                return UnknownMessage(raw=line)
            return FaultMessage(channel=int(parts[1]), ok=(parts[2] == "1"))
        except (ValueError, IndexError):
            return UnknownMessage(raw=line)

    # ── SENSOR:<name>:<value> ──
    if line.startswith("SENSOR:"):
        try:
            parts = line.split(":")
            if len(parts) != 3:
                return UnknownMessage(raw=line)
            return SensorMessage(name=parts[1], value=float(parts[2]))
        except (ValueError, IndexError):
            return UnknownMessage(raw=line)

    # ── OK ──
    if line == "OK":
        return AckMessage()

    # ── ERROR:<text> ──
    if line.startswith("ERROR:"):
        return ErrorMessage(message=line[6:])

    # ── STATUS:... ──
    if line.startswith("STATUS:"):
        return StatusMessage(raw=line)

    # Anything else (including ESP32 boot banner)
    return UnknownMessage(raw=line)