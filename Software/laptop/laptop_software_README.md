# Ground Station GUI — Technical Documentation

## What this software does

This is the laptop application for the CUSF static fire test stand. It connects to the ESP32-S3 microcontroller on the actuation board over a USB cable and lets the operator control valves and monitor sensors during an engine test firing.

The software is three Python files totalling about 500 lines. Each file has one job:

| File | Job | Analogy |
|------|-----|---------|
| `protocol.py` | Defines the language the GUI and ESP32 speak | A dictionary / phrasebook |
| `serial_comms.py` | Manages the USB cable connection | A telephone line |
| `main.py` | The window you see and click on | The control room |

---

## How it connects to the hardware

```
 ┌─────────────┐    USB-C cable    ┌───────────────────────────┐
 │   LAPTOP    │◄─────────────────►│   ACTUATION BOARD         │
 │             │   (virtual serial │                           │
 │  main.py    │    port, 115200   │   ESP32-S3-DevKitC-1     │
 │  (this GUI) │    baud)          │     │                     │
 │             │                   │     ├── GPIO → MPQ6610 #1 ──► Solenoid 1
 └─────────────┘                   │     ├── GPIO → MPQ6610 #2 ──► Solenoid 2
                                   │     ├── PWM  → Servo 1 (FT5330M)
                                   │     ├── PWM  → Servo 2 (FT5330M)
                                   │     ├── PWM  → Servo 3 (FT5330M)
                                   │     ├── PWM  → Servo 4 (FT5330M)
                                   │     └── Bus  → Instrumentation Board
                                   └───────────────────────────┘
```

The ESP32-S3 has **native USB** built into the chip (on GPIO19 and GPIO20). When you plug a USB-C cable into the DevKitC-1, the laptop sees it as a virtual serial port — `COM3` on Windows or `/dev/ttyACM0` on Linux. No special driver is needed.

Serial communication is just sending text back and forth, one line at a time, at 115200 bits per second. It is the same protocol the Arduino Serial Monitor uses.

---

## File 1: protocol.py

### Purpose

This file is a contract between the GUI and the ESP32 firmware. It defines exactly what strings are sent in each direction and provides Python classes to construct and parse them.

Your teammate writing the ESP32 firmware needs to agree on this file. If both sides follow the same protocol, they will work together even though they are written independently.

### How the protocol works

Every message is a line of ASCII text ending with a newline character (`\n`). We use text rather than binary because:

1. You can type commands into any serial terminal for debugging
2. You can read the log files with a text editor
3. Parsing is simple string splitting

### Commands (GUI → ESP32)

When you click a button in the GUI, it creates a Command object and sends it over serial. Here is the complete chain for turning on solenoid 1:

```
User clicks "SOL 1" button
        │
        ▼
GUI creates: SolenoidCommand(channel=1, on=True)
        │
        ▼
SolenoidCommand.to_str() returns: "SOL1:ON\n"
        │
        ▼
SolenoidCommand.to_bytes() encodes to: b"SOL1:ON\n"
        │
        ▼
pyserial sends these bytes over USB
        │
        ▼
ESP32 receives "SOL1:ON\n", sets MPQ6610 #1 EN=HIGH, IN=HIGH
        │
        ▼
MPQ6610 high-side MOSFET turns on → 24V flows through solenoid
        │
        ▼
Solenoid energises → valve opens
```

### Messages (ESP32 → GUI)

The ESP32 periodically sends sensor readings and fault status without being asked. The `parse_response()` function converts each line into a typed Python object:

```
ESP32 sends: "FAULT:1:0\n"
        │
        ▼
parse_response("FAULT:1:0") splits on ":"
        │
        ▼
Returns: FaultMessage(channel=1, ok=False)
        │
        ▼
GUI checks isinstance(msg, FaultMessage)
        │
        ▼
Updates the nFAULT indicator for channel 1 to red
```

### The class hierarchy

```
Command (base class — "something we send")
├── SolenoidCommand    channel, on        → "SOL1:ON\n"
├── ServoCommand       channel, pulse_us  → "SRV2:1500\n"
└── StatusCommand      (no data)          → "STATUS\n"

Message (base class — "something we receive")
├── FaultMessage       channel, ok        ← "FAULT:1:0"
├── SensorMessage      name, value        ← "SENSOR:PRESS1:3.45"
├── AckMessage         (no data)          ← "OK"
├── ErrorMessage       message            ← "ERROR:Invalid command"
├── StatusMessage      raw                ← "STATUS:SOL1:ON:..."
└── UnknownMessage     raw                ← anything else
```

### What @dataclass does

The `@dataclass` decorator is a shortcut. It automatically generates the `__init__` method and a readable `__repr__`. These two are equivalent:

```python
# Without @dataclass (verbose)
class FaultMessage(Message):
    def __init__(self, channel: int, ok: bool):
        self.channel = channel
        self.ok = ok
    def __repr__(self):
        return f"FaultMessage(channel={self.channel}, ok={self.ok})"

# With @dataclass (compact — Python generates the above for you)
@dataclass
class FaultMessage(Message):
    channel: int
    ok: bool
```

### What `__post_init__` does (in ServoCommand)

`@dataclass` generates `__init__` for you, so you cannot write your own. But sometimes you need to run extra logic after the object is created (like clamping the servo pulse range to 500–2500). `__post_init__` is called automatically right after `__init__` finishes:

```python
cmd = ServoCommand(channel=1, pulse_us=9999)
# 1. @dataclass __init__ runs:  self.channel=1, self.pulse_us=9999
# 2. __post_init__ runs:        self.pulse_us = max(500, min(2500, 9999)) → 2500
# Result: cmd.pulse_us == 2500
```

---

## File 2: serial_comms.py

### Purpose

Manages the USB serial connection to the ESP32. Handles port discovery, connecting, disconnecting, sending commands, and reading incoming data on a background thread.

### Why a background thread?

Serial reading is **blocking** — `ser.readline()` waits until a complete line arrives. If you did this on the main thread (the one running the GUI), the entire window would freeze:

```
Without background thread:
    User clicks button → GUI tries to read serial → waits...
    → waits... → waits... → meanwhile GUI is frozen, user
    can't click anything, window says "Not Responding"
```

With a background thread, reading happens independently:

```
Main thread:              Background thread:
    GUI draws widgets         Checks for serial data
    User clicks buttons       Reads a line
    Sliders move              Parses it
    Labels update             Calls the callback
         ▲                          │
         └──── self.after() ────────┘
              (safely passes data between threads)
```

### The callback pattern explained

The serial module doesn't know anything about the GUI. It doesn't import `main.py`. Instead, the GUI gives serial_comms a function to call when data arrives. This is called the **callback pattern**.

Here is the simplest possible example of the pattern:

```python
# === The serial module's perspective ===
class SerialConnection:
    def __init__(self):
        self._on_receive = None       # No callback yet

    def set_callback(self, func):
        self._on_receive = func       # Store the function

    def _read_loop(self):
        data = "FAULT:1:0"            # Pretend we read this
        msg = parse_response(data)    # Turn it into a FaultMessage
        self._on_receive(msg)         # Call whatever function was stored


# === The GUI's perspective ===
class MyGUI:
    def __init__(self):
        self.serial = SerialConnection()

        # "When you get data, call my handle_it method"
        self.serial.set_callback(self.handle_it)

    def handle_it(self, msg):
        print(f"GUI got: {msg}")
```

When `_read_loop` runs `self._on_receive(msg)`, it is calling `self.handle_it(msg)` on the GUI — because that is the function that was stored by `set_callback`.

Think of it like a restaurant:
1. You give the waiter your phone number (set_callback)
2. The waiter writes it down (_on_receive = func)
3. When your table is ready, the waiter calls you (_on_receive(msg))
4. You pick up the phone and act on it (handle_it runs)

### Thread safety and self.after()

There is one complication: the callback runs on the **background thread**, but GUI widgets (buttons, labels, etc.) can only be updated from the **main thread**. If you try to change a label's text from the background thread, the application crashes.

The solution is `self.after(0, function, args)`, which means: "schedule this function to run on the main thread as soon as possible."

```python
def on_serial_receive(self, msg):
    # This runs on the BACKGROUND thread (called by _read_loop)
    # Do NOT touch any GUI widgets here!

    # Instead, schedule _handle_message to run on the main thread:
    self.after(0, self._handle_message, msg)

def _handle_message(self, msg):
    # This runs on the MAIN thread (scheduled by self.after)
    # Safe to update GUI widgets here:
    self.fault_labels[1].configure(text="● FAULT!", text_color="red")
```

### The read loop in detail

```python
def _read_loop(self):
    while self._running:                    # Loop until disconnect() sets this False
        if self.ser.in_waiting:             # Are there bytes in the USB buffer?
            raw = self.ser.readline()       #   YES: read one line (blocks until \n)
            text = raw.decode("utf-8")      #   Convert bytes to string
            msg = parse_response(text)      #   "FAULT:1:0" → FaultMessage(1, False)
            self._on_receive(msg)           #   Call the GUI's callback
        else:
            time.sleep(0.01)                #   NO: wait 10ms, check again
```

The `in_waiting` check is important. Without it, `readline()` would block for up to 100ms (the timeout we set) every iteration, even when there is no data. By checking first, we only call readline when we know data is there — this keeps the loop responsive.

---

## File 3: main.py

### Purpose

The GUI window — everything the operator sees and interacts with. Built with `customtkinter`, which is a modern-looking wrapper around Python's built-in `tkinter` GUI library.

### Window layout

```
┌──────────────────────────────────────────────────┐
│  [Port: COM3 ▼]  [Refresh]  [Connect]  ● Green  │  ← Connection bar
├────────────────────────┬─────────────────────────┤
│  Solenoids             │  Sensors                │
│  ┌─────────────┬─────┐ │  Pressure 1:  3.4 bar   │
│  │ SOL 1: OFF  │ ● OK│ │  Pressure 2:  2.1 bar   │
│  │ SOL 2: OFF  │ ● OK│ │  Temperature: 25.3 °C   │
│  └─────────────┴─────┘ │  Thrust:      142.7 N   │
│                         │                         │
│  Servos                 │  Log                    │
│  [All Closed] [Centre]  │  [09:41:23] Connected   │
│  Servo 1: ───●─── 1500 │  [09:41:24] SOL 1 → ON  │
│  Servo 2: ───●─── 1500 │  [09:41:25] SENSOR ...  │
│  Servo 3: ───●─── 1500 │  [09:41:25] FAULT on 1! │
│  Servo 4: ───●─── 1500 │  [09:41:26] SENSOR ...  │
└────────────────────────┴─────────────────────────┘
```

### How each GUI element maps to hardware

**Solenoid buttons → MPQ6610 → Solenoid valves**

When you click "SOL 1", the GUI sends `SOL1:ON\n` over USB. The ESP32 firmware receives this, sets GPIO4=HIGH (EN) and GPIO5=HIGH (IN). Inside the MPQ6610, this turns on the high-side MOSFET, connecting the solenoid coil to the 24V supply. Current flows through the coil, creating a magnetic field that pulls the plunger and opens the valve.

When you click again (OFF), the GUI sends `SOL1:OFF\n`. The ESP32 sets GPIO4=LOW (EN). The MPQ6610 turns off both MOSFETs (Hi-Z state). Current stops flowing, the solenoid's spring pushes the plunger back, and the valve closes.

**nFAULT indicators → MPQ6610 nFAULT pin → ESP32 GPIO**

The MPQ6610 has a built-in fault detection pin (nFAULT). It is normally HIGH (pulled up by a 10kΩ resistor to 3.3V). If the MPQ6610 detects a problem — solenoid drawing too much current (>3A), chip overheating (>165°C), solenoid disconnected, or supply voltage too low (<4.1V) — it pulls nFAULT LOW.

The ESP32 reads this pin (GPIO8 for channel 1, GPIO9 for channel 2) and sends `FAULT:1:0\n` or `FAULT:1:1\n` to the GUI. The GUI updates the indicator dot: green for OK, red for FAULT.

**Servo sliders → ESP32 LEDC PWM → FT5330M servos → Ball valves**

When you drag a slider, the GUI sends (for example) `SRV1:1800\n`. The ESP32 firmware updates its LEDC peripheral to output a 50Hz PWM signal with an 1800µs pulse width on the corresponding GPIO pin (GPIO10–13). The FT5330M servo's internal controller reads this pulse and moves the output shaft to the corresponding angle. The servo horn is mechanically linked to a ball valve, rotating it to control flow.

The FT5330M accepts 500–2500µs pulses (0–180°). The slider is clamped to this range.

**Sensor readings → Instrumentation board → ESP32 → GUI**

The ESP32 periodically polls the instrumentation board (your teammate's board) over the inter-board bus. It reads pressure transducers, thermocouples, and the load cell, then sends `SENSOR:PRESS1:3.45\n` to the GUI. The GUI updates the number on screen. These readings are also logged to a CSV file.

The solenoid control logic on the ESP32 uses these sensor readings to make decisions — for example, opening the fuel valve when tank pressure reaches a target value.

### The main thread vs background thread

This is the most important architectural concept in the code. Python GUI applications (tkinter, Qt, etc.) have a strict rule: **only the main thread can touch GUI widgets**. If a background thread tries to change a label's text, the application crashes or behaves unpredictably.

But we need a background thread to read serial data continuously without freezing the GUI. The solution:

```
MAIN THREAD                         BACKGROUND THREAD
(runs the GUI)                      (reads serial data)
     │                                    │
     │  app.mainloop()                    │  _read_loop()
     │  ├── draws widgets                 │  ├── ser.readline()
     │  ├── handles button clicks         │  ├── parse_response()
     │  ├── runs self.after() tasks ◄─────│──├── self._on_receive(msg)
     │  │         │                       │  │     calls on_serial_receive()
     │  │         ▼                       │  │     which calls self.after()
     │  │   _handle_message(msg)          │  │
     │  │   ├── update fault label        │  └── sleep, loop again
     │  │   ├── update sensor value       │
     │  │   └── write to log              │
     │  └── loop                          │
```

`self.after(0, func, args)` is the bridge: it says "next time the main thread's event loop checks, please run this function." It is safe because the function actually executes on the main thread — the background thread only schedules it.

### Logging

Every event is logged to two places:

1. **GUI log panel** — visible on screen, scrolls automatically, timestamped
2. **CSV file** — in the `logs/` folder, named by date/time, can be opened in Excel or Python for post-test analysis

The CSV format is: `timestamp,type,data`

```csv
2026-03-27T14:23:01.123,SENSOR PRESS1: 3.45
2026-03-27T14:23:01.124,SENSOR TEMP1: 25.3
2026-03-27T14:23:01.125,FAULT on solenoid 1!
2026-03-27T14:23:02.001,SOL 1 → OFF
```

---

## How to test without the ESP32

The `add_mock_data()` function at the bottom of `main.py` simulates the ESP32 sending data. Uncomment it in `__main__`:

```python
if __name__ == "__main__":
    app = TestStandGUI()
    add_mock_data(app)      # ← uncomment this line
    app.mainloop()
```

This calls `_handle_message()` directly every 200ms with fake sensor readings and occasional fault events. The GUI behaves exactly as it would with a real ESP32, except no USB cable is needed. Use this to develop and demo the GUI before the hardware is ready.

---

## Dependencies

```bash
pip install pyserial customtkinter matplotlib
```

| Library | Version | What it does |
|---------|---------|-------------|
| `pyserial` | 3.5+ | Talks to the ESP32 over the USB virtual serial port |
| `customtkinter` | 5.0+ | Modern-looking GUI widgets (buttons, sliders, labels) |
| `matplotlib` | 3.7+ | For live plots (to be added in a future version) |
| `tkinter` | (built-in) | The underlying GUI framework. Ships with Python. |
| `threading` | (built-in) | Background thread for serial reading |
| `dataclasses` | (built-in) | The `@dataclass` decorator for protocol classes |

---

## File structure

```
ground-station/
├── main.py              # GUI window — run this
├── serial_comms.py      # USB serial connection handler
├── protocol.py          # Protocol definition (commands + messages)
├── logs/                # Auto-created, contains CSV log files
│   └── log_20260327_142301.csv
└── README.md            # This file
```

---

## Glossary

| Term | Meaning |
|------|---------|
| **Baud rate** | Communication speed in bits per second. 115200 is standard for ESP32. Both sides must agree on the same rate. |
| **Callback** | A function you pass to another module for it to call later. Like leaving your phone number at a restaurant. |
| **Daemon thread** | A background thread that automatically terminates when the main program exits. Set with `daemon=True`. |
| **LEDC** | LED Control — the ESP32's hardware PWM peripheral. Despite the name, it is used for servo control too. Generates precise pulse timing without CPU involvement. |
| **Main thread** | The thread that runs the GUI event loop (`mainloop()`). Only this thread may update widget properties. |
| **MPQ6610** | The half-bridge solenoid driver IC. Receives EN and IN signals from ESP32 GPIOs. Drives 24V through the solenoid coil. Reports faults on the nFAULT pin. |
| **nFAULT** | Active-low fault indicator on the MPQ6610. "n" prefix means the signal is active when LOW. Pulled HIGH by a 10kΩ resistor when everything is fine. |
| **PWM** | Pulse Width Modulation. A digital signal that switches between high and low at a fixed frequency. The ratio of high time to total period (the duty cycle) encodes information — for servos, it encodes the desired angle. |
| **pyserial** | Python library for serial port communication. Provides the `serial.Serial` class that reads/writes bytes over USB. |
| **self.after()** | A tkinter method that schedules a function to run on the main thread after a delay (0ms = as soon as possible). Used to safely pass data from a background thread to the GUI. |
| **Virtual serial port** | When the ESP32-S3's native USB is connected, the operating system creates a software serial port (COM3, /dev/ttyACM0) that programs can read/write as if it were a physical RS-232 port. |