# CUSF Test Stand — Actuation Board

**Cambridge University Spaceflight · Rocket Engine Test Stand Electronics**

Hardware design files for the actuation board that drives solenoid valves and servo-actuated ball valves on the CUSF static fire test stand. Designed for safety-critical operation with a hardware interlock chain independent of software.

---

## Overview

This board is one part of a multi-board test stand instrumentation system:

| Board | Owner | Function |
|-------|-------|----------|
| **Actuation Board** (this repo) | — | Solenoid + servo valve control, safety interlock |
| Data Acquisition Board | Separate team | Pressure, temperature, thrust measurement |
| Master Laptop | Shared | Control GUI, sequencing, logging |

The actuation board connects to the master laptop over USB (serial) and receives valve commands. It drives solenoid valves through MPQ6610 half-bridge drivers and servo-actuated ball valves through direct PWM. A hardware interlock chain ensures all solenoids de-energise on any fault condition, independent of firmware.

## Architecture

```
                        ┌─────────────────────┐
                        │    MASTER LAPTOP     │
                        │  Python GUI · pyserial│
                        └──────────┬──────────┘
                                   │ USB (CP2102 UART)
                                   │
┌──────────────────────────────────┴──────────────────────────────┐
│                      ACTUATION BOARD                            │
│                                                                 │
│  ┌──────────┐   GPIO    ┌───────────┐         ┌──────────┐    │
│  │  ESP32   │──────────▶│ MPQ6610×4 │────────▶│SOLENOIDS │    │
│  │  WROOM32 │           │(2pop+2DNP)│  24V    │ (valves) │    │
│  │          │   PWM     ├───────────┤         └──────────┘    │
│  │          │──────────▶│ Servo ×6  │────────▶┌──────────┐    │
│  │          │           │(4pop+2DNP)│   5V    │BALL VALVES│   │
│  │          │           └───────────┘         └──────────┘    │
│  │          │  heartbeat                                       │
│  │          │──────────▶┌───────────────────┐                  │
│  │          │           │ HW INTERLOCK CHAIN│──▶ EN_BUS        │
│  └──────────┘           │ARM·ESTOP·WATCHDOG │  (to all MPQ6610)│
│                         └───────────────────┘                  │
└────────────────────────────────────────────────────────────────┘
```

## Key Specs

| Parameter | Value |
|-----------|-------|
| Supply voltage | 24V DC (4–55V supported by MPQ6610) |
| Solenoid channels | 4 footprints (2 populated, 2 DNP) |
| Servo channels | 6 footprints (4 populated, 2 DNP) |
| Max solenoid current | 3A per channel (MPQ6610 limit) |
| MCU | ESP32-WROOM-32 (DevKitC V4) |
| Laptop interface | USB-C (serial, 115200 baud) |
| Interlock timeout | ~100ms (watchdog RC timer) |
| Safety philosophy | Solenoids fail de-energised on any fault |

## Solenoid Driver — MPQ6610

Each solenoid channel uses an [MPQ6610GS-AEC1](https://www.monolithicpower.com/en/mpq6610.html) half-bridge driver (SOIC-8). The MPQ6610 was selected for:

- 4–55V input range (covers 12V and 24V solenoid valves)
- 3A output with cycle-by-cycle current limiting
- Internal current sense (ISET pin) — no external shunt resistor needed
- OCP, UVLO, thermal shutdown, and open-load detection
- nFAULT open-drain output for fault reporting
- AEC-Q100 automotive qualified

### Per-Channel Circuit

```
24V ──┬──[C_VIN 100nF+10µF]──GND
      │
      ├── MPQ6610 VIN (pin 2)
      │
      ├── D (SS34 Schottky, cathode to 24V)──┐
      │                                       │
      │   MPQ6610 OUT (pin 3)─────────────────┼──▶ SOLENOID ──▶ 24V
      │       │                               │
      │   C_BST (100nF)                       │
      │       │                               │
      │   MPQ6610 BST (pin 1) ◀───────────────┘
      │
EN_BUS ──[R 100kΩ pull-up]── MPQ6610 EN (pin 8)
ESP32 GPIO ──────────────── MPQ6610 IN  (pin 7)
      │
      │   MPQ6610 ISET (pin 6) ──┬──[R_ISET]──GND
      │                          ├──[R_HOLD + Q (2N3906)]──GND
      │                          └──▶ ESP32 ADC (current monitor)
      │
3.3V ──[R 10kΩ]── MPQ6610 nFAULT (pin 5) ──▶ ESP32 IRQ (wire-OR)
```

### Hold Current Reduction

Solenoids only need full current for the pull-in stroke (~50–100ms). A PNP transistor (2N3906) switches R_HOLD in parallel with R_ISET after pull-in, reducing the current regulation threshold. This is controlled by a dedicated ESP32 GPIO per channel.

## Safety Interlock

The hardware interlock chain is a series circuit — all elements must be closed for EN_BUS to be high. Any single break kills all solenoid drivers.

```
+24V ──[47kΩ pull-up]──┬── ARM Switch (NO) ── E-STOP (NC) ── Watchdog ──┬── EN_BUS
                        │                                                 │
                        └── SW Abort (N-FET, ESP32 GPIO) ─────────────────┘
```

| Element | Type | Failure Mode |
|---------|------|--------------|
| ARM switch | Normally open (key) | Must be actively closed to arm |
| E-STOP | Normally closed (remote) | Pressing opens circuit → safe |
| Watchdog | RC timer + comparator | ESP32 must toggle >10Hz or timeout trips |
| SW abort | N-channel FET | ESP32 can pull chain to GND in firmware |

**The interlock is independent of firmware.** If the ESP32 hangs, the watchdog RC timer charges up within ~100ms and the comparator breaks the chain. If USB disconnects, the ESP32 stops receiving heartbeat packets and stops toggling the watchdog line — same result.

## ESP32 GPIO Assignment

### Solenoid Control

| GPIO | Function | Populated |
|------|----------|-----------|
| 25 | SOL1 IN (drive) | ✅ |
| 26 | SOL2 IN (drive) | ✅ |
| 27 | SOL3 IN (drive) | DNP |
| 14 | SOL4 IN (drive) | DNP |
| 16 | SOL1 hold current (Q base) | ✅ |
| 17 | SOL2 hold current (Q base) | ✅ |
| 18 | SOL3 hold current (Q base) | DNP |
| 19 | SOL4 hold current (Q base) | DNP |

### Servo PWM

| GPIO | Function | Populated |
|------|----------|-----------|
| 32 | SRV1 PWM (50Hz) | ✅ |
| 33 | SRV2 PWM (50Hz) | ✅ |
| 23 | SRV3 PWM (50Hz) | ✅ |
| 22 | SRV4 PWM (50Hz) | ✅ |
| 21 | SRV5 PWM (50Hz) | DNP |
| 12 | SRV6 PWM (50Hz) | DNP |

### ADC / Monitoring

| GPIO | Function | Populated |
|------|----------|-----------|
| 34 | nFAULT wire-OR (interrupt) | ✅ |
| 35 | ISET CH1 ADC | ✅ |
| 36 | ISET CH2 ADC | ✅ |
| 39 | ISET CH3 ADC | DNP |

### Safety / Status

| GPIO | Function |
|------|----------|
| 13 | Watchdog heartbeat output |
| 15 | Software abort (N-FET gate) |
| 4 | Interlock status readback (ADC) |
| 2 | LED ARM (green) |
| 0 | LED FAULT (red) |
| 5 | LED COMMS (blue) |

## Power Distribution

```
24V DC IN ──[F1 5A]──[D1 SMAJ24A TVS]──┬── 24V Bus (solenoids via MPQ6610)
                                         │
                                    [LM2596-5.0]
                                         │
                                    ┌── 5V Bus ──[470µF bulk]── Servo VCC
                                    │
                                [AMS1117-3.3]
                                    │
                                    └── 3.3V Bus ── ESP32 logic
```

| Rail | Regulator | Max Current | Load |
|------|-----------|-------------|------|
| 24V | Direct (fused) | 5A (fuse) | Solenoid valves |
| 5V | LM2596-5.0 buck | 3A | Servos (consider TPS54560 5A if 4+ servos stall) |
| 3.3V | AMS1117-3.3 LDO | 1A | ESP32, pull-ups, LEDs |

> ⚠️ **Note:** If all 4 servos stall simultaneously, peak current may exceed the LM2596's 3A rating. Consider upgrading to TPS54560 (5A) if this is a concern for your servo selection.

## Communication Protocol

Binary packet format over USB serial at 115200 baud:

```
[0xAA] [0x55] [Board ID] [Msg Type] [Length] [Payload...] [CRC16]
```

| Msg Type | Direction | Description |
|----------|-----------|-------------|
| 0x01 | Laptop → Board | Valve command (solenoid on/off, servo position) |
| 0x02 | Board → Laptop | Telemetry (ISET currents, fault status, interlock state) |
| 0x03 | Bidirectional | Heartbeat |
| 0x04 | Laptop → Board | Configuration (hold current timing, servo limits) |
| 0x10 | Laptop → Board | Abort command |

The ESP32 expects a heartbeat packet from the laptop at least every 100ms. If heartbeats stop arriving, the firmware stops toggling the watchdog GPIO, which triggers the hardware interlock timeout.

## Repository Structure

```
├── hardware/
│   ├── kicad/              # KiCad project files
│   │   ├── actuation.kicad_pro
│   │   ├── actuation.kicad_sch
│   │   ├── actuation.kicad_pcb
│   │   └── libs/           # Custom footprints & symbols
│   ├── bom/                # Bill of materials
│   └── datasheets/         # Component datasheets (MPQ6610, etc.)
│
├── firmware/
│   ├── src/
│   │   ├── main.cpp        # Entry point, setup, loop
│   │   ├── solenoid.cpp    # MPQ6610 driver (IN, hold, fault handling)
│   │   ├── servo.cpp       # Servo PWM control
│   │   ├── interlock.cpp   # Watchdog heartbeat, abort logic
│   │   ├── comms.cpp       # Serial protocol, packet parsing
│   │   └── telemetry.cpp   # ADC reading, current monitoring
│   ├── include/
│   ├── platformio.ini      # PlatformIO config (ESP32)
│   └── test/
│
├── docs/
│   ├── schematic_revB.html # Interactive schematic reference
│   ├── block_diagram.html  # Board block diagram
│   └── system_design.md    # Full system design document
│
└── README.md               # This file
```

## BOM Summary

### Populated Components

| Ref | Component | Package | Qty | Notes |
|-----|-----------|---------|-----|-------|
| U1 | LM2596-5.0 | TO-263 | 1 | 24V→5V buck |
| U2 | AMS1117-3.3 | SOT-223 | 1 | 5V→3.3V LDO |
| U3 | ESP32-DevKitC V4 | Module | 1 | MCU |
| U4, U5 | MPQ6610GS-AEC1-P | SOIC-8 | 2 | Solenoid drivers |
| U7 | TLV3201 | SOT-23-5 | 1 | Watchdog comparator |
| Q1, Q2 | 2N3906 | SOT-23 | 2 | Hold current PNP |
| Q4 | 2N7002 | SOT-23 | 1 | SW abort N-FET |
| D1 | SMAJ24A | SMA | 1 | Input TVS |
| D2 | 1N5824 | DO-201 | 1 | Buck catch diode |
| D3, D4 | SS34 | SMA | 2 | Solenoid flyback Schottky |
| L1 | 33µH / 3A | — | 1 | Buck inductor |
| F1 | 5A fuse | — | 1 | Input protection |
| C_BST | 100nF X7R ≥16V | 0603 | 2 | Bootstrap caps |
| C_VIN | 100nF + 10µF ≥50V | 0603/0805 | 2 sets | Per MPQ6610 |
| R_ISET | TBD | 0603 | 2 | **Pending solenoid specs** |
| R_HOLD | TBD | 0603 | 2 | **Pending solenoid specs** |
| Connectors | Molex Micro-Fit 3.0 (2pin) | — | 3 | Power + 2 solenoids |
| Connectors | JST-XH (3pin) | — | 4 | 4 servos |

### DNP Components (footprint only)

| Ref | Component | Qty | Notes |
|-----|-----------|-----|-------|
| U5, U6 | MPQ6610GS-AEC1-P | 2 | Spare solenoid channels |
| Q3 + support | 2N3906 + resistors | 2 sets | Spare hold circuits |
| D5, D6 | SS34 | 2 | Spare flyback diodes |
| J10, J11 | JST-XH 3pin | 2 | Spare servo headers |

## Getting Started

### Hardware

1. **Review schematics** — open `docs/schematic_revB.html` in a browser (interactive, 4 sheets).
2. **Get solenoid specs** — R_ISET and R_HOLD values are blocked on knowing the solenoid pull-in and hold current.
3. **Open KiCad project** — `hardware/kicad/actuation.kicad_pro`. Schematic entry based on reference schematic.
4. **Order components** — BOM in `hardware/bom/`. MPQ6610GS-AEC1-P available on Mouser/Digikey.

### Firmware

```bash
# Install PlatformIO
pip install platformio

# Clone and build
git clone https://github.com/cusf/test-stand-actuation.git
cd test-stand-actuation/firmware
pio run

# Flash to ESP32 (connect USB)
pio run --target upload

# Monitor serial
pio device monitor --baud 115200
```

### Calculating R_ISET

Once solenoid specs are known:

```
R_ISET = V_TRIP / (I_SET_RATIO × I_LIMIT)

Where:
  V_TRIP  = 1.5V (internal comparator threshold)
  I_SET_RATIO = 100µA/A (ISET current per amp of load)
  I_LIMIT = desired current regulation threshold (amps)

Example: For 1.5A current limit:
  R_ISET = 1.5V / (100µA/A × 1.5A) = 1.5V / 150µA = 10kΩ

ISET pin voltage = R_ISET × I_SET_RATIO × I_LOAD
  With 10kΩ: V_ISET = 10kΩ × 100µA/A × I_LOAD = 1.0V/A
```

## Dependencies

| Tool | Version | Purpose |
|------|---------|---------|
| KiCad | 8.0+ | Schematic & PCB design |
| PlatformIO | Latest | Firmware build system |
| ESP-IDF | Via PlatformIO | ESP32 framework |
| Python 3 | 3.10+ | Laptop control software |
| pyserial | Latest | Serial communication |

## Status

- [ ] System architecture defined
- [ ] Component selection (MPQ6610, ESP32)
- [ ] Reference schematic (interactive HTML)
- [ ] GPIO assignment finalised
- [ ] Safety interlock chain designed
- [ ] KiCad schematic entry
- [ ] PCB layout
- [ ] Firmware — solenoid driver
- [ ] Firmware — servo control
- [ ] Firmware — serial protocol
- [ ] Firmware — interlock watchdog
- [ ] Laptop GUI
- [ ] Board fabrication & assembly
- [ ] Integration testing

## Blocked Items

| Item | Blocked On | Impact |
|------|-----------|--------|
| R_ISET values | Solenoid valve specs (voltage, pull-in current, hold current) | Cannot finalise current regulation |
| R_HOLD values | Solenoid hold current requirement | Cannot set hold current threshold |
| 5V regulator selection | Servo stall current (may need upgrade from LM2596 to TPS54560) | Power budget |

## Contributing

This is a CUSF internal project. If you're a CUSF member working on the test stand:

1. Create a feature branch from `main`
2. Make changes, test on hardware where possible
3. Open a PR with a description of what changed and why
4. Get review from at least one other team member

For schematic changes, always export updated PDFs into `docs/` alongside the KiCad files.

## References

- [MPQ6610 Datasheet](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MPQ6610GS-AEC1/document_id/9493/)
- [ESP32-WROOM-32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)
- [LM2596 Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)
