# Cambridge University Space Flight Engine Test Stand

**Hardware, firmware, and documentation for the electronics that control and instrument the Cambridge University Spaceflight static fire test stand.**

This repo contains documents for CUSF engine test stand. The electronics are split across 2 circuit boards:

| **Servo + Solenoid Board**  Drives 2 solenoid valves (MPQ6610 half-bridge ICs) and up to 4 servo-actuated ball valves with position feedback.|
| **Instrumentation Board** Reads pressure transducers, thermocouples, and a load cell for thrust measurement.|

A laptop running ground-station software sends commands to the ESP32 on the servo board, and polls the Instrumentation Board (to read sensor data back).

## Board Details

### Servo + Solenoid Board

This board directly controls all actuated valves on the test stand. It combines solenoid driving (for fast on/off valves) and servo driving (for proportional ball valves) onto a single PCB. Uses an ESP32 microcontroller to control servos and solenoids. 

#### Solenoid Section

Each of the solenoid channels uses an **MPS MPQ6610** half-bridge power driver. 

#### Servo Section

Up to 4 servos
 - PWM signal from ESP32

### Instrumentation Board

This board is the sensor interface. It conditions and digitises analog signals from the test stand sensors and makes the data available to the Master Board over the inter-board bus.

## Repository Structure

```
cusf-engine_test_stand/
│
├── hardware/
│   ├── master-board/
│   │   ├── kicad/              # Schematic + PCB project
│   │
│   ├── servo-solenoid-board/
│   │   ├── kicad/
│   │
│   ├── instrumentation-board/
│   │   ├── kicad/
│   │
│   ├── Datasheets
│   │   ├── MPQ6610.pdf
│   │
│   └── lib/                    # Shared KiCad symbols + footprints
│       ├── cusf-symbols.kicad_sym
│       └── cusf-footprints.pretty/
│
├── firmware/
│   ├── master/
│   ├── servo/
│   ├── solenoid/
│   ├── instrumentation/
│
├── Laptop software (GUI + logging) #Development hasn't started
│
├── Documents/
│   ├── design.docx   
│   └── ...
│
└── README.md                  
