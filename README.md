# Cambridge University Space Flight Engine Test Stand

This repo contains documents for CUSF engine test stand. The electronics are split across 2 circuit boards:

**Servo + Solenoid Board**  Drives 2 solenoid valves (MPQ6610 half-bridge ICs) and up to 4 servos
**Instrumentation Board** Reads pressure transducers, thermocouples, and a load cell for thrust measurement

A laptop running ground-station software sends commands to the ESP32 on the servo board, and polls the Instrumentation Board to read sensor data

## Board Details

### Servo + Solenoid Board

This board contains the circuits for driving the servos and solenoids. Uses an ESP32 microcontroller (ESP32 S3 DevKit C1). The microcontroller is alos responsible for: 
 - Communication with laptop
 - Driving servos and solenoids
 - Inputs raw data from sensors and sends to laptop

#### Solenoid Section

Each of the solenoid channels uses an **MPS MPQ6610** half-bridge power driver. 

#### Servo Section

4 servos (FeeTech 180 Degrees Digital Servo 7.4V 35kg/cm FT5330M)
 - PWM signal from ESP32

### Instrumentation Board

- Load cell
- 8 Pressure sensors
- 4 thermocouples

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
│
├── Software/
│   ├── servo/
│   ├── solenoid/
│   ├── instrumentation/
│   ├──Laptop software (GUI + logging)
│
└── README.md                  
