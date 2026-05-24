# CUSF Load Cell Interface Board

## Overview

Signal conditioning board for the PW16A load cell, providing excitation voltage, instrumentation amplification, low-pass filtering, and buffered voltage references for ADC reading.

---

## Connector

| Designator | Description |
|------------|-------------|
| J1 | RS Pro M12 Connector Female, 8W-8C, 5m, PVC, Shield |

---

## Load Cell

| Parameter | Value |
|-----------|-------|
| Model | PW16A |
| Sensitivity | 2 mV/V |
| Excitation Voltage | 10 V |
| Full-scale Output | 20 mV (0.02 V) |

---

## Power Section

### Voltage Regulation & References

| Designator | Part | Description | Package |
|------------|------|-------------|---------|
| U4 | L78L05 | 5V Positive Linear Regulator, 100mA, 30V Max Input | SOIC-8 |
| U2 | REF5010AD | 10V Precision Voltage Reference, 0.1% Accuracy, 10mA, Low Noise | SOIC-8 |
| U5 | REF3033 | 3.3V CMOS Voltage Reference, 50 ppm/°C Max, 50 µA | SOT-23 |

### Op-Amp Buffers

| Designator | Part | Description | Package | Purpose |
|------------|------|-------------|---------|---------|
| U3 | OPA197xD | 36V Precision, Rail-to-Rail I/O, Low Offset Voltage Op-Amp | SOIC-8 | 10V reference buffer |
| U6 | OPA333xxD | 1.8V microPower, CMOS Zero-Drift Op-Amp | SOIC-8 | 3.3V reference buffer |

### Resistors

| Designator | Value | Package |
|------------|-------|---------|
| R2 | 1 Ω | 0805 |
| R3 | 16.2 kΩ | 0805 |
| R4 | 1.8 kΩ | 0805 |

### Capacitors

| Designator | Value | Package |
|------------|-------|---------|
| C3 | 100 nF | 0805 |
| C4 | 100 nF | 0805 |
| C5 | 100 nF | 0805 |
| C6 | 100 nF | 0805 |
| C7 | 10 µF | 0805 |
| C8 | 10 µF | 0805 |
| C9 | 10 µF | 0805 |
| C10 | 1 µF | 0805 |
| C11 | 100 nF | 0805 |
| C12 | 1 µF | 0805 |
| C13 | 100 nF | 0805 |

### Outputs

| Signal | Source | Description |
|--------|--------|-------------|
| VOUT_10V | REF5010AD + OPA197xD buffer | 10V excitation for load cell |
| VREF_0.33V | REF3033 + OPA333xxD buffer | 0.33V reference for ADC |

---

## Analogue Signal Section

### Instrumentation Amplifier

| Designator | Part | Description | Package |
|------------|------|-------------|---------|
| U1 | AD8422 | Single High Performance, Low Power, Rail-to-Rail Precision In-Amp | SOIC-8 |

| Parameter | Value |
|-----------|-------|
| Gain (G) | 150 |
| Gain Resistor (RG) | 133 Ω |
| Formula | RG = 19.8 kΩ / (G − 1) |
| Supply Voltage | +15V (single-supply) |
| Output to ESP | 3.0 V |

### Gain Resistor

| Designator | Value | Package |
|------------|-------|---------|
| RG | 133 Ω | 0805 |

### Low-Pass Filter

| Designator | Value | Package | Notes |
|------------|-------|---------|-------|
| R1 | 5.36 kΩ | 0805 | Series resistor |
| C1 | 10 nF | 0805 | Shunt capacitor |

| Parameter | Value |
|-----------|-------|
| Cut-off Frequency | 2.97 kHz |
| Topology | RC low-pass filter |

### Decoupling Capacitor

| Designator | Value | Package |
|------------|-------|---------|
| C2 | 0.1 µF | 0805 |

### Signals

| Signal | Direction | Description |
|--------|-----------|-------------|
| LC_SIG+ | Input | Load cell differential signal (+) |
| LC_SIG- | Input | Load cell differential signal (−) |
| +15V_Analog | Input | 15V supply for AD8422 |
| V_REF | Input | 0.33V reference for AD8422 output offset |
| ADC_OUT_TO_MIC | Output | Filtered, amplified signal to microcontroller ADC |
| AGND | — | Analogue ground |

---

## Inter-Sheet Connections

| Main Sheet Label | Power Sheet | Analogue Signal Sheet | J1 (M12 Connector) |
|------------------|-------------|----------------------|---------------------|
| VIN_15V | Input | — | — |
| VOUT_10V | ← Output | — | — |
| VREF_0.33V | ← Output | → V_REF | — |
| +10V_EXC | — | — | Pins 1, 8 |
| LC_SIG+ | — | ← Input | Pin (signal) |
| LC_SIG- | — | ← Input | Pin (signal) |
| AGND | ✓ | ✓ | Ground pins |
| ADC_OUT_TO_MIC | — | → Output | — |
