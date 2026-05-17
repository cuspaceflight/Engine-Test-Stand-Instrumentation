/*
 * calibration.h — Sensor scaling coefficients for CAN-received raw counts.
 *
 * The instrumentation board sends raw ADC/IC counts over CAN. This firmware
 * converts them to engineering units before forwarding to the GUI as SENSOR:
 * messages. Fill in the real values once each sensor chain is characterised.
 *
 * Conversion formula applied to every channel:
 *
 *     engineering_value = raw_count * SCALE + OFFSET
 *
 * Until calibrated, leave SCALE = 1.0 and OFFSET = 0.0. The GUI pipeline
 * works end-to-end and you can verify that CAN frames are arriving and being
 * decoded correctly; the numbers will just be raw counts, not bar / °C / N.
 *
 * How to calibrate (pressure example):
 *   1. Apply a known pressure to the transducer and record the raw count
 *      printed by the GUI.
 *   2. Repeat at a second known pressure.
 *   3. Compute: SCALE  = (P2 - P1) / (raw2 - raw1)
 *               OFFSET = P1 - raw1 * SCALE
 *   4. Update the arrays below, rebuild, and reflash.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

// ─── Pressure transducers (ADS1115 raw counts → bar) ────────────────────────
// Eight channels; arrays are 1-indexed (element [0] is unused).
// TODO: replace 1.0 / 0.0 with real coefficients once sensors are benched.
const float PRESS_SCALE[9]  = {
    0.0f,   // [0] unused
    1.0f,   // PRESS1
    1.0f,   // PRESS2
    1.0f,   // PRESS3
    1.0f,   // PRESS4
    1.0f,   // PRESS5
    1.0f,   // PRESS6
    1.0f,   // PRESS7
    1.0f,   // PRESS8
};
const float PRESS_OFFSET[9] = {
    0.0f,   // [0] unused
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// ─── Thermocouples (MAX31856 linearised register → °C) ───────────────────────
// The MAX31856 linearised thermocouple temperature register is 19 bits,
// signed, with 1 LSB = 1/64 °C (0.015625 °C). The instrumentation firmware
// packs this into an int16 — confirm the exact packing (e.g. upper 16 bits
// of the 19-bit value → effective LSB = 4/64 = 1/16 °C) and update SCALE.
// TODO: verify packing format with instrumentation board firmware.
const float TEMP_SCALE[5]  = {
    0.0f,   // [0] unused
    1.0f,   // TEMP1
    1.0f,   // TEMP2
    1.0f,   // TEMP3
    1.0f,   // TEMP4
};
const float TEMP_OFFSET[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// ─── Load cell (HX711 raw counts → Newtons) ──────────────────────────────────
// The HX711 is a 24-bit ADC; the instrumentation board transmits the reading
// sign-extended to int32. Typical calibration procedure:
//   1. Record raw count with no load (tare).  Set FORCE_OFFSET = -tare_count.
//   2. Apply a known weight W (kg). Record raw count C.
//      Set FORCE_SCALE = W * 9.81 / C  (convert kg to Newtons).
// TODO: calibrate with known weights.
const float FORCE_SCALE  = 1.0f;
const float FORCE_OFFSET = 0.0f;

#endif // CALIBRATION_H
