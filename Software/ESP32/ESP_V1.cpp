#include <Arduino.h>
#include <ESP32Servo.h>

// Servo + Solenoid Board for CUSF engine test stand
// Controls 4 servos and 2 solenoids, and reports instrumentation board data.

// Servo pins
const int servoPins[4] = {18, 19, 21, 22};
Servo servos[4];

// Solenoid driver pins (MPQ6610 half-bridge inputs)
const int solenoidPins[2] = {25, 26};

// nFAULT pins from MPQ6610
const int faultPins[2] = {8, 9};

// Instrumentation analog inputs
const int loadCellPin = 34;
const int pressurePins[8] = {32, 33, 35, 36, 39, 14, 27, 13};
const int thermocouplePins[4] = {15, 2, 4, 16};

// Communication buffer
String inputBuffer = "";

// Timing for periodic reports
unsigned long lastReportTime = 0;
const unsigned long reportInterval = 1000; // 1 second

void setSolenoid(int index, bool state) {
  if (index < 0 || index >= 2) return;
  digitalWrite(solenoidPins[index], state ? HIGH : LOW);
}

void setServoPulse(int index, int pulse_us) {
  if (index < 0 || index >= 4) return;
  pulse_us = constrain(pulse_us, 500, 2500);
  servos[index].writeMicroseconds(pulse_us);
}

int readAnalogPin(int pin) {
  return analogRead(pin);
}

float readLoadCell() {
  int raw = readAnalogPin(loadCellPin);
  // Placeholder conversion. Calibrate with actual load cell amplifier/resistance.
  return (raw / 4095.0f) * 3.3f;
}

float readPressure(int index) {
  if (index < 0 || index >= 8) return 0.0f;
  int raw = readAnalogPin(pressurePins[index]);
  return (raw / 4095.0f) * 3.3f;
}

float readThermocouple(int index) {
  if (index < 0 || index >= 4) return 0.0f;
  int raw = readAnalogPin(thermocouplePins[index]);
  return (raw / 4095.0f) * 3.3f;
}

bool readFault(int index) {
  if (index < 0 || index >= 2) return true; // assume fault if invalid
  return digitalRead(faultPins[index]) == LOW; // nFAULT is active low
}

void reportSensors() {
  // Send sensor data as separate SENSOR messages
  for (int i = 0; i < 8; i++) {
    Serial.print("SENSOR:PRESS");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.println(readPressure(i), 3);
  }
  for (int i = 0; i < 4; i++) {
    Serial.print("SENSOR:TEMP");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.println(readThermocouple(i), 3);
  }
  Serial.print("SENSOR:THRUST:");
  Serial.println(readLoadCell(), 3);
}

void reportFaults() {
  for (int i = 0; i < 2; i++) {
    Serial.print("FAULT:");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.println(readFault(i) ? "1" : "0");
  }
}

void processCommand(const String &cmd) {
  if (cmd.length() == 0) return;

  String token = cmd;
  token.trim();

  // Parse SOL{channel}:{ON|OFF}
  if (token.startsWith("SOL")) {
    int colonIndex = token.indexOf(':');
    if (colonIndex > 3) {
      String channelStr = token.substring(3, colonIndex);
      int channel = channelStr.toInt() - 1; // 1-based to 0-based
      String state = token.substring(colonIndex + 1);
      bool on = (state == "ON");
      setSolenoid(channel, on);
      Serial.println("OK");
      return;
    }
  }

  // Parse SRV{channel}:{pulse_us}
  if (token.startsWith("SRV")) {
    int colonIndex = token.indexOf(':');
    if (colonIndex > 3) {
      String channelStr = token.substring(3, colonIndex);
      int channel = channelStr.toInt() - 1; // 1-based to 0-based
      String pulseStr = token.substring(colonIndex + 1);
      int pulse_us = pulseStr.toInt();
      setServoPulse(channel, pulse_us);
      Serial.println("OK");
      return;
    }
  }

  // Parse STATUS
  if (token == "STATUS") {
    Serial.println("STATUS:SOL1:OFF,SOL2:OFF,SRV1:1500,SRV2:1500,SRV3:1500,SRV4:1500");
    return;
  }

  Serial.println("ERROR:Invalid command");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("CUSF Test Stand ESP32 Initialized");

  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i], 500, 2400);
    servos[i].writeMicroseconds(1500); // center position
  }

  for (int i = 0; i < 2; i++) {
    pinMode(solenoidPins[i], OUTPUT);
    digitalWrite(solenoidPins[i], LOW);
    pinMode(faultPins[i], INPUT_PULLUP);
  }

  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(loadCellPin, ADC_11db);
  for (int i = 0; i < 8; i++) analogSetPinAttenuation(pressurePins[i], ADC_11db);
  for (int i = 0; i < 4; i++) analogSetPinAttenuation(thermocouplePins[i], ADC_11db);
}

void loop() {
  // Process incoming commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Periodic reporting
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval) {
    reportSensors();
    reportFaults();
    lastReportTime = currentTime;
  }
}
