/*
 * Arduino Nano 33 IoT low-side shunt ammeter.
 *
 * Wiring:
 *   Battery/load negative -> SHUNT -> battery negative / Nano GND
 *   A0 -> load side of shunt
 *   Nano GND -> battery negative side of shunt
 *
 * Configure SHUNT_OHMS below to match the resistor you installed.
 * The sketch prints CSV:
 *   millis,current_mA,voltage_mV,raw_avg,raw_min,raw_max
 */

#include <Arduino.h>

const uint8_t SENSE_PIN = A0;

// Pick one:
//   10.0 ohm: good general choice if this is your smallest resistor
//   100.0 ohm: sleep/current-leakage measurements only
const float SHUNT_OHMS = 10.0f;

// Nano 33 IoT analog input range is 0..3.3 V with the default analog reference.
const float ADC_REF_MV = 3300.0f;

#if defined(ARDUINO_ARCH_SAMD)
  const uint16_t ADC_MAX_COUNTS = 4095;
#else
  const uint16_t ADC_MAX_COUNTS = 1023;
#endif

// Averaging improves noise, but does not remove systematic ADC/reference error.
const uint16_t SAMPLES_PER_READING = 512;
const uint16_t SAMPLE_DELAY_US = 150;
const uint16_t PRINT_INTERVAL_MS = 200;

// Measure with A0 tied to GND and put the observed raw_avg here if needed.
// Usually 0 is fine; use this only if you see a stable nonzero offset.
const float ZERO_COUNTS = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  #if defined(ARDUINO_ARCH_SAMD)
    analogReadResolution(12);
  #endif
  pinMode(SENSE_PIN, INPUT);

  Serial.println(F("millis,current_mA,voltage_mV,raw_avg,raw_min,raw_max"));
}

void loop() {
  static uint32_t last_print_ms = 0;

  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - last_print_ms) < PRINT_INTERVAL_MS) {
    return;
  }
  last_print_ms = now_ms;

  uint32_t sum = 0;
  uint16_t raw_min = ADC_MAX_COUNTS;
  uint16_t raw_max = 0;

  for (uint16_t i = 0; i < SAMPLES_PER_READING; i++) {
    uint16_t raw = analogRead(SENSE_PIN);
    sum += raw;
    if (raw < raw_min) raw_min = raw;
    if (raw > raw_max) raw_max = raw;
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  float raw_avg = (float)sum / (float)SAMPLES_PER_READING;
  float corrected_counts = raw_avg - ZERO_COUNTS;
  if (corrected_counts < 0.0f) corrected_counts = 0.0f;

  float shunt_mV = corrected_counts * ADC_REF_MV / (float)ADC_MAX_COUNTS;
  float current_mA = shunt_mV / SHUNT_OHMS;

  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(current_mA, 4);
  Serial.print(',');
  Serial.print(shunt_mV, 4);
  Serial.print(',');
  Serial.print(raw_avg, 4);
  Serial.print(',');
  Serial.print(raw_min);
  Serial.print(',');
  Serial.println(raw_max);
}
