// Simple ATTiny85 blink test
// Toggles PB1 (Arduino digital pin 1) using millis().
// Make sure the ATTinyCore board/clock setting matches your fuses
// so millis() is accurate.
#include <Arduino.h>

const uint8_t LED_PIN = 1; // PB1 on ATTiny85

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static uint32_t last_toggle_ms = 0;
  static bool led_on = false;

  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - last_toggle_ms) >= 5000UL) {
    last_toggle_ms = now_ms;
    led_on = !led_on;
    digitalWrite(LED_PIN, led_on ? HIGH : LOW);
  }
}
