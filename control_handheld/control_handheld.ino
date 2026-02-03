#include <Arduino.h>
#include <avr/io.h>
#include <Adafruit_NeoPixel.h>

// ---------------- USER CONFIG ----------------

// Set to true to emit TURN_ON continuously
// Set to false to emit TURN_OFF continuously
#define EMIT_ON false

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

#define FRAME_SPACING_MS 25

// RGB indicator
#define RGB_PIN PB2
#define NUM_PIXELS 1

// IR output
#define IR_TX PB1

// ------------------------------------------------

Adafruit_NeoPixel strip(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- IR LOW LEVEL ----------------

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    delayMicroseconds(15);
    PORTB &= ~(1 << IR_TX);
    delayMicroseconds(15);
  }
}

static inline void mark_us(uint16_t us) {
  uint16_t cycles = us / 30;
  if (cycles < 1) cycles = 1;
  emit_pulse(cycles);
}

static inline void space_us(uint16_t us) {
  PORTB &= ~(1 << IR_TX);
  delayMicroseconds(us);
}

// ---------------- FRAME FORMAT ----------------

void send_code(uint8_t v) {

  uint8_t inv = ~v;

  // START
  mark_us(6000);
  space_us(3000);

  // First byte
  for (int8_t i = 7; i >= 0; i--) {
    if (v & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  // Second byte (inverted)
  for (int8_t i = 7; i >= 0; i--) {
    if (inv & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  space_us(6000);
}

// ---------------- MAIN ----------------

int main(void) {

  init();

  // Setup RGB
  strip.begin();
  strip.setBrightness(20);
  strip.clear();

  // Setup IR pin
  DDRB |= (1 << IR_TX);
  PORTB &= ~(1 << IR_TX);

  // Choose code
  uint8_t current_code = EMIT_ON ? CODE_TURN_ON : CODE_TURN_OFF;

  // Set indicator color
  if (EMIT_ON) {
    strip.setPixelColor(0, strip.Color(0, 50, 0));   // Green
  } else {
    strip.setPixelColor(0, strip.Color(50, 0, 0));   // Red
  }

  strip.show();

  // Continuous transmit loop
  while (1) {

    send_code(current_code);
    delay(FRAME_SPACING_MS);
  }

  return 0;
}
