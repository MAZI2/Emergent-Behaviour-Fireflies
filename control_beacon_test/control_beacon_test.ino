#include <Arduino.h>
#include <avr/io.h>

#include <Adafruit_NeoPixel.h>

#define RGB_PIN PB2
#define NUM_PIXELS 1

Adafruit_NeoPixel strip(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);


// ----------- CONFIG -----------

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

#define BURST_DURATION_MS  60000UL        // 60 seconds
#define QUIET_DURATION_MS  (6UL * 60000UL)
#define CYCLE_DURATION_MS  ((BURST_DURATION_MS + QUIET_DURATION_MS) * 2UL)

#define FRAME_SPACING_MS  25

#define IR_TX PB1


// ----------- IR LOW-LEVEL -----------

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


// ----------- FRAME FORMAT -----------

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



// ----------- MAIN -----------

int main(void) {

  init();

  strip.begin();
  strip.setBrightness(20);   // dim (0–255)
  strip.clear();
  strip.show();


  DDRB |= (1 << IR_TX);
  PORTB &= ~(1 << IR_TX);

  uint32_t cycle_start = millis();

  while (1) {

    uint32_t now = millis();

    uint32_t cycle_elapsed = now - cycle_start;
    if (cycle_elapsed >= CYCLE_DURATION_MS) {
      cycle_start = now;
      cycle_elapsed = 0;
    }

    bool emit_on =
      cycle_elapsed < BURST_DURATION_MS;

    bool emit_off =
      cycle_elapsed >= (BURST_DURATION_MS + QUIET_DURATION_MS) &&
      cycle_elapsed <  (BURST_DURATION_MS + QUIET_DURATION_MS + BURST_DURATION_MS);

    bool in_flood_window = emit_on || emit_off;
    uint8_t current_code = emit_on ? CODE_TURN_ON : CODE_TURN_OFF;

    // ---------- RGB STATUS INDICATOR ----------
    if (in_flood_window) {
        if (emit_on) {
            // Emitting TURN_ON
            strip.setPixelColor(0, strip.Color(0, 50, 0));   // dim green
        } else {
            // Emitting TURN_OFF
            strip.setPixelColor(0, strip.Color(50, 0, 0));   // dim red
        }
    } else {
        // Quiet
        strip.setPixelColor(0, strip.Color(0, 0, 50));       // dim blue
    }

    strip.show();


    // ---------- Are we inside flood window? ----------
    if (in_flood_window) {

        send_code(current_code);
        delay(FRAME_SPACING_MS);
    }
    else {
        // quiet period
        delay(200);
    }
  }

  return 0;
}
