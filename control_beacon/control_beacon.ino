#include <Arduino.h>
#include <avr/io.h>

#include <Adafruit_NeoPixel.h>

#define RGB_PIN PB2
#define NUM_PIXELS 1

Adafruit_NeoPixel strip(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);


// ----------- CONFIG -----------

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

#define ON_PERIOD_MIN     180    // total ON phase length (e.g. 2 hours)
#define OFF_PERIOD_MIN    60    // total OFF phase length (e.g. 4 hours)

// ON phase behavior
#define ON_FLOOD_DURATION      60000UL        // 1 minute
#define ON_FLOOD_INTERVAL      (10UL * 60000UL)  // every 10 minutes

// OFF phase behavior
#define OFF_FLOOD_DURATION     10000UL        // 10 seconds
#define OFF_FLOOD_INTERVAL     (60UL * 60000UL)  // every 1 hour

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

  // Start
  mark_us(6000);
  space_us(3000);

  // 8 bits MSB first
  for (int8_t i = 7; i >= 0; i--) {
    if (v & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  // End gap
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

  bool in_on_phase = true;

  uint32_t phase_start = millis();
  uint32_t last_flood_start = 0;

  while (1) {

    uint32_t now = millis();

    // ---------- Determine current phase ----------
    uint32_t phase_duration =
      (in_on_phase ? ON_PERIOD_MIN : OFF_PERIOD_MIN) * 60000UL;

    if (now - phase_start >= phase_duration) {
      in_on_phase = !in_on_phase;
      phase_start = now;
      last_flood_start = 0;   // reset scheduling
    }

    // ---------- Select flood parameters ----------
    uint32_t flood_interval =
      in_on_phase ? ON_FLOOD_INTERVAL : OFF_FLOOD_INTERVAL;

    uint32_t flood_duration =
      in_on_phase ? ON_FLOOD_DURATION : OFF_FLOOD_DURATION;

    uint8_t current_code =
      in_on_phase ? CODE_TURN_ON : CODE_TURN_OFF;

    // ---------- Should we start a new flood? ----------
    if (last_flood_start == 0 ||
        now - last_flood_start >= flood_interval) {

        last_flood_start = now;
    }

    bool in_flood_window = (now - last_flood_start < flood_duration);

    // ---------- RGB STATUS INDICATOR ----------
    if (in_flood_window) {
        if (in_on_phase) {
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
    if (now - last_flood_start < flood_duration) {

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
