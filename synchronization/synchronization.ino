#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

// NeoPixel definitions
#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN  PB2
#define NUM_PIXELS    1
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Behavior tuning
// #define PHASE_MAX 255
uint8_t PHASE_MAX = 255;
#define EPSILON 64
#define PHASE_STEP 1
#define TICK_DELAY_MS 2
#define JUMP_TO_FLASH_MARGIN 16
#define JITTER_INTERVAL 8

// Pins
#define IR_TX PB1
#define IR_RX PB0
#define LED   PB2
#define BUZZER PB3

// Sanguine gradient
#define SANGUINE_START_R 100   // stronger red start
#define SANGUINE_START_G 255    // add some green for orange warmth
#define SANGUINE_START_B 100    // low blue

#define SANGUINE_END_R 255     // max red at end
#define SANGUINE_END_G 183     // more green for bright orange
#define SANGUINE_END_B 0 

void setup() {
  DDRB |= (1 << IR_TX);     // IR emitter output
  DDRB &= ~(1 << IR_RX);    // IR receiver input
  PORTB |= (1 << IR_RX);    // Pull-up on receiver
  DDRB |= (1 << LED);       // LED output
  DDRB |= (1 << BUZZER);    // Buzzer output
  PORTB &= ~(1 << LED);     // LED off
  PORTB &= ~(1 << BUZZER);  // Buzzer off

  TCCR0B = (1 << CS01);     // Timer0 for jitter

  //randomSeed(analogRead(2));
  //PHASE_MAX = random(0, 256);
}

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    _delay_us(15);
    PORTB &= ~(1 << IR_TX);
    _delay_us(15);
  }
}

void beep(uint8_t tone) {
  uint8_t i;
  // Buzzer tone generation for ~100ms
  for (i = 0; i < 100; i++) {
    PORTB |= (1 << BUZZER);
    _delay_us(tone);
    PORTB &= ~(1 << BUZZER);
    _delay_us(tone);

    // // Buzzer tone generation for ~100ms at ~1kHz (1ms period = 0.5ms high + 0.5ms low)
    // for (uint16_t i = 0; i < 100; i++) {
    //   PORTB |= (1 << BUZZER);  // Buzzer ON
    //   _delay_us(500);          // 0.5ms
    //   PORTB &= ~(1 << BUZZER); // Buzzer OFF
    //   _delay_us(500);          // 0.5ms
    // }
    // Delay ON time
    // switch (tone >> 6) {
    //   case 0: _delay_us(250); break;  // tone < 64
    //   case 1: _delay_us(200); break;  // tone < 128
    //   case 2: _delay_us(150); break;  // tone < 192
    //   default: _delay_us(100); break; // tone >= 192
    // }

    // PORTB &= ~(1 << BUZZER);

    // // Delay OFF time — same as ON to keep square wave
    // switch (tone >> 6) {
    //   case 0: _delay_us(250); break;
    //   case 1: _delay_us(200); break;
    //   case 2: _delay_us(150); break;
    //   default: _delay_us(100); break;
    // }
  }
}

void flash_led_rgb() {
  uint8_t r = rand() % 256;
  uint8_t g = rand() % 256;
  uint8_t b = rand() % 256;

  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();

  _delay_ms(100);

  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
}

void phase_rgb(uint8_t phase) {
  //uint8_t r, g, b;

  uint16_t hue = ((uint16_t)phase * 65535UL) / PHASE_MAX;

  uint32_t color = strip.ColorHSV(hue, 255, 255);

  strip.setPixelColor(0, color);
  strip.show();

  //_delay_ms(100);

  // strip.setPixelColor(0, strip.Color(0, 0, 0));
  // strip.show();
}

void phase_rgb_sanguine(uint8_t phase) {
  uint8_t r = SANGUINE_START_R + ((uint16_t)(SANGUINE_END_R - SANGUINE_START_R) * phase) / PHASE_MAX;
  uint8_t g = SANGUINE_START_G + ((uint16_t)(SANGUINE_END_G - SANGUINE_START_G) * phase) / PHASE_MAX;
  uint8_t b = SANGUINE_START_B + ((uint16_t)(SANGUINE_END_B - SANGUINE_START_B) * phase) / PHASE_MAX;

  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}


int main(void) {
  setup();

  uint8_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;
  uint8_t jitter_tick = 0;

  while (1) {
    _delay_ms(TICK_DELAY_MS);

    // if (++jitter_tick >= JITTER_INTERVAL) {
    //   jitter_tick = 0;
    //   uint8_t jitter = (TCNT0 >> 3) & 0x01;
    //   phase += PHASE_STEP + jitter;
    // } else {
    //   phase += PHASE_STEP;
    // }
    phase += PHASE_STEP;

    if (phase > PHASE_MAX) phase = PHASE_MAX;

    if (phase >= PHASE_MAX) {
      emit_pulse(200);
      //flash_led_rgb();
      //phase_rgb(phase);
      phase = 0;
      refractory = 20;
    }
    //beep(phase)
    phase_rgb_sanguine(phase);    

    uint8_t current_rx = (PINB & (1 << IR_RX));
    if (last_rx_state && !current_rx && refractory == 0) {

      // Why this if?
      if (phase > (PHASE_MAX / 4)) {
        uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
        phase += delta;
        if (phase > PHASE_MAX) phase = PHASE_MAX;

        // if (PHASE_MAX - phase < JUMP_TO_FLASH_MARGIN) {
        //   emit_pulse(200);
        //   flash_led_rgb();
        //   phase = 0;
        //   refractory = 20;
        // } else {
        //   refractory = 10;
        // }
      }
    }

    if (refractory > 0) refractory--;
    last_rx_state = current_rx;
  }
}
