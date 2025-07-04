#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define PHASE_MAX 255
#define EPSILON 64
#define PHASE_STEP 1
#define TICK_DELAY_MS 2
#define JUMP_TO_FLASH_MARGIN 16
#define JITTER_INTERVAL 8  // apply jitter every N ticks

// Pins
#define IR_TX PB1
#define IR_RX PB0
#define LED   PB2

void setup() {
  DDRB |= (1 << IR_TX);   // IR emitter output
  DDRB &= ~(1 << IR_RX);  // IR receiver input
  PORTB |= (1 << IR_RX);  // pull-up
  DDRB |= (1 << LED);     // visible LED output
  PORTB &= ~(1 << LED);   // LED off

  // Start Timer0 for jitter source
  TCCR0B = (1 << CS01); // prescaler /8
}

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    _delay_us(15);
    PORTB &= ~(1 << IR_TX);
    _delay_us(15);
  }
}

void flash_led() {
  PORTB |= (1 << LED);
  _delay_ms(10);
  PORTB &= ~(1 << LED);
}

int main(void) {
  setup();

  uint8_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;
  uint8_t jitter_tick = 0;

  while (1) {
    _delay_ms(TICK_DELAY_MS);

    // Add jitter only every N ticks
    if (++jitter_tick >= JITTER_INTERVAL) {
      jitter_tick = 0;
      uint8_t jitter = (TCNT0 >> 3) & 0x01;  // weak jitter: bit 3 of TCNT0
      phase += PHASE_STEP + jitter;
    } else {
      phase += PHASE_STEP;
    }

    if (phase > PHASE_MAX) phase = PHASE_MAX;

    // Emit flash when threshold reached
    if (phase >= PHASE_MAX) {
      emit_pulse(200);
      flash_led();
      phase = 0;
      refractory = 20;
    }

    // Detect falling edge from IR receiver
    uint8_t current_rx = (PINB & (1 << IR_RX));
    if (last_rx_state && !current_rx && refractory == 0) {
      // Only respond if phase > 25% full
      if (phase > (PHASE_MAX / 4)) {
        uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
        phase += delta;
        if (phase > PHASE_MAX) phase = PHASE_MAX;

        if (PHASE_MAX - phase < JUMP_TO_FLASH_MARGIN) {
          emit_pulse(200);
          flash_led();
          phase = 0;
          refractory = 20;
        } else {
          refractory = 10;
        }
      }
    }

    if (refractory > 0) refractory--;
    last_rx_state = current_rx;
  }
}
