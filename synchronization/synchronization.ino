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
#define PHASE_MAX 255
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

volatile uint8_t buzzer_state = 0;
volatile uint16_t buzzer_delay_us = 1000;

// Toggle buzzer pin using Timer1 Compare Match A interrupt
ISR(TIM1_COMPA_vect) {
  if (buzzer_state) {
    PORTB &= ~(1 << BUZZER);
  } else {
    PORTB |= (1 << BUZZER);
  }
  buzzer_state = !buzzer_state;
}

// Update buzzer frequency based on phase value
void update_buzzer_frequency(uint8_t phase) {
  // Map phase 0–255 to freq 200–4000Hz
  uint16_t freq = 200 + ((3800UL * phase) / PHASE_MAX);
  uint16_t half_period_us = 500000UL / freq;

  cli();
  OCR1A = (half_period_us > 255) ? 255 : (uint8_t)half_period_us; // Limited by 8-bit timer
  sei();
}

void setup() {
  // Configure pins
  DDRB |= (1 << IR_TX);     // IR emitter output
  DDRB &= ~(1 << IR_RX);    // IR receiver input
  PORTB |= (1 << IR_RX);    // Pull-up on receiver
  DDRB |= (1 << LED);       // LED output
  DDRB |= (1 << BUZZER);    // Buzzer output
  PORTB &= ~(1 << LED);     // LED off
  PORTB &= ~(1 << BUZZER);  // Buzzer off

  // Setup Timer1 for buzzer PWM toggle
  TCCR1 = 0;
  GTCCR = 0;
  OCR1C = 255;
  OCR1A = 125; // Start with 1kHz (approx)
  TIMSK |= (1 << OCIE1A);     // Enable compare match A interrupt
  TCCR1 |= (1 << CTC1) | (1 << CS10); // CTC mode, no prescaler

  // NeoPixel begin
  strip.begin();
  strip.show();

  sei(); // Enable global interrupts
}

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    _delay_us(15);
    PORTB &= ~(1 << IR_TX);
    _delay_us(15);
  }
}

void flash_led_rgb() {
  uint8_t r = rand() % 256;
  uint8_t g = rand() % 256;
  uint8_t b = rand() % 256;

  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();

  _delay_ms(100); // Leave LED on briefly

  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
}

int main(void) {
  setup();

  uint8_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;

  while (1) {
    _delay_ms(TICK_DELAY_MS);

    phase += PHASE_STEP;
    if (phase > PHASE_MAX) phase = PHASE_MAX;

    update_buzzer_frequency(phase); // Frequency modulated by phase

    if (phase >= PHASE_MAX) {
      emit_pulse(200);
      flash_led_rgb();
      phase = 0;
      refractory = 20;
    }

    uint8_t current_rx = (PINB & (1 << IR_RX));
    if (last_rx_state && !current_rx && refractory == 0) {
      if (phase > (PHASE_MAX / 4)) {
        uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
        phase += delta;
        if (phase > PHASE_MAX) phase = PHASE_MAX;

        if (PHASE_MAX - phase < JUMP_TO_FLASH_MARGIN) {
          emit_pulse(200);
          flash_led_rgb();
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
