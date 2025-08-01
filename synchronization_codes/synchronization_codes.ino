#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

// Include all libraries first
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
// PB4

// Sanguine gradient
#define SANGUINE_START_R 255   // stronger red start
#define SANGUINE_START_G 0    // add some green for orange warmth
#define SANGUINE_START_B 0    // low blue

#define SANGUINE_END_R 255     // max red at end
#define SANGUINE_END_G 183     // more green for bright orange
#define SANGUINE_END_B 0

bool code_received = false;
uint32_t timer = 0;
uint32_t red_timer = 0;
bool red = false;

uint8_t state = 0;

uint32_t start_time;
uint32_t duration;
uint32_t decoding_start_time;
uint8_t code[8];
uint8_t code_num;

void setup_timer1_for_micros() {
  TCCR1 = (1 << CS11); // Clock prescaler = 8 → 1 tick = 1µs at 8MHz

  TCNT1 = 0;            // Reset counter
  TIMSK |= (1 << TOIE1); // Enable overflow interrupt
}
volatile int32_t timer1_overflows = 0;
ISR(TIMER1_OVF_vect) {
  timer1_overflows++;
}

uint32_t my_micros() {
  uint8_t t;
  uint32_t o;

  cli(); // atomic read
  t = TCNT1;
  o = timer1_overflows;
  if ((TIFR & (1 << TOV1)) && (t < 255)) {
    // An overflow happened right after we read
    o++;
  }
  sei();

  int32_t raw_ticks = ((o << 8) | t);
  return (raw_ticks * 1000L) / 3850;
}


void setup() {
  // IR pins
  DDRB |= (1 << IR_TX);      // IR emitter as output
  DDRB &= ~(1 << IR_RX);     // IR receiver as input
  PORTB |= (1 << IR_RX);     // Enable pull-up on receiver
  
  // Configure LED and buzzer
  DDRB |= (1 << LED);        // LED as output
  DDRB |= (1 << BUZZER);     // Buzzer as output
  PORTB &= ~(1 << LED);      // LED off initially
  PORTB &= ~(1 << BUZZER);   // Buzzer off initially
  
  // Initialize timer for jitter (currently unused but ready)
  TCCR0B = (1 << CS01);      // Timer0 prescaler /8
  
  // Initialize NeoPixel
  strip.begin();
  strip.clear();
  strip.show();

  setup_timer1_for_micros();
}

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    _delay_us(15);
    PORTB &= ~(1 << IR_TX);
    _delay_us(15);
  }
}

void send_code(uint8_t code) {
  emit_pulse(100);  // longer pulse for start (40 cycles * 30us = ~1.2ms)

  //_delay_us(600);  // Short gap after start pulse

  // Send bits, MSB first
  for (int8_t i = 7; i >= 0; i--) {
    if (code & (1 << i)) {
      // Send '1' bit: pulse for 15us ON, 15us OFF
      emit_pulse(60); // 1 cycle = 30us total (15us on + 15us off)
    } else {
      // Send '0' bit: no pulse, just delay 30us (one cycle length)
      _delay_us(1800);
    }
  }

}

void print_code() {
  for (int8_t i = 7; i >= 0; i--) {
    if (code_num & (1 << i)) {
      // Send '1' bit: pulse for 15us ON, 15us OFF
      // emit_pulse(60); // 1 cycle = 30us total (15us on + 15us off)
      PORTB |= (1 << PB4);
      _delay_us(300);
    } else {
      // Send '0' bit: no pulse, just delay 30us (one cycle length)
      PORTB &= ~(1 << PB4);
      _delay_us(300);
    }
  }
} 

void construct_code() {
  code_num = 0;
  for (uint8_t i = 4; i < 7; i++) {
    code_num = (code_num << 1) | code[i];
  }
  print_code();
}



void read_code() {
  bool is_high = false;

  if ((PINB & (1 << IR_RX)) == 0) {
    is_high = true;
  } else {
    is_high = false;
  }

  if (state == 0) {
    if (is_high) {
      start_time = my_micros();
      
      state = 1;
      code_received = false;
    }

  } else if (state == 1) {
    if (is_high) {
      duration = my_micros();
      if (duration - start_time > 2750) {
        decoding_start_time = my_micros() + 1600;
        state = 2;
      }
    } else {
      state = 0;
    }

  } else if (state == 2) {
    int32_t delta = my_micros() - decoding_start_time; // ensure 32-bit
    int8_t slot = (delta * 8) / 14500; // maps [0,2249] -> [0,7]
    
    if (slot > 7) slot = 7;

    if (slot > 0) code[slot] = is_high;

    if (slot >= 7) {
      construct_code();
      code_received = true;
      state = 0;
    }
  }
}

void phase_rgb_sanguine(uint8_t phase) {
  uint8_t r = SANGUINE_START_R + ((uint16_t)(SANGUINE_END_R - SANGUINE_START_R) * phase) / PHASE_MAX;
  uint8_t g = SANGUINE_START_G + ((uint16_t)(SANGUINE_END_G - SANGUINE_START_G) * phase) / PHASE_MAX;
  uint8_t b = SANGUINE_START_B + ((uint16_t)(SANGUINE_END_B - SANGUINE_START_B) * phase) / PHASE_MAX;

  if (red) {
    r = 255;
    g = 0;
    b = 0;
  }

  strip.setPixelColor(0, strip.Color(g, r, b));
  strip.show();
}

void beep(uint8_t tone) {
  uint8_t i;
  // Buzzer tone generation for ~100ms
  for (i = 0; i < 100; i++) {
    PORTB |= (1 << BUZZER);
    _delay_us(tone);
    PORTB &= ~(1 << BUZZER);
    _delay_us(tone);
  }
}

void flash_led_rgb() {
  uint8_t r = 255;//rand() % 256;
  uint8_t g = 100; //rand() % 256;
  uint8_t b = 0; //rand() % 256;

  strip.setPixelColor(0, strip.Color(g, r, b));
  strip.show();

  // _delay_ms(100);

  // strip.setPixelColor(0, strip.Color(0, 0, 0));
  // strip.show();
}



int main(void) {
  // cli(); 
  setup();
  // sei();

  uint8_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;
  uint8_t jitter_tick = 0;

  while (1) {
    _delay_ms(TICK_DELAY_MS);
    // phase += PHASE_STEP;

    // uint32_t time = my_micros();

    // if (time - red_timer < 10000) {
    //   red = true;
    // } else {
    //   red = false;
    //   code_num = 0;
    // }

    // read_code();

    // if (code_num == 5) {
    //   red_timer = my_micros();
    //   phase += random(0, 255);
    // }

    // if (time - timer > 2000) {
    //   timer = time;

    phase += PHASE_STEP;
  
    if (phase > PHASE_MAX) phase = PHASE_MAX;

    if (phase >= PHASE_MAX) {
      // emit_pulse(200);
      send_code(170);
      // flash_led_rgb();
      // beep(phase);

      phase = 0;
      refractory = 170;
    }
    
    phase_rgb_sanguine(phase);
    // flash_led_rgb();
    
    uint8_t current_rx = (PINB & (1 << IR_RX));
    // if (last_rx_state && !current_rx && refractory == 0) {
    //   if (phase > (PHASE_MAX / 4)) {
    //     uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
    //     phase += delta;
    //     if (phase > PHASE_MAX) phase = PHASE_MAX;

    //     // Check if close enough to flash immediately
    //     if (PHASE_MAX - phase < JUMP_TO_FLASH_MARGIN) {
    //       emit_pulse(200);
    //       flash_led_rgb();
    //       phase = 0;
    //       refractory = 20;
    //     } else {
    //       refractory = 10;
    //     }
    //   }
    // }

    if (refractory > 0) refractory--;
    last_rx_state = current_rx;
    // }
  }
}
