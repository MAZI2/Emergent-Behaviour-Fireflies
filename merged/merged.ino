/*
 * Firefly Synchronization Device
 * 
 * This code implements a firefly-like synchronization behavior using:
 * - IR communication for detecting other fireflies
 * - RGB LED for visual flash
 * - Buzzer for audio chirp
 * 
 * Based on the Kuramoto model for oscillator synchronization
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>

// ===== HARDWARE CONFIGURATION =====
#define NEOPIXEL_PIN    PB2
#define NUM_PIXELS      1
#define IR_TX           PB1
#define IR_RX           PB0
#define LED             PB2
#define BUZZER          PB3

// ===== BEHAVIOR PARAMETERS =====
#define EPSILON                 64
#define PHASE_STEP              1
#define TICK_DELAY_MS           2
#define JUMP_TO_FLASH_MARGIN    16
#define JITTER_INTERVAL         8

// ===== AUDIO PARAMETERS =====
#define CHIRP_BASE_DELAY        175
#define CHIRP_STEPS             8
#define CHIRP_CYCLES_PER_STEP   20
#define CHIRP_DELAY_DECREMENT   5
#define CHIRP_MIN_DELAY         20
#define CHIRP_PAUSE_MS          10

// ===== TIMING PARAMETERS =====
#define IR_PULSE_HALF_PERIOD    15
#define REFRACTORY_FLASH        20
#define REFRACTORY_TRIGGER      10

const uint8_t GRADIENT_START_R[4] = {100, 255,   0, 200};
const uint8_t GRADIENT_START_G[4] = {255, 255, 0, 210};
const uint8_t GRADIENT_START_B[4] = {100,   0,   255,   0};

const uint8_t GRADIENT_END_R[4] = {255, 255,   0, 255};
const uint8_t GRADIENT_END_G[4] = {183,   0,   255, 110};
const uint8_t GRADIENT_END_B[4] = {  0, 255, 0,   0};

// ===== GLOBAL VARIABLES =====
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== Dynamic Phase Configuration =====
const uint16_t PHASE_SEQUENCE[] = {255, 450, 210, 350};
#define PHASE_SEQUENCE_LEN (sizeof(PHASE_SEQUENCE) / sizeof(PHASE_SEQUENCE[0]))
uint8_t phase_index = 0;
uint16_t PHASE_MAX = 255;

uint16_t tick_counter = 0;
#define SWITCH_INTERVAL_MS (2400000) // 4 minutes
#define TICKS_PER_SWITCH (SWITCH_INTERVAL_MS / TICK_DELAY_MS)

// ===== FUNCTION PROTOTYPES =====
void setup_hardware(void);
void emit_ir_pulse(uint16_t cycles);
void delay_us_custom(uint16_t us);
void chirp(void);
void half_chirp(void);
void set_fade_color(uint16_t phase, uint32_t diff);

// codes
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
/////////

/**
 * Initialize hardware peripherals
 */
void setup_hardware(void) {
    // Configure IR pins
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

/**
 * Emit IR pulse train for communication
 * @param cycles Number of on/off cycles to transmit
 */
void emit_ir_pulse(uint16_t cycles) {
    for (uint16_t i = 0; i < cycles; i++) {
        PORTB |= (1 << IR_TX);
        _delay_us(IR_PULSE_HALF_PERIOD);
        PORTB &= ~(1 << IR_TX);
        _delay_us(IR_PULSE_HALF_PERIOD);
    }
}

void send_code(uint8_t code) {
  emit_ir_pulse(100);  // longer pulse for start (40 cycles * 30us = ~1.2ms)

  //_delay_us(600);  // Short gap after start pulse

  // Send bits, MSB first
  for (int8_t i = 7; i >= 0; i--) {
    if (code & (1 << i)) {
      // Send '1' bit: pulse for 15us ON, 15us OFF
      emit_ir_pulse(60); // 1 cycle = 30us total (15us on + 15us off)
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
      // emit_ir_pulse(60); // 1 cycle = 30us total (15us on + 15us off)
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

/**
 * Custom microsecond delay function for audio generation
 * Calibrated for 8MHz clock
 * @param us Microseconds to delay
 */
void delay_us_custom(uint16_t us) {
    while (us--) {
        // Adjust loop count for 1μs per iteration at 8MHz
        for (uint8_t i = 0; i < 3; i++) {
            asm volatile("nop");
        }
    }
}

/**
 * Executes chirp sequence
 */
void chirp(void) {
    uint16_t delay_val = CHIRP_BASE_DELAY;
    for (uint8_t chirp = 0; chirp < CHIRP_STEPS; chirp++) {
        for (uint8_t i = 0; i < CHIRP_CYCLES_PER_STEP; i++) {
            PORTB |= (1 << BUZZER);
            delay_us_custom(delay_val);
            PORTB &= ~(1 << BUZZER);
            delay_us_custom(delay_val);
        }
        delay_val -= CHIRP_DELAY_DECREMENT;
        if (delay_val < CHIRP_MIN_DELAY) {
            delay_val = CHIRP_MIN_DELAY;
        }
        _delay_ms(CHIRP_PAUSE_MS);
    }
}

void half_chirp() {
    strip.setPixelColor(0, strip.Color(0, 123, 0)); // green only
    strip.show();
    delay_us_custom(50);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
    for (uint8_t i = 0; i < 5; i++) {
        PORTB |= (1 << BUZZER);
        delay_us_custom(100);
        PORTB &= ~(1 << BUZZER);
        delay_us_custom(100);
    }
}

void set_fade_color(uint16_t phase, uint32_t diff) {
    uint8_t i = phase_index;  // Current gradient index

    uint8_t r = GRADIENT_START_R[i] + ((uint16_t)(GRADIENT_END_R[i] - GRADIENT_START_R[i]) * phase) / PHASE_MAX;
    uint8_t g = GRADIENT_START_G[i] + ((uint16_t)(GRADIENT_END_G[i] - GRADIENT_START_G[i]) * phase) / PHASE_MAX;
    uint8_t b = GRADIENT_START_B[i] + ((uint16_t)(GRADIENT_END_B[i] - GRADIENT_START_B[i]) * phase) / PHASE_MAX;

    strip.setPixelColor(0, strip.Color(g, r, b));  // GRB order
    strip.show();
}

void chaotic_spiral_disrupt(uint8_t steps, uint16_t step_delay_ms) {
    uint16_t base_delay = 100;  // start around 5kHz
    uint16_t max_delay = 20;    // end around 25kHz

    for (uint8_t i = 0; i <= steps; i++) {
        // Interpolate up in pitch (down in delay)
        uint16_t pitch_delay = base_delay - ((base_delay - max_delay) * i / steps);
        pitch_delay += random(-10, 10);  // add chaos jitter

        if (pitch_delay < 10) pitch_delay = 10;

        // Random number of buzz pulses per step (like "shards" of sound)
        uint8_t bursts = random(3, 7);

        for (uint8_t b = 0; b < bursts; b++) {
            PORTB |= (1 << BUZZER);
            delay_us_custom(pitch_delay);
            PORTB &= ~(1 << BUZZER);
            delay_us_custom(pitch_delay + random(10, 60));
        }

        // Growing silence = fade-out effect
        _delay_ms(step_delay_ms + i / 4);
    }
}




/**
 * Main program loop
 * Implements Kuramoto-style phase synchronization
 */
int main(void) {
    setup_hardware();
    
    uint16_t phase = 0;
    uint8_t last_rx_state = (PINB & (1 << IR_RX));
    uint8_t refractory = 0;
    bool half_chirped = false;

    while (1) {
      // _delay_ms(TICK_DELAY_MS);
      uint32_t time = my_micros();

      // if (red_timer != 0 && time - red_timer < 10000000) {
      //   red = true;
      // } else {
      //   red = false;
      //   red_timer = 0;
      //   code_num = 0;
      // }
      read_code();

      if (code_num == 5) {
       // Freeze phase color at the moment of reception
        uint8_t i = phase_index;  // use current gradient set

        uint8_t target_r = GRADIENT_START_R[i] + ((uint16_t)(GRADIENT_END_R[i] - GRADIENT_START_R[i]) * phase) / PHASE_MAX;
        uint8_t target_g = GRADIENT_START_G[i] + ((uint16_t)(GRADIENT_END_G[i] - GRADIENT_START_G[i]) * phase) / PHASE_MAX;
        uint8_t target_b = GRADIENT_START_B[i] + ((uint16_t)(GRADIENT_END_B[i] - GRADIENT_START_B[i]) * phase) / PHASE_MAX;


        // Fade from red to frozen target color
        const uint8_t steps = 100;
        const uint16_t delay_ms = 10;

        for (uint8_t i = 0; i <= steps; i++) {
            uint16_t blend = ((uint16_t)i * 255) / steps;
            uint16_t inv_blend = 255 - blend;

            uint8_t r = (uint8_t)((255 * inv_blend + target_r * blend) / 255);
            uint8_t g = (uint8_t)((0   * inv_blend + target_g * blend) / 255);
            uint8_t b = (uint8_t)((0   * inv_blend + target_b * blend) / 255);

            strip.setPixelColor(0, strip.Color(g, r, b));  // GRB order
            strip.show();
            chaotic_spiral_disrupt(1, delay_ms);
        }

        strip.setPixelColor(0, strip.Color(target_g, target_r, target_b));
        strip.show();

        code_num = 0;
        code_received = false;
        red = false;
        red_timer = 0;

        // Delay random phase jump *after* color is stable
        phase += random(0, 255);
      }


      if (time - timer > 2000) {
        timer = time;
          
        set_fade_color(phase, (time - red_timer));

        tick_counter++;
        if (tick_counter >= TICKS_PER_SWITCH) {
            tick_counter = 0;
            phase_index = (phase_index + 1) % PHASE_SEQUENCE_LEN;
            PHASE_MAX = PHASE_SEQUENCE[phase_index];
            phase = 0;
            half_chirped = false;
        }

        // Advance phase
        phase += PHASE_STEP;
        if (phase > PHASE_MAX) {
            phase = PHASE_MAX;
        }

        if ((phase >= (PHASE_MAX / 2)) && !half_chirped) {
            half_chirp();
            half_chirped = true;
        }

        if (phase >= PHASE_MAX) {
            emit_ir_pulse(200);
            chirp();
            phase = 0;
            half_chirped = false;
            refractory = REFRACTORY_FLASH;
        }

        uint8_t current_rx = (PINB & (1 << IR_RX));

        if (last_rx_state && !current_rx && refractory == 0) {
            if (phase > (PHASE_MAX / 4)) {
                uint16_t delta = ((uint32_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
                phase += delta;
                if (phase > PHASE_MAX) {
                    phase = PHASE_MAX;
                }
                if (PHASE_MAX - phase < JUMP_TO_FLASH_MARGIN) {
                    emit_ir_pulse(200);
                    chirp();
                    phase = 0;
                    half_chirped = false;
                    refractory = REFRACTORY_FLASH;
                } else {
                    refractory = REFRACTORY_TRIGGER;
                }
            }
        }

        if (refractory > 0) {
            refractory--;
        }

        last_rx_state = current_rx;
      }
    }

    return 0;
}
