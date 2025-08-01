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

// ===== GLOBAL VARIABLES =====
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== Dynamic Phase Configuration =====
const uint16_t PHASE_SEQUENCE[] = {255, 500, 70, 350, 140};
#define PHASE_SEQUENCE_LEN (sizeof(PHASE_SEQUENCE) / sizeof(PHASE_SEQUENCE[0]))
uint8_t phase_index = 0;
uint16_t PHASE_MAX = 255;

uint16_t tick_counter = 0;
#define SWITCH_INTERVAL_MS (240000) // 4 minutes
#define TICKS_PER_SWITCH (SWITCH_INTERVAL_MS / TICK_DELAY_MS)

// ===== FUNCTION PROTOTYPES =====
void setup_hardware(void);
void emit_ir_pulse(uint16_t cycles);
void delay_us_custom(uint16_t us);
void chirp(void);
void half_chirp(void);
void set_fade_color(uint16_t phase);

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
 * Creates RGB flash with ascending pitch chirp
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

void set_fade_color(uint16_t phase) {
    uint8_t g = 255 - ((uint32_t)phase * 255 / PHASE_MAX);
    uint8_t b = ((uint32_t)phase * 255 / PHASE_MAX);
    strip.setPixelColor(0, strip.Color(g, 0, b));  // GRB order
    strip.show();
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
        set_fade_color(phase);
        _delay_ms(TICK_DELAY_MS);
        
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

    return 0;
}
