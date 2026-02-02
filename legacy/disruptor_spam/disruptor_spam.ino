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
#define PHASE_MAX               17
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

// ===== FUNCTION PROTOTYPES =====
void setup_hardware(void);
void emit_ir_pulse(uint16_t cycles);
void delay_us_custom(uint16_t us);
void chirp(void);
void half_chirp(void);
uint8_t get_random_color_component(void);

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

    
    // Generate ascending chirp
    uint16_t delay_val = CHIRP_BASE_DELAY;
    
    for (uint8_t chirp = 0; chirp < CHIRP_STEPS; chirp++) {
        // Generate tone at current frequency
        for (uint8_t i = 0; i < CHIRP_CYCLES_PER_STEP; i++) {
            PORTB |= (1 << BUZZER);
            delay_us_custom(delay_val);
            PORTB &= ~(1 << BUZZER);
            delay_us_custom(delay_val);
        }
        
        // Increase pitch (decrease delay)
        delay_val -= CHIRP_DELAY_DECREMENT;
        if (delay_val < CHIRP_MIN_DELAY) {
            delay_val = CHIRP_MIN_DELAY;
        }
        
        // Pause between chirp steps
        _delay_ms(CHIRP_PAUSE_MS);
    }
    
}

void half_chirp() {

    strip.setPixelColor(0, strip.Color(0, 123, 0)); //red only
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

void set_fade_color(uint8_t phase) {
    // Phase should go from 0 to PHASE_MAX
    uint8_t g = 255 - ((uint16_t)phase * 255 / PHASE_MAX);  // Green: 255 → 0
    uint8_t b = (uint16_t)phase * 255 / PHASE_MAX;          // Blue:  0   → 255

    strip.setPixelColor(0, strip.Color(g, 0, b));  // GRB order!
    strip.show();
}


/**
 * Main program loop
 * Implements Kuramoto-style phase synchronization
 */
int main(void) {
    setup_hardware();
    
    // State variables
    uint8_t phase = 0;
    uint8_t last_rx_state = (PINB & (1 << IR_RX));
    uint8_t refractory = 0;
    bool half_chirped = false;

    while (1) {
        set_fade_color(phase);
        _delay_ms(TICK_DELAY_MS);
        
        // Advance phase
        phase += PHASE_STEP;
        if (phase > PHASE_MAX) {
            phase = PHASE_MAX;
        }

        if ((phase >= (PHASE_MAX / 2)) && !half_chirped) {
            //half_chirp();
            half_chirped = true;
        }
        
        // Check for autonomous flash
        if (phase >= PHASE_MAX) {
            emit_ir_pulse(200);
            chirp();
            phase = 0;
            half_chirped = false;
            refractory = REFRACTORY_FLASH;
        }
        
        // Check for IR trigger from other fireflies
        uint8_t current_rx = (PINB & (1 << IR_RX));


        
        // Detect falling edge (IR signal received)
        if (last_rx_state && !current_rx && refractory == 0) {
            // Only respond if we're past the initial phase
            if (phase > (PHASE_MAX / 4)) {
                // Calculate phase adjustment (Kuramoto coupling)
                //uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
                //phase += delta;
                
                if (phase > PHASE_MAX) {
                    phase = PHASE_MAX;
                }
                
                // Check if close enough to flash immediately
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
        
        // Decrement refractory period
        if (refractory > 0) {
            refractory--;
        }
        
        last_rx_state = current_rx;
    }
    
    return 0; // Never reached
}