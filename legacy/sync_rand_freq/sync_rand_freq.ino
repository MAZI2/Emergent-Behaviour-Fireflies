#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel setup
#define NEOPIXEL_PIN PB2
#define NUM_PIXELS 1
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Behavior tuning
#define PHASE_MAX 255
#define EPSILON 64
#define PHASE_STEP 1
#define TICK_DELAY_MS 20  // Increased for stability
#define JUMP_TO_FLASH_MARGIN 16

// Pins
#define IR_TX PB1
#define IR_RX PB0
#define LED PB2
#define BUZZER PB3

// Custom delay that works with variables
void delay_us(uint16_t us) {
    while (us--) {
        _delay_us(1);
    }
}

void setup() {
    // Initialize ports
    DDRB |= (1 << IR_TX) | (1 << LED) | (1 << BUZZER);
    DDRB &= ~(1 << IR_RX);
    PORTB |= (1 << IR_RX);
    PORTB &= ~((1 << LED) | (1 << BUZZER));
    
    // Initialize NeoPixel
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    
    // Disable interrupts during NeoPixel access
    cli();
}

void emit_pulse(uint16_t cycles) {
    for (uint16_t i = 0; i < cycles; i++) {
        PORTB |= (1 << IR_TX);
        delay_us(15);
        PORTB &= ~(1 << IR_TX);
        delay_us(15);
    }
}

void flash_led_rgb() {
    // Re-enable interrupts temporarily for random()
    sei();
    uint8_t r = rand() % 256;
    uint8_t g = rand() % 256;
    uint8_t b = rand() % 256;
    cli();
    
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    delay_us(100000); // 100ms delay
    strip.setPixelColor(0, 0);
    strip.show();
}

void play_tone(uint16_t period_us, uint16_t duration_ms) {
    uint16_t cycles = (duration_ms * 1000UL) / period_us;
    for (uint16_t i = 0; i < cycles; i++) {
        PORTB |= (1 << BUZZER);
        delay_us(period_us/2);
        PORTB &= ~(1 << BUZZER);
        delay_us(period_us/2);
    }
}

int main(void) {
    setup();
    
    uint8_t phase = 0;
    uint8_t last_rx_state = (PINB & (1 << IR_RX));
    uint8_t refractory = 0;
    uint16_t tone_counter = 0;

    while (1) {
        // Calculate frequency based on phase (400Hz-1600Hz)
        uint16_t period = 1250 - ((uint16_t)850 * phase) / PHASE_MAX;
        
        // Play tone for shorter duration to maintain responsiveness
        play_tone(period, 5); // Play for 5ms
        
        // Only update phase every few iterations
        if (++tone_counter >= 4) {
            tone_counter = 0;
            
            // Phase management
            phase += PHASE_STEP;
            if (phase >= PHASE_MAX) {
                phase = 0;
                emit_pulse(200);
                flash_led_rgb();
                refractory = 20;
            }

            // IR detection
            uint8_t current_rx = (PINB & (1 << IR_RX));
            if (last_rx_state && !current_rx && !refractory) {
                if (phase > (PHASE_MAX / 4)) {
                    uint8_t delta = ((uint16_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
                    phase += delta;
                    if (phase > PHASE_MAX) {
                        phase = 0;
                        emit_pulse(200);
                        flash_led_rgb();
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
}