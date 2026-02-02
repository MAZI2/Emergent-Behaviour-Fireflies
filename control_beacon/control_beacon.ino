#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

// Pins
#define IR_TX PB1

// -------- Setup --------
void setup() {
    // IR emitter as output
    DDRB |= (1 << IR_TX);
    PORTB &= ~(1 << IR_TX);
}

// -------- IR Carrier Burst --------
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

// -------- Frame Sender --------
void send_code(uint8_t value) {

    // Start burst
    mark_us(6000);
    space_us(3000);

    // 8 bits MSB first
    for (int8_t i = 7; i >= 0; i--) {

        if (value & (1 << i)) {
            mark_us(2000);   // logical 1
            space_us(2000);
        } else {
            mark_us(1000);   // logical 0
            space_us(3000);
        }
    }

    // End gap
    space_us(6000);
}

// -------- Main --------
int main(void) {

    init();
    setup();

    while (1) {

        // ---- TURN ON for 10 seconds ----
        uint32_t start = millis();
        while (millis() - start < 10000UL) {
            send_code(CODE_TURN_ON);
            delay(20);
        }

        // ---- Wait 1 minute ----
        uint32_t waitStart = millis();
        while (millis() - waitStart < 60000UL) {
            delay(100);
        }

        // ---- TURN OFF for 10 seconds ----
        start = millis();
        while (millis() - start < 10000UL) {
            send_code(CODE_TURN_OFF);
            delay(20);
        }
    }
}
