#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

// Setup Timer0 for phase-correct PWM at 33kHz on OC0B (PB1)
void setup() {
  // Set PB1 (OC0B / pin 6) as output for IR emitter
  DDRB |= (1 << PB1);

  // Disconnect OC0B initially, configure Phase Correct PWM mode (WGM02:0 = 0b101)
  // TCCR0A = (1 << WGM00);                  // Phase Correct PWM (part 1)
  // TCCR0B = (1 << WGM02) | (1 << CS00);    // Phase Correct PWM (part 2), no prescaler

  // // Set frequency: OCR0A = F_CPU / (2 * f)
  // OCR0A = 15;//F_CPU / 33000UL;          // ≈ 121 for ~33kHz
  // OCR0B = OCR0A / 2;                      // 50% duty cycle

  // PB0 (pin 5) as input for TSOP receiver
  DDRB &= ~(1 << PB0);
  PORTB |= (1 << PB0);  // enable pull-up

  // PB2 (pin 7) as output for visible LED
  DDRB |= (1 << PB2);
  PORTB &= ~(1 << PB2);  // initially off
}

// void emit_pulse() {
//   TCCR0A |= (1 << COM0B1);   // Enable PWM on PB1
//   _delay_ms(10);             // Emit for 10ms
//   TCCR0A &= ~(1 << COM0B1);  // Disable PWM
//   PORTB &= ~(1 << PB1);      // Ensure pin is low
// }
void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << PB1);
    _delay_us(15);   // High for ~15us (33kHz)
    PORTB &= ~(1 << PB1);
    _delay_us(15);   // Low for ~15us
  }
}

void flash_led() {
  PORTB |= (1 << PB2);
  _delay_ms(10);
  PORTB &= ~(1 << PB2);
}

int main() {
  setup();

  while (1) {
    emit_pulse(200);  // Transmit burst
    //_delay_ms(30); // Ignore self-detection window

    // Wait for TSOP signal (active LOW)
    uint8_t prev = (PINB & (1 << PB0));
    uint16_t timeout = 0;

    while (timeout < 5000) { // check for 500ms (100us steps)
      uint8_t current = (PINB & (1 << PB0));

      if (prev && !current) {  // Falling edge
        flash_led();
        //break;
      }

      prev = current;
      _delay_us(100);
      timeout++;
    }

    //_delay_ms(3000); // Wait before next emission cycle
  }
}
