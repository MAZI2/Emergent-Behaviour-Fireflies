#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
  // Set PB0 (pin 5) as output
  DDRB |= (1 << PB0);

  while (1) {
    // Turn LED on
    PORTB |= (1 << PB0);
    _delay_ms(500);

    // Turn LED off
    PORTB &= ~(1 << PB0);
    _delay_ms(500);
  }
}
