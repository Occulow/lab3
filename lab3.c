

/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>

#define F_CPU 8000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>
#define BAUD 115200
#include <util/setbaud.h>

#define DELAY_OFFSET 11
// declare it for good measure
void beep(unsigned long hz, unsigned long ms);

void setup_speaker() {
    DDRB |= (1<<PORTB2);
    PORTB |= (1<<PORTB2);
}
void beep(unsigned long hz, unsigned long ms) {
    int i;
    // reference: 440hz = 2273 usec/cycle, 1136 usec/half-cycle
    unsigned long us = (500000 / hz) - DELAY_OFFSET;
    unsigned long rep = (ms * 500L) / (us + DELAY_OFFSET);
    for (i = 0; i < rep; i++) {
        // On
        PORTB |= (1<<PORTB2);
        _delay_us(us);
        // Off
        PORTB &= ~(1<<PORTB2);
        _delay_us(us);
    }
}

void uart_init(void) {
   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;

#if USE_2X
   UCSR0A |= _BV(U2X0);
#else
   UCSR0A &= ~(_BV(U2X0));
#endif

   UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
   UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
    UDR0 = c;
}

char uart_getchar(void) {
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


int main(void)
{
    /* Setup serial port */
    uart_init();
    setup_speaker();
    stdout = &uart_output;
    stdin  = &uart_input;

    while(1) {
        beep(440, 1000);
    }

}
