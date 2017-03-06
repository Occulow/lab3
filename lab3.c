/*!
 * **********************************************************************************************
 * 
 * Copyright information for source of code for Ultrasonic Range Finder: HC-SR04
 * \author      :   Praveen Kumar
 * \date        :   Mar 24, 2014
 * Copyright(c)         :   Praveen Kumar - www.veerobot.com
 *
 * **********************************************************************************************
 */

#include <inttypes.h>
#include <avr/io.h>
// Hint: See https://github.com/vancegroup-mirrors/avr-libc/blob/master/avr-libc/include/avr/iom328p.h for pinouts
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>

// For delay functions
#define F_CPU 8000000UL
// Hotfix for getting delay to work
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

// For UART
#define BAUD 115200
#include <util/setbaud.h>

#define DELAY_OFFSET 11

#include "sonar.h"
 
volatile uint32_t overFlowCounter = 0;
volatile uint32_t trig_counter = 0;
volatile uint32_t no_of_ticks = 0;

void beep(unsigned long hz, unsigned long ms);

/* BEGIN: UART code from lab2 */
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
/* END */

/**
 * Initialize speaker
 */
void setup_speaker() {
	// TODO: Make this more portable
	// ie. SET_OUTPUT(<port>, <offset>)
	// Set data rate (of Port B[2]) to OUTPUT
	DDRB |= (1<<PORTB2);
	// Set initial value of PortB[2]
	PORTB |= (1<<PORTB2);
}

/**
 * Plays a tone at the given frequency for the given duration (in ms)
 */
void beep(unsigned long hz, unsigned long ms) {
	int i;
	// reference: 440hz = 2273 usec/cycle, 1136 usec/half-cycle
	unsigned long us = (500000 / hz) - DELAY_OFFSET;
	unsigned long rep = (ms * 500L) / (us + DELAY_OFFSET);
	for (i = 0; i < rep; i++) {
		// TODO: Make these portable, like SET_PIN or DIGITAL_WRITE
		// On
		PORTB |= (1<<PORTB2);
		_delay_us(us);
		// Off
		PORTB &= ~(1<<PORTB2);
		_delay_us(us);
	}
}
 
/********** ...- . . .-. --- -... --- - *********************************
 * Initiate Ultrasonic Module Ports and Pins
 * Input:   none
 * Returns: none
*********** ...- . . .-. --- -... --- - *********************************/
void init_sonar(){
    TRIG_OUTPUT_MODE();     // Set Trigger pin as output
    ECHO_INPUT_MODE();      // Set Echo pin as input
}
 
// /********** ...- . . .-. --- -... --- - *********************************
//  * Send 10us pulse on Sonar Trigger pin
//  * 1.   Clear trigger pin before sending a pulse
//  * 2.   Send high pulse to trigger pin for 10us
//  * 3.   Clear trigger pin to pull it trigger pin low
//  *  Input:   none
//  *  Returns: none
// ********** ...- . . .-. --- -... --- - *********************************/
void trigger_sonar(){
    TRIG_LOW();             // Clear pin before setting it high
    _delay_us(1);           // Clear to zero and give time for electronics to set
    TRIG_HIGH();            // Set pin high
    _delay_us(12);          // Send high pulse for minimum 10us
    TRIG_LOW();             // Clear pin
    _delay_us(1);           // Delay not required, but just in case...
}
 
// /********** ...- . . .-. --- -... --- - *********************************
//  * Increment timer on each overflow
//  * Input:   none
//  * Returns: none
// ********** ...- . . .-. --- -... --- - *********************************/
ISR(TIMER1_OVF_vect){   // Timer1 overflow interrupt
    overFlowCounter++;
    TCNT1=0;
}
 
// ********* ...- . . .-. --- -... --- - *********************************
//  * Calculate and store echo time and return distance
//  * Input:   none
//  * Returns: 1. -1       :   Indicates trigger error. Could not pull trigger high
//  *          2. -2       :   Indicates echo error. No echo received within range
//  *          3. Distance :   Sonar calculated distance in cm.
// ********** ...- . . .-. --- -... --- - ********************************
unsigned int read_sonar(){
    int dist_in_cm = 0;
    init_sonar();                       // Setup pins and ports
    trigger_sonar();                    // send a 10us high pulse
 
    while(!(ECHO_PIN & (1<<ECHO_BIT))){   // while echo pin is still low
        trig_counter++;
         uint32_t max_response_time = SONAR_TIMEOUT;
        if (trig_counter > max_response_time){   // SONAR_TIMEOUT
            return TRIG_ERROR;
        }
    }
 
    TCNT1=0;                            // reset timer
    TCCR1B |= (1<<CS10);              // start 16 bit timer with no prescaler
    TIMSK1 |= (1<<TOIE1);             // enable overflow interrupt on timer1
    overFlowCounter=0;                  // reset overflow counter
    sei();                              // enable global interrupts
 
    while((ECHO_PIN & (1<<ECHO_BIT))){    // while echo pin is still high
        if (((overFlowCounter*TIMER_MAX)+TCNT1) > SONAR_TIMEOUT){
            return ECHO_ERROR;          // No echo within sonar range
        }
    };
 
    TCCR1B = 0x00;                      // stop 16 bit timer with no prescaler
    cli();                              // disable global interrupts
    no_of_ticks = ((overFlowCounter*TIMER_MAX)+TCNT1);  // counter count
    dist_in_cm = (no_of_ticks/(CONVERT_TO_CM*CYCLES_PER_US));   // distance in cm
    return (dist_in_cm);
}

int main(void)
{
	uart_init();
	stdout = &uart_output;
	stdin  = &uart_input;

	char input;
	setup_speaker();
	int distance;

	// Setup ports
	DDRB |= (1<<1) | (1<<0);
	PORTB |= (1<<0);
	PORTB &= ~(1<<1);

	/* Print hello and then echo serial
	** port data while blinking LED */
	printf("Hello world!\r\n");

	while(1){
		distance = read_sonar();

		/* Sensor data over UART */
		if(distance > 0)
			printf("Distance = %d\r\n",distance);

		/* Actuator over UART */
		// input = getchar();
		// if(input == 's')
		// 	beep(440,1000);

		/* Sensor Liked to Actuator */
		beep(10000/distance,500);

	}

}