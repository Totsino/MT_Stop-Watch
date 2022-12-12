/*
 * mini.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Youssef
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>



unsigned char n=0;
unsigned char hours=0;
unsigned char mins=0;
unsigned char sec=0;


void int_init(){

	DDRD &= ~(1<<2); //PORT D2 INT0 INPUT
	DDRD &= ~(1<<3); //PORT D3 INT1 INPUT
	DDRB &= ~(1<<2); //PORT B2 INT2 INPUT

	PORTD |= (1<<2); //PULL UP RESISTOR ENABLE
	PORTB |= (1<<2); //PULL UP RESISTOR ENABLE

	SREG = (1<<7);
	GICR = (1<<INT0) | (1<<INT1) | (1<<INT2); //MODULE ENABLE FOR INT 0 , 1 , 2
	MCUCR = (1<<ISC11) | (1<<ISC10) | (ISC01);// FALLING EDGE FOR INT0 AND 2 , RISING FOR INT1
	MCUCSR &= ~(1<<ISC2);
}

ISR(INT0_vect){
	sec=0;
	mins=0;
	hours=0;
}

ISR(INT1_vect){

	TCCR1B = 0;
}

ISR(INT2_vect){

	TCCR1B |= (1<<CS10) | (1<<CS12)| (1<<WGM12);
}


void timer1_init(){
	TCCR1B |= (1<<CS10) | (1<<CS12) | (1<<WGM12);  //prescaler FCPU/1024 AND compare mode on;

	SREG |= (1<<7); //GLOBAL INTERUBT ENBALE

	TCNT1L = 0;//initially from 0;
	TCNT1H = 0;

	OCR1A = 0x03E8;// to make 1000 ms in both lines


	TCCR1A |= (1<<FOC1A); //compare for A and B


	TIMSK |= (1<<OCIE1A); //INTERRUPT MODULE ENABLE


}

ISR(TIMER1_COMPA_vect){ //for Counting


	sec++;
	if(sec == 60){
		mins++;
		sec=0;
	}

	if (mins == 60){
		hours++;
		mins=0;
	}
	if (hours>23){
		hours=0;
	}



}




int main(){

	int_init();
	timer1_init();

	DDRC |= 0x0F; //MAKE PINS OUTPUT
	PORTC &= ~(0x0F); //INITIALLY ZERO
	DDRA |= 0x3F; //FIRST 6 PINS IN A ARE OUTPUTS
	PORTA |= (0x3F); //MAKE THEM OFF AT BEGINNING


	while(1){

		PORTA =(1<<0);
		PORTC = sec % 10;
		_delay_us(5);
		PORTA =(1<<1);
		PORTC = sec / 10;
		_delay_us(5);
		PORTA = (1<<2);
		PORTC = mins % 10;
		_delay_us(5);
		PORTA = (1<<3);
		PORTC = mins / 10;
		_delay_us(5);
		PORTA = (1<<4);
		PORTC = hours % 10;
		_delay_us(5);
		PORTA = (1<<5);
		PORTC = hours / 10;
		_delay_us(5);


	}
}
