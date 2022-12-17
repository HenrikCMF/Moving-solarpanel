/*
 exam number: 493947
 */ 
 
 #define F_CPU 16000000UL
#define ICRval 250
#define ADC_POWER_CONVERTER 0 
#define ADC_PR_LEFT 1
#define ADC_PR_RIGHT 2
#define ADC_SOLARPANEL 3
 #include <stdio.h>
 #include <stdbool.h>
// #include "lcd.h"
 #include <avr/io.h>
 #include <util/delay.h>
 #include <stdlib.h>
 #include <avr/interrupt.h>
//#include "i2cmaster.h"
#include "usart.h"
#include "PID.h"
//#include <avr/sleep.h>
volatile float errorcorrection;
volatile int VIN_READY=0, VOUT_READY=0, PR_LEFT_READY=0, PR_RIGHT_READY=0;
volatile float VIN, VOUT, PR_LEFT, PR_RIGHT;
void pwm1setup(void)
{
	//Setting up PWM on timer 1
	TCCR1A|=(1<<COM1A1)|(1<<WGM11);  //non inverting mode
	ICR1=ICRval;//define top value to count to, giving a frequency of 504Hz
	OCR1A=0; //starting with a duty cycle of 50%
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11); //fast PWM ICR prescaler=256
	DDRB=0xFF;
}
void pwm0setup(void) //setup pwm signal for motor
{
	TCCR0A|=(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
	TCCR0B|=(1<<WGM02)|(1<<CS01)|(1<<CS00);
	OCR0A=255;
	OCR0B=150;
}
void timer2(unsigned int  miliseconds) //function for waiting x amount of miliseconds
{ 
	for (int i = 0; i < miliseconds; i++)
	{
		TCCR2A|=(1 << WGM21);   //timer in the CTC mode
		TCCR2B|=(1 << CS22); //prescaler 64
		OCR2A=249;            //top value 249
		while((TIFR2 & (1 << OCF2A)) == 0) // flag register waiting for overflow
		{
		}
		TIFR2 = (1 << OCF2A); // reset the flag register
	}
}
void sweep(float PR_LEFT_ST, float PR_RIGHT_ST)
{
	unsigned int miliseconds;
	float diff;
	diff=abs(PR_LEFT_ST - PR_RIGHT_ST);
	if (diff <= 0.01) //if they are almost the same, stop motor
	{	
		PORTD &= ~(1<<PORTD6);
		PORTD &= ~(1<<PORTD7);
	}
	else
	{
		miliseconds=diff*100; //miliseconds of rotation

		if(PR_LEFT_ST > PR_RIGHT_ST) //if there is more light on the right side, configure motor to go towards the right
		{ 
			
			PORTD |= (1<<PORTD6);
			
			PORTD &= ~(1<<PORTD7);
			timer2(miliseconds);
		}
		else //else go towards the left
		{
			PORTD |= (1<<PORTD7);
			
			PORTD &= ~(1<<PORTD6);
			timer2(miliseconds);
		}
	}
}

void adcsetup(void)
{
	ADMUX|=(1<<REFS0); //ref voltage for adc (AVCC with external capacitor at AREF pin)
	ADCSRA=(1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN)| (1<<ADIE); //ADEN=ADC Enable ADPS0 & ADPS1 & ADPS2 = division factor 128 ADIE enables interrupt
	DDRC=0xF0; //make all PINC inputs
	PORTC=0x3F;
	DDRD = 0xFF; //I/O board:PD4…7 as outputs, for LEDs
	ADMUX &= 0xf0;
	sei();
	ADCSRA |= (1<<ADSC);
}

void climb(float top) //slowly increment duty cycle of buck-boost to avoid surge current
{
	for (int i=OCR1A; i<top; i++)
	{
		OCR1A++;
		_delay_ms(1);
	}
}

void dutycycle_correction(void) //for electronics buck boost converter
{
	if((OCR1A+round(errorcorrection))>OCR1A_MAX)
	{
		climb(OCR1A_MAX);
	}
	else if((OCR1A+round(errorcorrection))<OCR1A_MIN)
	{
		OCR1A=OCR1A_MIN;
	}
	if (errorcorrection+OCR1A<OCR1A)
	{
		OCR1A=errorcorrection+OCR1A;
	}
	else climb(errorcorrection+OCR1A);
}

int main (void)
{
	DDRB=0xFF;
	PORTB=0xFF;
	pwm1setup();
	
	for (int i=0; i<100; i++) //slow start of buckboost
	{
		OCR1A++;
		_delay_ms(10);
	}
	pwm0setup();
	adcsetup();
	while(1)
	{		
		if(VOUT_READY) //if a new buckboost ADC measurement is ready
		{
			errorcorrection=PID_calculate(VOUT*8.5, 13.8,0.001);
			dutycycle_correction();
			VOUT_READY=0;
		}
		if (PR_LEFT_READY && PR_RIGHT_READY) //else if a new photo resistor measurement is ready, use motor
		{
			sweep(PR_LEFT,PR_RIGHT);

			PR_LEFT_READY=0;
			PR_RIGHT_READY=0;
		}
	}
}

ISR(ADC_vect) //ADC interrupt
{
	volatile static char adc_read_adress=0;
	uint16_t latadc=ADC;
	static int count=0;
	static float average=0;
	float voltage=(latadc*4.95/1024.0); //convert ADC value to voltage
	count++;
	average+=voltage;
	if (count==8) //average 8 readings to avoid 1-offs
	{
		average/=8;
		switch(adc_read_adress) //based on the current reading, load it to the correct variable
		{
			case 0:
			VOUT_READY=1;
			VOUT=average;
			break;
			case 1:
			PR_LEFT_READY=1;
			PR_LEFT=average;
			break;
			case 2:
			PR_RIGHT_READY=1;
			PR_RIGHT=average;
			break;
			case 3:
			VIN_READY=1;
			VIN=average;
			adc_read_adress=-1;
			break;
		} // then increment variable so all addresses between 0 and 3 are read
		adc_read_adress++;
		count=0;
		average=0;
	}
	ADMUX &= 0xf0;
	ADMUX |= adc_read_adress;
	ADCSRA |= (1<<ADSC); //read again
	
}
