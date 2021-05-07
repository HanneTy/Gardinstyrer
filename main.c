/*
 * prosjekt.c
 *
 * Created: 03.05.2021 09:55:00
 * Author : Hanne
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"

volatile char flag = 0;

/* Funksjoner */

static inline void initPOT(void) {							//Funksjon for potmeter
	
	ADMUX = (1<<REFS0) | (0x01);							//ADC1 (PC1)
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//Enabler ADC og prescaler
}

static inline void initLGT(void) {							//Funksjon for lyssensor
	
	ADMUX = (1<<REFS0) | (0x00);							//ADC0 (PC0)
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//Enabler ADC og prescaler
}

static inline void Timer1PWM(void){							//Funksjon for servo ved bruk av timer1
	
	TCCR1A |= (1 << COM1A1) | (1 << WGM11);					// Output på PB1/OC1A
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);	//Setter prescaling, teller i ms, setter fast PWM og ICR1A til top som oppdateres på bunn, flag settes på topp
	ICR1 = 40000;											//Toppverdi på 20ms
	DDRB |= (1 << PINB1);									//Output PB1 for servo
}

static inline void initInterrupt(void) {					//Funksjon til interrupt
	/* KNAPP */
	EIMSK |= (1<<INT0);										//enable int0
	EICRA |= (1<<ISC01)|(1<<ISC00);							//Setting int0 on rising edge
	PORTD |= (1<<PD2);										//Pullup på PD2
	
}

/* Interrupt rutine */

ISR(INT0_vect){
	
	printString("Du gjør et avbrudd! \n");
	_delay_ms(200);											
	if(flag == 0){											//Sjekk verdi på flag
		flag = 1;
	}else if(flag == 1){
		flag = 0;
	}
	_delay_ms(50);											//delay to remove debounce	
}

/* Main løkke */

int main(void){
	Timer1PWM();											//Kaller på Servofunksjonen
	initUSART();											//Kaller på USART
	initInterrupt();										//Kaller interrupt
											
	uint16_t adcPOT;
	uint16_t adcLGT;
	
	sei();													//"Tillater" interrupt

	while(1){
		if (flag == 0){
			initPOT();											//Kaller på POT
			ADCSRA |= (1<<ADSC);
			loop_until_bit_is_clear(ADCSRA, ADSC);
			adcPOT = ADC;										//Henter verdi fra POT
		
			initLGT();											//Kaller på LYSSENSOR
			ADCSRA |= (1<<ADSC);
			loop_until_bit_is_clear(ADCSRA, ADSC);				
			adcLGT = ADC;										//Henter verdi fra LYSSENSOR
			
		
			if (adcLGT > adcPOT){
				OCR1A = 3500;
				printString("lys høyere enn pot \n");
				_delay_ms(50);
			
		
			}else if (adcLGT < adcPOT){
				OCR1A = 1500;
				printString("lys lavere enn pot \n");
				_delay_ms(50);
		
			}else{
				OCR1A = 2500;
				printString("lik \n");
				_delay_ms(50);
				
			}
			_delay_ms(200);
		}else{
			printString("Nå styrer du manuelt, trykk igjen for automatisk styring \n");
			_delay_ms(50);
		}		
	}
}


			