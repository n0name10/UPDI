/*
* IO.h
*
* Created: 2022-07-02 06:31:38
*  Author: Adamko
*/


#ifndef UPDI_IO_H_
#define UPDI_IO_H_
#include <stdio.h>
#include <avr/io.h>

void __attribute__((always_inline)) inline UPDI_Set(uint8_t ch){
	if(ch == 0) SET_C2CK;
	else if(ch == 1) SET_C2CK;
}

void __attribute__((always_inline)) inline UPDI_Reset(uint8_t ch){
	if(ch == 0) RESET_C2CK;
	else if(ch == 1) RESET_C2CK;
}

uint8_t __attribute__((always_inline)) inline UPDI_Read(uint8_t ch){
	if(ch == 0) return (PINB & (1<<2));
	return (PINB & (1<<2));
}


void __attribute__((always_inline)) inline UPDI_IO(uint8_t ch, uint8_t io){
	if(ch == 0){
		if(io)C2CK_OUT;
		else C2CK_IN;
	}else if(ch == 1)
	{
		if(io)C2CK_OUT;
		else C2CK_IN;
	}
}

void inline __attribute__((always_inline)) UPDI_UartInit(uint8_t baud){
	DDRD &= ~(1 << PD2);
	DDRD |= (1 << PD3);
	PORTD |= (1<<PD3);
	
	if(baud == 0)	UBRR1L = 16;
	else if(baud == 1)	UBRR1L = 7;
	else if(baud == 2)	UBRR1L = 3;
	else if(baud == 3)	UBRR1L = 1;
	
	UCSR1A |= (1<< U2X1);
	UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	UCSR1C |= (1<<USBS1)|(3<<UCSZ1);
}

void inline __attribute__((always_inline)) UPDI_UartDeinit(void){
	DDRD &= ~(1 << PD2);
	DDRD &= ~(1 << PD3);
	PORTD &= ~(1<<PD3);

	UCSR1B = 0;
}


void inline __attribute__((always_inline)) UPDI_UartWriteByte(uint8_t data){
	UDR1 = data;
	while ( !( UCSR1A & (1<<UDRE1)) );
}


void __attribute__((optimize("O0"))) __attribute__((always_inline)) inline UPDI_DelayUs(uint16_t us){
	us = us*2;
	while(us--){
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
}


#endif /* IO_H_ */