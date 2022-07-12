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

#define SET_C2CK PORTB |= (1<<1)
#define RESET_C2CK PORTB &= ~(1<<1)

#define SET_C2D PORTB |= (1<<2)
#define RESET_C2D PORTB &= ~(1<<2)

#define C2D_DATA (PINB & (1<<2))

#define C2CK_OUT DDRB |= (1<<1)
#define C2D_OUT DDRB |= (1<<2)
#define C2D_IN DDRB &= ~(1<<2)

#define C2CK_IN DDRB &= ~(1<<1)

#define SET_C2CK PORTB |= (1<<1)
#define RESET_C2CK PORTB &= ~(1<<1)

void __attribute__((always_inline)) inline UPDI_Pin_Set(uint8_t ch){
	if(ch == 0) PORTD |= (1<<PD3);
	else if(ch == 1) PORTD |= (1<<PD3);
}

void __attribute__((always_inline)) inline UPDI_Pin_Reset(uint8_t ch){
	if(ch == 0) PORTD &= ~(1<<PD3);
	else if(ch == 1) PORTD &= ~(1<<PD3);
}

uint8_t __attribute__((always_inline)) inline UPDI_Read(uint8_t ch){
	if(ch == 0) return (PIND & (1<<3));
	return (PIND & (1<<3));
}


void __attribute__((always_inline)) inline UPDI_IO(uint8_t ch, uint8_t io){
	if(ch == 0){
		if(io)DDRD |= (1<<3);
		else DDRD &= ~(1<<3);
	}else if(ch == 1)
	{
		if(io)DDRD |= (1<<3);
		else DDRD &= ~(1<<3);
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
	UCSR1C |= (1<<USBS1)|(3<<UCSZ1) | (1<<UPM11);
	UCSR1B |= (1<<TXEN1);
}

void inline __attribute__((always_inline)) UPDI_UartDeinit(void){
	DDRD &= ~(1 << PD2);
	DDRD &= ~(1 << PD3);
	PORTD &= ~(1<<PD3);

	UCSR1B = 0;
}


void inline __attribute__((always_inline)) UPDI_UartWriteByte(uint8_t data){
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1 = data;
	while ( !( UCSR1A & (1<<TXC1)) );
	UCSR1A |= (1<<TXC1);
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

uint8_t inline __attribute__((always_inline)) UPDI_UartReadBytes(uint8_t *data, uint16_t len){
	uint16_t tim = 0;
	UCSR1B |= (1<<RXEN1);
	for(uint16_t i = 0;i<len;i++){
		tim = 0;
		while (!(UCSR1A & (1<<RXC1))){
			tim++;
			if(tim >= 60) return 1;
			UPDI_DelayUs(100);
		}
		*(data+i) = UDR1;
	}
	UCSR1B &= ~(1<<RXEN1);
	return 0;
}


#endif /* IO_H_ */