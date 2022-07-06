/*
* RL78_Protocol.c
*
* Created: 23.06.2022 12:07:44
*  Author: adam.pawlowski
*/
#include "UPDI.h"

#define OK 0x00
#define ERR 0x01
#define SYNCH 0x55

#define LDS 0x00
#define STS 0x02

#define LD 0x01
#define ST 0x03

#define LDCS 0x04
#define STCS 0x06

#define REPEAT 0x05
#define KEY 0x07

uint8_t UPDI_HandShake(void){
	UPDI_Reset(0);
	UPDI_DelayUs(1);
	UPDI_IO(0);
	UPDI_Set(0);
	UPDI_IsSet(0);
	UPDI_UartWriteByte(SYNCH);
}

uint8_t UPDI_IsSet(uint16_t tim){
	for(uint16_t i = 0;i<tim*10;i++){
		if(UPDI_Read(0) & (1<<0)) return OK;
		UPDI_DelayUs(1);
	}
	return ERR;
}