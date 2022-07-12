#include <avr/io.h>
#include "UPDI_IO.h"

#ifndef UPDI_H_
#define UPDI_H_

uint8_t UPDI_Init(void);
uint8_t UPDI_DeInit(void);
uint8_t UPDI_EraseDevice(void);
uint8_t UPDI_WriteDataAndCheck(uint16_t adr, uint16_t len, uint8_t *data);
uint8_t UPDI_WriteBlock(uint16_t adr, uint8_t len, uint8_t *data);
uint8_t UPDI_ReadBlock(uint16_t adr, uint8_t len, uint8_t *data);
uint8_t UPDI_StartProg(void);
uint8_t UPDI_ReadSib(uint8_t* sib);

#endif 