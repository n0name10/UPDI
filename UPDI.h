#include <avr/io.h>
#include "../ProgLogic/ProgLogic_CMD.h"
#include "UPDI_IO.h"

#ifndef UPDI_H_
#define UPDI_H_

uint8_t UPDI_Init(void);
uint8_t UPDI_DeInit(void);
uint8_t UPDI_EraseDevice(void);
uint8_t UPDI_NVMProg(void);
uint8_t UPDI_ReadSib(uint8_t* sib);

#endif 