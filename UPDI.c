/*
* RL78_Protocol.c
*
* Created: 23.06.2022 12:07:44
*  Author: adam.pawlowski
*/
#include "UPDI.h"

#define TIME_OUT 5000

#define OK 0x00
#define ERR 0x01
#define NO_ACK 0x02
#define SYNCH 0x55
#define ACK 0x40

#define OPCODE_LDS 0x00
#define OPCODE_SDS 0x02
#define OPCODE_LD 0x01
#define OPCODE_SD 0x03
#define OPCODE_LDCS 0x04
#define OPCODE_SDCS 0x06
#define OPCODE_REPEAT 0x05
#define OPCODE_KEY 0x07

#define BYTE_SIZE 0x00
#define WORD_SIZE 0x01
#define TRIBYTES_SIZE 0x02
#define ADR_SIZE_POS 2
#define DATA_SIZE_POS 0
#define REG_POS 0
#define OPCODE_POS 5

#define PTR_ACCESS_POS 2
#define KEY_SIZE_POS 0
#define SIB_POS 2
#define SIZE_POS 0

#define PTR_DATA_ACCESS 0
#define PTR_DATA_INC_ACCESS 1
#define PTR_ACCESS 2

#define KEY_ERASE 0x4E564D4572617365
#define KEY_NVMPROG 0x4E564D50726F6720
#define KEY_USERROW_WRITE 0x4E564D5573267465
#define RESET_SIGNATURE 0x59

#define STATUSA 0x00
#define STATUSB	0x01
#define CTRLA 0x02
#define CTRLB 0x03
#define ASI_KEY_STATUS 0x07
#define ASI_RESET_REQ 0x08
#define ASI_CTRLA 0x09
#define ASI_SYS_CTRLA 0x0A
#define ASI_SYS_STATUS 0x0B
#define ASI_CRC_STATUS 0x0C

#define LOCKSTATUS_BIT 0
#define ERASEFAILED_BIT 1
#define NVMPROG_BIT 3

uint8_t UPDI_UartWriteBytes(uint8_t* adr, uint8_t len);
uint8_t UPDI_LD(uint8_t* data, uint8_t access, uint8_t size);
uint8_t UPDI_SD(uint8_t* data, uint8_t access, uint8_t size);
uint8_t UPDI_SDS(uint16_t adr, uint8_t* data, uint8_t adr_size, uint8_t data_size);
uint8_t UPDI_LDS(uint16_t adr, uint8_t* data, uint8_t adr_size, uint8_t data_size);
uint8_t UPDI_LDCS(uint8_t reg, uint8_t* data);
uint8_t UPDI_SDCS(uint8_t reg, uint8_t data);
uint8_t UPDI_Repeat(uint8_t size);
uint8_t UPDI_WriteKey(uint64_t key);
uint8_t UPDI_WriteBlockAndCheck(uint16_t adr, uint8_t len, uint8_t *data);
uint8_t UPDI_DataIsEqual(uint8_t* data1,uint8_t* data2, uint16_t len);
uint8_t UPDI_UartReadBytes(uint8_t *data, uint16_t len);

uint8_t NVMPROG_WriteCtrla(uint8_t cmd);
uint8_t NVMPROG_WaitFlashBusy(void);

uint8_t UPDI_Reset(void);
uint8_t UPDI_WaitForAck(void);
uint8_t UPDI_IsSet(uint16_t tim);

uint8_t UPDI_Init(void){
	UPDI_UartDeinit();
	UPDI_IO(0,1);
	UPDI_Pin_Reset(0);
	UPDI_DelayUs(1);
	UPDI_IO(0,0);
	UPDI_Pin_Set(0);
	UPDI_IsSet(0);
	UPDI_UartInit(0);
	UPDI_DelayUs(100);
	UPDI_SDCS(CTRLB,(1<<3));
	return OK;
}

uint8_t UPDI_DeInit(void){
	UPDI_SDCS(CTRLB,(1<<2));
	return OK;
}

uint8_t UPDI_LD(uint8_t* data, uint8_t access, uint8_t size){
	uint8_t ld = OPCODE_LD<<OPCODE_POS | access<<PTR_ACCESS_POS | size<<SIZE_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(ld);
	return OK;
}
uint8_t UPDI_SD(uint8_t* data, uint8_t access, uint8_t size){
	uint8_t ld = OPCODE_SD<<OPCODE_POS | access<<PTR_ACCESS_POS | size<<SIZE_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(ld);
	return OK;
}

uint8_t UPDI_SDS(uint16_t adr, uint8_t* data, uint8_t adr_size, uint8_t data_size){
	uint8_t sts = OPCODE_SDS<<OPCODE_POS | WORD_SIZE<<ADR_SIZE_POS | BYTE_SIZE<<DATA_SIZE_POS;
	uint8_t status = 0;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(sts);
	UPDI_UartWriteBytes((uint8_t*)&adr,adr_size+1);
	status = UPDI_WaitForAck();
	if(status != OK) return status;
	UPDI_UartWriteBytes(data,data_size+1);
	status = UPDI_WaitForAck();
	if(status != OK) return status;
	return OK;
}

uint8_t UPDI_LDS(uint16_t adr, uint8_t* data, uint8_t adr_size, uint8_t data_size){
	uint8_t lds = OPCODE_LDS<<OPCODE_POS | adr_size<<ADR_SIZE_POS | data_size<<DATA_SIZE_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(lds);
	UPDI_UartWriteBytes((uint8_t*)&adr,adr_size+1);
	return UPDI_UartReadBytes(data, data_size+1);
}

uint8_t UPDI_LDCS(uint8_t reg, uint8_t* data){
	uint8_t ldcs = OPCODE_LDCS<<OPCODE_POS | reg<<REG_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(ldcs);
	UPDI_UartReadBytes(data,1);
	return OK;
}

uint8_t UPDI_SDCS(uint8_t reg, uint8_t data){
	uint8_t stcs = OPCODE_SDCS<<OPCODE_POS | reg<<REG_POS ;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(stcs);
	UPDI_UartWriteByte(data);
	return OK;
}

uint8_t UPDI_Repeat(uint8_t size){
	uint8_t repeat = OPCODE_REPEAT<<OPCODE_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(repeat);
	UPDI_UartWriteByte(size);
	return OK;
}


uint8_t UPDI_UartWriteBytes(uint8_t* adr, uint8_t len){
	for(uint16_t i = 0;i<len;i++){
		UPDI_UartWriteByte(*(adr+i));
	}
	return OK;
}

uint8_t UPDI_WriteKey(uint64_t key){
	uint8_t buf = OPCODE_KEY<<OPCODE_POS | 0x00<<KEY_SIZE_POS | 0x00<<SIB_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(buf);
	UPDI_UartWriteBytes((uint8_t*)&key,8);
	return OK;
}

uint8_t UPDI_EraseDevice(void){
	uint8_t buf = 0;
	UPDI_WriteKey(KEY_ERASE);
	UPDI_LDCS(ASI_KEY_STATUS,&buf);
	if(!(buf&(1<<3)))return ERR;
	UPDI_Reset();
	for(uint16_t i = 0;i<100;i++){
		UPDI_LDCS(ASI_SYS_STATUS,&buf);
		if(!(buf&(1<<LOCKSTATUS_BIT))) return OK;
		UPDI_DelayUs(100);
	}
	return ERR;
}

uint8_t UPDI_Reset(void){
	UPDI_SDCS(ASI_RESET_REQ,RESET_SIGNATURE);
	UPDI_SDCS(ASI_RESET_REQ,0);
	return OK;
}

#define BLOCK_SIZE 128

uint8_t UPDI_WriteDataAndCheck(uint16_t adr, uint16_t len, uint8_t *data){
	uint8_t status = 0;
	uint8_t blocks = len/BLOCK_SIZE;
	uint8_t complement = len%BLOCK_SIZE;
	if(UPDI_StartProg() != OK) return ERR;
	for(uint16_t i = 0;i<blocks;i++){
		status = UPDI_WriteBlockAndCheck(adr+i*BLOCK_SIZE,BLOCK_SIZE,data+i*BLOCK_SIZE);
		if(status != OK) return status;
	}
	status = UPDI_WriteBlockAndCheck(adr+blocks*BLOCK_SIZE,complement,data+blocks*BLOCK_SIZE);
	if(status != OK) return status;
	UPDI_Reset();
	return OK;
}

uint8_t NVMPROG_WriteCtrla(uint8_t cmd){
	UPDI_SDS(0x1000,&cmd,1,0);
	return OK;
}

uint8_t NVMPROG_WaitFlashBusy(void){
	uint8_t status = 0;
	for(uint8_t i=0;i<10;i++){
		UPDI_LDS(0x1002,&status,1,0);
		if(status == 0) return OK;
		UPDI_DelayUs(100);
	}
	return ERR;
}


uint8_t UPDI_StartProg(void){
	uint16_t buf = 0;
	UPDI_WriteKey(KEY_NVMPROG);
	UPDI_LDCS(ASI_KEY_STATUS,(uint8_t*)&buf);
	if(!(buf&(1<<4)))return ERR;
	UPDI_Reset();
	for(uint16_t i = 0;i<100;i++){
		UPDI_LDCS(ASI_SYS_STATUS,(uint8_t*)&buf);
		if(buf&(1<<NVMPROG_BIT)) break;
		UPDI_DelayUs(100);
	}
	return OK;
}

uint8_t UPDI_WriteBlock(uint16_t adr, uint8_t len, uint8_t *data){
	UPDI_SDS(adr,data,1,0);
	NVMPROG_WriteCtrla(0x02);
	NVMPROG_WaitFlashBusy();
	NVMPROG_WriteCtrla(0x04);
	NVMPROG_WaitFlashBusy();
	UPDI_SD((uint8_t*)&adr,2,1);
	UPDI_UartWriteBytes((uint8_t*)&adr,2);
	if(UPDI_WaitForAck() != OK)return ERR;
	UPDI_Repeat(len-1);
	UPDI_SD(data,1,0);
	for(uint8_t i = 0;i<len;i++){
		UPDI_UartWriteByte(*(data+i));
		if(UPDI_WaitForAck() != OK)return ERR;
	}
	NVMPROG_WriteCtrla(0x03);
	NVMPROG_WaitFlashBusy();
	return OK;
}

uint8_t UPDI_WriteBlockAndCheck(uint16_t adr, uint8_t len, uint8_t *data){
	uint8_t status = 0;
	uint8_t buf[BLOCK_SIZE];
	if(len>BLOCK_SIZE) return ERR;
	status = UPDI_WriteBlock(adr,len,data);
	if(status != OK) return status;
	status = UPDI_ReadBlock(adr,len,buf);
	if(status != OK) return status;
	status = UPDI_DataIsEqual(data,buf,len);
	if(status != OK) return status;
	return OK;
}

uint8_t UPDI_DataIsEqual(uint8_t* data1,uint8_t* data2, uint16_t len){
	for(uint8_t j = 0;j<len;j++){
		if(*(data1+j) != *(data2+j)) return ERR;
	}
	return OK;
}

uint8_t UPDI_ReadBlock(uint16_t adr, uint8_t len, uint8_t *data){
	UPDI_SD((uint8_t*)&adr,2,1);
	UPDI_UartWriteBytes((uint8_t*)&adr,2);
	if(UPDI_WaitForAck() != OK)return ERR;
	UPDI_Repeat(len-1);
	UPDI_LD(data,1,0);
	UPDI_UartReadBytes(data, len);
	return OK;
}

uint8_t UPDI_ReadSib(uint8_t* sib){
	uint8_t key = OPCODE_KEY<<OPCODE_POS | 0x01<<KEY_SIZE_POS | 0x01<<SIB_POS;
	UPDI_UartWriteByte(SYNCH);
	UPDI_UartWriteByte(key);
	UPDI_UartReadBytes(sib, 16);
	return OK;
}


uint8_t UPDI_WaitForAck(void){
	uint8_t data = 0;
	if(UPDI_UartReadBytes(&data,1)) return ERR;
	if(data == ACK) return OK;
	return NO_ACK;
}

uint8_t UPDI_IsSet(uint16_t tim){
	for(uint16_t i = 0;i<tim*10;i++){
		if(UPDI_Read(0) & (1<<0)) return OK;
		UPDI_DelayUs(1);
	}
	return ERR;
}

volatile uint16_t len_rx = 0, flag_rx = 0,rx_counter = 0;
volatile uint8_t* rx_data_wsk;

uint8_t UPDI_UartReadBytes(uint8_t *data, uint16_t len){
	len_rx = len;
	rx_data_wsk = data;
	rx_counter = 0;
	flag_rx = 1;
	for(uint16_t i = 0;i<2000;i++){
		if(flag_rx == 0) return OK;
		UPDI_DelayUs(10);
	}
	flag_rx = 0;
	return ERR;
}

void UPDI_ByteRx_Handler(uint8_t rx_data){
	if(flag_rx){
		rx_data_wsk[rx_counter++] = rx_data;
		if(rx_counter == len_rx){
			flag_rx = 0;
		}
	}else rx_counter = 0;
}
