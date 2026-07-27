/*
 * sd_spi.c
 *
 *  Created on: 14 Jul 2026
 *      Author: yokyo
 */

#include "sd_spi.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

static DSTATUS sd_status = STA_NOINIT;
static uint8_t sd_is_high_capacity = 0;

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

static uint8_t SD_SPI_TxRx(uint8_t data){
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;}

static void SD_SPI_SendDummyClocks(uint8_t count){
    SD_CS_HIGH();
    for (uint8_t i = 0; i < count; i++){
        SD_SPI_TxRx(0xFF);}
}

static uint8_t SD_SPI_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc){
    uint8_t response;
    SD_CS_LOW();

    SD_SPI_TxRx(0xFF);   // give card 8 clocks after CS goes low

    SD_SPI_TxRx(cmd | 0x40);

    SD_SPI_TxRx((uint8_t)(arg >> 24));
    SD_SPI_TxRx((uint8_t)(arg >> 16));
    SD_SPI_TxRx((uint8_t)(arg >> 8));
    SD_SPI_TxRx((uint8_t)arg);

    SD_SPI_TxRx(crc);

    // give card 80 clocks
    for (uint8_t i = 0; i < 10; i++){
        response = SD_SPI_TxRx(0xFF);

        if ((response & 0x80) == 0){
            return response;}
    }
    return 0xFF;
}

static uint8_t SD_SPI_WaitForDataToken(void){
    uint8_t token;
    uint32_t timeout = HAL_GetTick() + 500;

    do{
    	token = SD_SPI_TxRx(0xFF);
        if (token == 0xFE){
            return 1;}
    } while (HAL_GetTick() < timeout);
    return 0;
}

static DRESULT SD_SPI_ReadBlock(BYTE *buff, DWORD sector){
    uint8_t response;
    uint32_t address;
    if (sd_is_high_capacity){
        address = sector;}
    else{
        address = sector * 512;}

    response = SD_SPI_SendCommand(17, address, 0xFF); // CMD17 = read single block

    if (response != 0x00){
        return RES_ERROR;}

    if (!SD_SPI_WaitForDataToken()){
        return RES_ERROR;}

    for (uint16_t i = 0; i < 512; i++){
        buff[i] = SD_SPI_TxRx(0xFF);}

    // discard CRC bytes
    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);

    SD_SPI_SendDummyClocks(1);

    return RES_OK;
}

static DRESULT SD_SPI_WriteBlock(const BYTE *buff, DWORD sector){
    uint8_t response;
    uint32_t address;

    if (sd_is_high_capacity){
        address = sector;}
    else{
        address = sector * 512;}

    response = SD_SPI_SendCommand(24, address, 0xFF); // CMD24 = write single block

    if (response != 0x00){
        return RES_ERROR;}

    // Start block token for single block write
    SD_SPI_TxRx(0xFE);

    // Send 512 bytes
    for (uint16_t i = 0; i < 512; i++){
        SD_SPI_TxRx(buff[i]);
    }

    // Dummy CRC
    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);

    // Read data response token
    response = SD_SPI_TxRx(0xFF);

    if ((response & 0x1F) != 0x05){
        return RES_ERROR;}

    // Wait while card is busy writing
    uint32_t timeout = HAL_GetTick() + 500;

    while (SD_SPI_TxRx(0xFF) == 0x00){
        if (HAL_GetTick() > timeout){
            return RES_ERROR;}
    }

    SD_SPI_SendDummyClocks(1);

    return RES_OK;
}

static void SD_Debug_Print(char *msg){
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void SD_SPI_ResetState(void){
	sd_status = STA_NOINIT;
	sd_is_high_capacity = 0;

	SD_CS_HIGH();

	for (uint8_t i = 0; i < 10; i++){
		SD_SPI_TxRx(0xFF);}
}

DSTATUS SD_SPI_Initialize(void){
	uint8_t response;
	HAL_Delay(10);

	SD_SPI_SendDummyClocks(10);   // 80 clocks
	response = SD_SPI_SendCommand(0, 0x00000000, 0x95); // CMD0

	char msg[50];
	sprintf(msg, "CMD0 response: 0x%02X\r\n", response);
	SD_Debug_Print(msg);

	SD_SPI_SendDummyClocks(1); // extra 8 clocks after command

	if (response != 0x01){
		sd_status = STA_NOINIT;
	    return sd_status;}

	response = SD_SPI_SendCommand(8, 0x000001AA, 0x87); // CMD8

	uint8_t r7[4];
	r7[0] = SD_SPI_TxRx(0xFF);
	r7[1] = SD_SPI_TxRx(0xFF);
	r7[2] = SD_SPI_TxRx(0xFF);
	r7[3] = SD_SPI_TxRx(0xFF);

	sprintf(msg, "CMD8 response: 0x%02X\r\n", response);
	SD_Debug_Print(msg);

	sprintf(msg, "CMD8 extra: %02X %02X %02X %02X\r\n", r7[0], r7[1], r7[2], r7[3]);
	SD_Debug_Print(msg);

	SD_SPI_SendDummyClocks(1); // extra 8 clocks after command

	if (response != 0x01 || r7[2] != 0x01 || r7[3] != 0xAA){
	    sd_status = STA_NOINIT;
	    return sd_status;}

	uint32_t timeout = HAL_GetTick() + 1000;

	do{
		response = SD_SPI_SendCommand(55, 0x00000000, 0xFF); // CMD55
	    SD_SPI_SendDummyClocks(1);

	    response = SD_SPI_SendCommand(41, 0x40000000, 0xFF); // ACMD41 with HCS
	    SD_SPI_SendDummyClocks(1);

	    sprintf(msg, "ACMD41 response: 0x%02X\r\n", response);
	    SD_Debug_Print(msg);

	    if (response == 0x00){
	        break;}
	}while (HAL_GetTick() < timeout);

	if (response != 0x00)
	{
	    sd_status = STA_NOINIT;
	    return sd_status;
	}

	response = SD_SPI_SendCommand(58, 0x00000000, 0xFF); // CMD58: read OCR

	uint8_t ocr[4];
	ocr[0] = SD_SPI_TxRx(0xFF);
	ocr[1] = SD_SPI_TxRx(0xFF);
	ocr[2] = SD_SPI_TxRx(0xFF);
	ocr[3] = SD_SPI_TxRx(0xFF);

	sprintf(msg, "CMD58 response: 0x%02X\r\n", response);
	SD_Debug_Print(msg);

	sprintf(msg, "OCR: %02X %02X %02X %02X\r\n", ocr[0], ocr[1], ocr[2], ocr[3]);
	SD_Debug_Print(msg);

	SD_SPI_SendDummyClocks(1);

	if (response != 0x00){
	    sd_status = STA_NOINIT;
	    return sd_status;}

	if (ocr[0] & 0x40){
		sd_is_high_capacity = 1;
	    SD_Debug_Print("Card type: SDHC/SDXC\r\n");}
	else{
	    sd_is_high_capacity = 0;
	    SD_Debug_Print("Card type: SDSC\r\n");}

	// CMD0, CMD8, ACMD41 and CMD58 worked. Card initialisation is complete.
	sd_status = 0;
	return sd_status;
}

DSTATUS SD_SPI_Status(void){
    return sd_status;
}

DRESULT SD_SPI_Read(BYTE *buff, DWORD sector, UINT count){
	if (sd_status & STA_NOINIT){
		return RES_NOTRDY;}
	if (buff == 0 || count == 0){
	    return RES_PARERR;}

	for (UINT i = 0; i < count; i++){
		if (SD_SPI_ReadBlock(buff + (i * 512), sector + i) != RES_OK){
			return RES_ERROR;}
	}
	return RES_OK;
}

DRESULT SD_SPI_Write(const BYTE *buff, DWORD sector, UINT count){
	if (sd_status & STA_NOINIT){ //& not == as we are checking that bit only not necessarily all of the sd_status
	        return RES_NOTRDY;}

	for (UINT i = 0; i < count; i++){
		if (SD_SPI_WriteBlock(buff + (i * 512), sector + i) != RES_OK){
			return RES_ERROR;}
	    }
	    return RES_OK;
}

DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff){
	if (sd_status & STA_NOINIT){
		return RES_NOTRDY;}

	switch (cmd){
		case CTRL_SYNC:
		{
			SD_CS_LOW();
	        uint32_t timeout = HAL_GetTick() + 500;

	        while (SD_SPI_TxRx(0xFF) == 0x00){
	        	if (HAL_GetTick() > timeout){
	        		SD_SPI_SendDummyClocks(1);
	                return RES_ERROR;}
	        }

	        SD_SPI_SendDummyClocks(1);
	        return RES_OK;
		}
	     case GET_SECTOR_SIZE:
	    	 *(WORD*)buff = 512;
	         return RES_OK;

	     case GET_BLOCK_SIZE:
	    	 *(DWORD*)buff = 1;
	    	 return RES_OK;

	     default:
	         return RES_PARERR;
	    }
}
