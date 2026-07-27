/*
 * sd_spi.h
 *
 *  Created on: 14 Jul 2026
 *      Author: yokyo
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#include "main.h"
#include "ff.h"
#include "diskio.h"
#include "ff_gen_drv.h"

/*
 * These functions are called by user_diskio.c.
 * They connect FatFS to the SD card over SPI.
 */
void SD_SPI_ResetState(void);

DSTATUS SD_SPI_Initialize(void);

DSTATUS SD_SPI_Status(void);

DRESULT SD_SPI_Read(BYTE *buff, DWORD sector, UINT count);

DRESULT SD_SPI_Write(const BYTE *buff, DWORD sector, UINT count);

DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff);

#endif /* INC_SD_SPI_H_ */
