/*
 * SD_driver.c
 *
 *  Created on: 14/5/2019
 *      Author: root
 */
#include "SD_driver.h"

#include "delay.h"


static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file

FRESULT fr;
FSIZE_t	fil_size;
UINT nbytes;
BYTE type;

void SD_spiConfig(void)
{

	/* Set up clock and power for SSP1 module */
	// Configure SSP SSP1 pins
	Chip_SCU_PinMuxSet(0xf, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CLK0
	Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); // MISO1
	Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5)); // MOSI1

	Chip_SCU_PinMuxSet(0x6, 1, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CS1 configured as GPIO0
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 0);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 3, 0, FALSE);
	// Initialize SSP Peripheral
	Chip_SSP_Init( LPC_SSP1 );
	Chip_SSP_Enable( LPC_SSP1 );

}

void SD_spiBitRate_config (uint64_t rate)
{
	Chip_SSP_SetBitRate(LPC_SSP1, rate); //10000000
}

void SD_mount(void)
{
	uint8_t result = 1;
	fs.fs_type = FS_FAT32;
	// Give a work area to the default drive
	result= f_mount( &fs, "", 0 );

	if( result== FR_OK ){
		LedOn( LED_GREEN );

	}
	else {
		LedOn( LED_RED );
	}

}


uint32_t SD_read(const TCHAR* nombre, const void* buffer)
{
	uint8_t result =1;
	FILINFO info;
	uint32_t size;

	fr = f_stat(nombre, &info);								/*Get file status*/
	size = info.fsize;										/*size of file*/

	if(fr == FR_OK)
	{
		result = f_open( &fp, nombre, FA_READ | FA_OPEN_EXISTING );

		if( result == FR_OK )
		{
			f_read( &fp, buffer, size, &nbytes );			/*read the file and fill the buffer*/

			if(nbytes == size) LedOn( LED_1 );			/*if file successfully read, turn on yellow led*/

	        f_close(&fp);
		}

	}

	return nbytes;

}

void SD_write(const TCHAR* nombre, const void* buffer, uint16_t bytes)
{
	uint8_t result =1;
	FILINFO info;

	result = f_stat(nombre, &info);

	switch(result)
	{
		case FR_NO_FILE:
			result = f_open( &fp, nombre, FA_WRITE |  FA_CREATE_ALWAYS );
			break;
		case FR_OK:
			result = f_open( &fp, nombre, FA_WRITE | FA_OPEN_APPEND  );
			break;
	}

	if(result == FR_OK ){

	   f_write( &fp, buffer, bytes, &nbytes );				/*write the number of bytes data from buffer to file
	    													 nbytes is the number of bytes actually written*/
	   if(nbytes == bytes) {

		  // LedOn( LED_1 );				/*if file successfully written...*/
		  // DelayMs(500);
		   //LedOff( LED_1 );
	   }

	   f_close(&fp);
	}

}

void SD_umount(void)
{
	LedOff( LED_GREEN);

	f_mount(NULL, "", 0);
}


void CS_SetLow()
{
	//Chip_SCU_PinMuxSet(0x06, 0x01, SCU_MODE_FUNC0 | SCU_MODE_INACT);
	//Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0x03, 0x00);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x03, 0x00, FALSE);

}

void CS_SetHigh()
{
	//Chip_SCU_PinMuxSet(0x06, 0x01, SCU_MODE_FUNC0 | SCU_MODE_INACT);
	//Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0x03, 0x00);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x03, 0x00, TRUE);
}


