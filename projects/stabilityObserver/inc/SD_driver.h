/* SD_driver.h
 *
 *  Created on: 14/5/2019
 *      Author: root
 */

#ifndef _SD_DRIVER_H_
#define _SD_DRIVER_H_

#include "chip.h"
#include "ff.h"
#include "board.h"

void SD_spiConfig(void);

void SD_spiBitRate_config (uint64_t rate);

void SD_mount(void);

/*Reads from file, fill data buffer and return the number of
 * bytes read from file*/
uint32_t SD_read(const TCHAR* nombre, const void* buffer);

/*Writes the number of bytes from buffer to file*/
void SD_write(const TCHAR* nombre, const void* buffer, uint16_t bytes);

void SD_umount(void);

void CS_SetLow(void);

void CS_SetHigh(void);

#endif /* _SD_DRIVER_H_ */
