/*
 * Hardware_driver.c
 *
 *  Created on: 14/5/2019
 *      Author: root
 */

#include "Hardware_driver.h"

#include "board.h"


void LedOn(uint8_t led)
{
	Board_LED_Set(led, 1);
}

void LedOff(uint8_t led)
{
	Board_LED_Set(led, 0);
}

void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();
    StopWatch_Init();
}

