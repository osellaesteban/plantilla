/*
 * delay.h
 *
 *  Created on: 18/5/2019
 *      Author: root
 */

#ifndef _DELAY_C_
#define _DELAY_C_

#include "delay.h"

#include "chip.h"
#include "stopwatch.h"

void DelayUs(uint32_t uS)
{
	StopWatch_DelayUs(uS);
}

void DelayMs(uint32_t mS)
{
	StopWatch_DelayMs(mS);
}

#endif /* PROJECTS_STABILITYOBSERVER_SRC_DELAY_C_ */
