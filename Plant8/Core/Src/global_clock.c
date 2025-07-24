/**
  ******************************************************************************
  * File    global_clock.c (Global clock object)
  * Author  HAN University of Applied Sciences
  * Version V1.0
  * Date    23 January 2019
  * Brief   UART functions and definitions
  *
  * Copyright (C) 2019 HAN University of Applied Sciences. All Rights Reserved.
  *
  * Permission is hereby granted, free of charge, to any person obtaining a
  * copy of this software and associated documentation files (the "Software"),
  * to deal in the Software without restriction, including without limitation
  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the
  * Software is furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  ******************************************************************************
  */

#include "global_clock.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint32_t global_tick = 0; // Incremented every 50ms
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * Brief   This function:
           -
 * Param   None
 * Retval  None
 */
void Configure_GlobalClock(void)
{
	global_tick = 0;
}

uint32_t Get_GlobalClock(void)
{
	return global_tick;
}

void Set_GlobalClock(uint32_t t)
{
	global_tick = t;

	// For debugging purpose: show the synchronization of the global clocks
	// by toggling an LED
	if((global_tick % 20) == 0)
	{
		GPIOA->BSRR = GPIO_BSRR_BR_5;
	}
	else if((global_tick % 10) == 0)
	{
		GPIOA->BSRR = GPIO_BSRR_BS_5;
	}
}

void Inc_GlobalClock(void)
{
	global_tick++;

	// For debugging purpose: show the synchronization of the global clocks
	// by toggling an LED
	if((global_tick % 20) == 0)
	{
		GPIOA->BSRR = GPIO_BSRR_BR_5;
	}
	else if((global_tick % 10) == 0)
	{
		GPIOA->BSRR = GPIO_BSRR_BS_5;
	}
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/
