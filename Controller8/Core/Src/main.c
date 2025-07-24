/**
  ******************************************************************************
  * File    main.c
  * Author  HAN University of Applied Sciences
  * Version V1.0
  * Date    23 January 2019
  * Brief   Project for Distributed Systems
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

// Includes --------------------------------------------------------------------
#include "stm32f0xx.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "cni.h"
#include "fake_sensor.h"
#include "global_clock.h"
#include "tii.h"
#include "AdaptiveController.h"

// Private typedef -------------------------------------------------------------

typedef enum
{
	NODE = 0,
	MASTER
}mode_t;

// Private define --------------------------------------------------------------
// Private macro ---------------------------------------------------------------
// Private variables -----------------------------------------------------------
static volatile mode_t mode = MASTER;
static volatile uint32_t local_tick = 0;
volatile bool setPointOn = 0;


// Private function prototypes -------------------------------------------------
void ConfigureGPIO(void);
void ConfigureExternalIT(void);
void delay_ms(const uint32_t t);
void Inc_LocalClock(void);
uint32_t Get_LocalClock(void);

// Private functions -----------------------------------------------------------
void statesToInt(double x)
{
	char *tmpSign = (x < 0) ? "-" : "";
	double tmpVal = (x < 0) ? -x : x;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	double tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).
	TransmitString_TII(tmpSign);
	printStates(tmpInt1);
	TransmitString_TII(".");
	printStates(tmpInt2);
}
void printStates(int number)
{
	int16_t base_val = 10, digit, i = 0, n = 0;
		char x_str[32], t;
		while (number)
		{
			digit = number % base_val;
			number /= base_val;
			if (digit < 10)
			x_str[n++] = digit + '0';
		    else
			 x_str[n++] = digit + 'A' - 10;   /*handle base > 9 */
		}
		/* Terminate string */
		x_str[n] = '\0';
		/* Reverse string */
		for (i=0; i<n/2; i++)
		{
			t = x_str[i];
			x_str[i] = x_str[n-i-1];
			x_str[n-i-1] = t;
	    }
		if (x_str[0]=='\0')
		{
			TransmitString_TII("0");
		}
		else
		{
			TransmitString_TII(x_str);
		}
}

void Inc_LocalClock(void)
{
	local_tick++;
}
uint32_t Get_LocalClock(void)
{
	return local_tick;
}
/**
  * Brief  Main program.
  * Param  None
  * Return None
  */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
     * this is done through SystemInit() function which is called from startup
     * file (startup_stm32f072xb.s) before to branch to application main.
     * To reconfigure the default setting of SystemInit() function, refer to
     * system_stm32f0xx.c file
     */

	SysTick_Config(80000); // 8MHz / 8000 = 1000 Hz = 1ms, SysTick interrupt every 1ms(System runs on 8MHz)

	Configure_GlobalClock();

    ConfigureGPIO();
    ConfigureExternalIT();
    ConfigureGPIO_TII(); // Used for monitoring the component
    Configure_TII();
    Configure_CNI(); // For master-node communication
    Configure_FakeSensor();

    AdaptiveController_initialize();

    TransmitString_TII("Controller\r\n");

    while(1) // Infinite loop
    {
    	if(mode == MASTER)
    	{
    		IncomingMessageHandler_CNI();
    		AdaptiveController_step();
    		observation_t obs =
    			{"reference clock", Get_GlobalClock(), Get_GlobalClock()};

    		// State estimation
    		// WCETsend = discarded
			// WCCOM = 1/baudrate * nof bits in frame * sizeof(observation_t)
			//       = 25ms
    		// ---------------------------------------------------------- START
    		// REMOVE THIS CODE
    		// ----------------------------------------------------------
    		// WCCOM = 1/9600 * nof bits in frame * sizeof(observation_t)
    		//       = 1/9600 * (1 + 8 + 1) * (16 + 4 + 4)
    		//       = 1/9600 * (10) * (24)
    		//       = 25ms
    		// ----------------------------------------------------------- STOP
    		obs.time += 1;
    		obs.val += 1;

    		// Transmit the observation message
    		TransmitMessage_CNI(&obs);

			statesToInt(rtU.referenceThetaDoubleDot);        // Show set point
			TransmitString_TII(" ");
			statesToInt(rtY.control);     		 // torque
			TransmitString_TII(" ");
    		statesToInt(rtU.thetaDoubleDot);     // Angular acceleration
    		TransmitString_TII(" ");
    		statesToInt(rtU.thetaDot);           // Angular velocity
    		TransmitString_TII(" ");
    		statesToInt(rtU.theta);              // Angular position
    		TransmitString_TII(" ");
    		statesToInt((double)Get_GlobalClock());        // Show global clock

    		TransmitString_TII("\r\n");

			observation_t obss =
				{"torque", Get_GlobalClock(), (double)rtY.control};

			// State estimation is not possible, because there is no
			// mathematical relation between the global time and the fake
			// sensor's value

			// Transmit the observation message
			TransmitMessage_CNI(&obss);
    	    if((local_tick % 1000) == 0) {
    	        // Global time sync

        		// Send global clock observation message
        		// Read the RT image of the global clock

    	    }
    	    if((local_tick % 500) == 0){
        		// ---------------------------------------------------------- START

        		// ----------------------------------------------------------- STOP
    	    }
    	}
    	else // Node
    	{
			// Check if there are any messages


    		// Read the RT image of the fake sensor
    		if(Get_FakeSensorValue() == 1)
    		{
    			// Transmit message
    			TransmitString_TII("B1 clicked on master\r\n");
    		}
    	}
    }
}

/**
  * Brief  This function:
  *        - loops for t milliseconds
  *        - uses the global variable tick and assumes that this global
  *        - variable is incremented every 1ms
  * Param  None
  * Return None
  */
inline void delay_ms(const uint32_t t)
{
	uint32_t tick_stop = local_tick + t;

	while(local_tick < tick_stop)
	{
		; // Do nothing
	}
}

/**
  * Brief  This function:
  *        - enables the peripheral clocks on GPIO port A,
  *        - configures GPIO PA5 in output mode for the Green LED pin
  * Param  None
  * Return None
  */
inline void ConfigureGPIO(void)
{  
    // 1. Enable the peripheral clock of GPIOA
    // 2. Select output mode (01) on PA5
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
}

/**
  * Brief  This function:
  *        - enables the peripheral clocks on PORTC
  *        - PC13 is kept in the default configuration
  *          (input, no pull-up, nopull-down)
  *        - EXTICR is configured to select PORTC for EXTI13
  *        - enables interrupts on both falling and rising edges
  *        - configures NVIC IRQ
  * Param  None
  * Return None
  */
inline void ConfigureExternalIT(void)
{  
    // 1. Enable the peripheral clock of SYSCFG
    // 2. Enable the peripheral clock of GPIOC
    // 3. Select PORTC for EXTI13 by writing 0010 in SYSCFG_EXTICR4
    // 4. Configure the corresponding mask bit in the EXTI_IMR register
    // 5. Configure the Trigger Selection bits on falling edge
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_MR13;
    EXTI->FTSR |= EXTI_FTSR_TR13;

    // Configure NVIC for External Interrupt
    // 6. Set priority for EXTI4_15
    // 7. Enable Interrupt on EXTI4_15
    NVIC_SetPriority(EXTI4_15_IRQn, 3);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * Brief  This function handles NMI exception.
  * Param  None
  * Return None
  */
void NMI_Handler(void)
{
}

/**
  * Brief  This function handles Hard Fault exception.
  * Param  None
  * Return None
  */
void HardFault_Handler(void)
{
    // Go to infinite loop when Hard Fault exception occurs
    while (1)
    {
    }
}

/**
  * Brief  This function handles SVCall exception.
  * Param  None
  * Return None
  */
void SVC_Handler(void)
{
}

/**
  * Brief  This function handles PendSVC exception.
  * Param  None
  * Return None
  */
void PendSV_Handler(void)
{
}

/**
  * Brief  This function handles SysTick exception.
  * Param  None
  * Return None
  */
void SysTick_Handler(void)
{
	// Increment local clock every 1ms
	local_tick++;

	// Increment global clock every 50ms
	if((local_tick % 50) == 0)
	{
		// Write the RT image of the global clock
		Inc_GlobalClock();
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                             */
/******************************************************************************/

/**
  * Brief  This function handles EXTI4_15 interrupt request.
  * Param  None
  * Return None
  */
void EXTI4_15_IRQHandler(void)
{
    // Check line 13 has triggered the IT
    if ((EXTI->PR & EXTI_PR_PR13) != 0)
    {
    if (setPointOn==0)
    {
    	setPointOn = 1;
    	rtU.referenceThetaDoubleDot = 1;
    }
    else if (setPointOn==1)
    {
    	setPointOn=0;
    	rtU.referenceThetaDoubleDot = 0;
    }

        // Toggle the green LED
        GPIOA->ODR ^= GPIO_ODR_5;

        // Clear the pending bit
        EXTI->PR = EXTI_PR_PR13;
    }
    else // Should never occur
    {
        ;
    }

}
