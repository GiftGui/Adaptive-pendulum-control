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

#include "cni.h"
#include "fake_sensor.h"
#include "global_clock.h"
#include "tii.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "Undamped.h"

// Private typedef -------------------------------------------------------------

typedef enum
{
	NODE = 0,
	MASTER
}mode_t;

// Private define --------------------------------------------------------------
// Private macro ---------------------------------------------------------------
// Private variables -----------------------------------------------------------
static volatile mode_t mode = NODE;
static volatile uint32_t local_tick = 0;

volatile bool forceOn = 0;

// Private function prototypes -------------------------------------------------
void ConfigureGPIO(void);
void ConfigureExternalIT(void);
void delay_ms(const uint32_t t);

void statesToInt(double, int);
void printStates(int);
void Inc_LocalClock(void);
uint32_t Get_LocalClock(void);

// Private functions -----------------------------------------------------------
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

void statesToInt(double x, int decimal_places)
{
    char *tmpSign = (x < 0) ? "-" : "";
    double tmpVal = (x < 0) ? -x : x;

    // Calculate multiplier based on desired decimal places
    int multiplier = 1;
    for(int i = 0; i < decimal_places; i++) {
        multiplier *= 10;
    }

    // Round to avoid floating-point precision issues
    tmpVal = tmpVal + 0.5 / multiplier;

    int tmpInt1 = (int)tmpVal;                              // Integer part
    double tmpFrac = tmpVal - tmpInt1;                      // Fractional part
    int tmpInt2 = (int)(tmpFrac * multiplier);              // Fractional digits as integer

    TransmitString_TII(tmpSign);
    printStates(tmpInt1);
    TransmitString_TII(".");

    // Print fractional part with leading zeros
    printStatesWithLeadingZeros(tmpInt2, decimal_places);
}

// Modified printStates to handle leading zeros for decimal part
void printStatesWithLeadingZeros(int number, int min_digits)
{
    char x_str[32];
    int n = 0, i;

    // Handle zero case
    if (number == 0) {
        for(i = 0; i < min_digits; i++) {
            TransmitString_TII("0");
        }
        return;
    }

    // Convert number to string
    int temp = number;
    while (temp > 0) {
        x_str[n++] = (temp % 10) + '0';
        temp /= 10;
    }

    // Add leading zeros if needed
    while (n < min_digits) {
        x_str[n++] = '0';
    }

    // Reverse and transmit
    for (i = n - 1; i >= 0; i--) {
        char single_char[2] = {x_str[i], '\0'};
        TransmitString_TII(single_char);
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
	SysTick_Config(80000);

	Configure_GlobalClock();

    ConfigureGPIO();
    ConfigureExternalIT();
    ConfigureGPIO_TII(); // Used for monitoring the component
    Configure_TII();
    Configure_CNI(); // For master-node communication
    Configure_FakeSensor();

    /* Initialize Simulink model */
     Undamped_initialize();
     TransmitString_TII("Pendulum Plant Emulator Started\r\n");

     uint32_t tick=0;
     uint32_t lasttick=0;

    while(1) // Infinite loop
    {
    	if(mode == MASTER)
    	{
        	;
    	}
    	else // Node
    	{
    		lasttick = tick;
        	tick= Get_LocalClock();
        	if (tick>lasttick)
        	{
        		IncomingMessageHandler_CNI();
        		// Execute one simulation step
        		Undamped_step();

        		// Output data in the format: torque, acceleration, velocity, displacement
        		statesToInt(rtU.controlTorque, 3);
        		TransmitString_TII((char*)" ");  // Input torque
        		statesToInt(rtY.thetaDoubleDot, 3);     // Angular acceleration
        		TransmitString_TII(" ");
        		statesToInt(rtY.thetaDot, 3);           // Angular velocity
        		TransmitString_TII(" ");
        		statesToInt(rtY.theta, 3);              // Angular position
        		TransmitString_TII(" ");
        		statesToInt((double)Get_GlobalClock(), 3);        // Show global clock

        		TransmitString_TII("\r\n");

        		observation_t obs =
        			{1, (double)rtY.thetaDoubleDot, (double)rtY.thetaDot, (double)rtY.theta};

        		// Transmit the observation message
        		TransmitMessage_CNI(&obs);
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
	// Code for question1 is here, but not be used in subsequent questions
    // Check line 13 has triggered the IT
    if ((EXTI->PR & EXTI_PR_PR13) != 0)
    {
    	if (forceOn == 0)
    	{
    		forceOn = 1;
    		rtU.controlTorque = 100.0;  // Fixed: use controlTorque instead of force
    	}
    	else if (forceOn == 1)
    	{
    		forceOn = 0;
    		rtU.controlTorque = 0.0;    // Fixed: use controlTorque instead of force
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
