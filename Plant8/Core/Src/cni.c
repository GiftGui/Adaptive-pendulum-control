/**
  ******************************************************************************
  * File    cni.c (Communication Network Interface)
  * Author  HAN University of Applied Sciences
  * Version V1.0
  * Date    23 January 2019
  * Brief   Communication Network Interface functions and definitions
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

#include "cni.h"

#include "fake_sensor.h"
#include "global_clock.h"
#include "stddef.h"
#include "string.h"
#include "Undamped.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*  UART Buffer Defines */
#define UART1_RX_BUFFER_SIZE 256 /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART1_TX_BUFFER_SIZE 256

#define UART1_RX_BUFFER_MASK (UART1_RX_BUFFER_SIZE - 1)
#if (UART1_RX_BUFFER_SIZE & UART1_RX_BUFFER_MASK)
    #error RX buffer size is not a power of 2
#endif
#define UART1_TX_BUFFER_MASK (UART1_TX_BUFFER_SIZE - 1)
#if (UART1_TX_BUFFER_SIZE & UART1_TX_BUFFER_MASK)
    #error TX buffer size is not a power of 2
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
static volatile char UART1_RxHead;
static volatile char UART1_RxTail;
static char UART1_TxBuf[UART1_TX_BUFFER_SIZE];
static volatile char UART1_TxHead;
static volatile char UART1_TxTail;

/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO_UART1(void);
void Configure_UART1(void);
char ReceiveByte_CNI(void);
void TransmitByte_CNI(char data);
unsigned char nUnreadBytes_CNI(void);
uint8_t ReceiveMessage_CNI(observation_t *obs);

/**
 * Brief   This function:
           - configures the CNI
 * Param   None
 * Retval  None
 */
inline void Configure_CNI(void)
{
	ConfigureGPIO_UART1();
	Configure_UART1();
}

void TransmitMessage_CNI(const observation_t *obs)
{
	char *p = (char *)obs;

	// Transmit the observation message
	for(uint32_t i = 0; i < sizeof(observation_t); i++)
	{
		TransmitByte_CNI(*(p++));
	}
}

void IncomingMessageHandler_CNI(void)
{
	observation_t obs = {};

	// Receive an observation message
	if(ReceiveMessage_CNI(&obs))
	{
		// -------------------------------------------------------------- START
		// REMOVE THIS CODE
		// --------------------------------------------------------------
		if(obs.id == 0)
		{
			rtU.controlTorque = (real_T)obs.val1;
		}
		// --------------------------------------------------------------- STOP
	}
}

uint8_t ReceiveMessage_CNI(observation_t *obs)
{
	// Data available?
	if(nUnreadBytes_CNI() > 0)
	{
		// Wait for approximately 40ms, making sure an entire message must have
		// arrived
		uint32_t clock_stop = Get_GlobalClock() + 4;
		while(Get_GlobalClock() < clock_stop)
		{;}

		// Received at least an entire message?
		if(nUnreadBytes_CNI() >= sizeof(observation_t))
		{
			char *p = (char *)obs;

			for(uint32_t i = 0; i < sizeof(observation_t); i++)
			{
				*(p++) = ReceiveByte_CNI();
			}

			return 1; // Successfully received a message
		}
		else // Invalid message length
		{
			// Flush the data
			while(nUnreadBytes_CNI() != 0)
			{
				(void)ReceiveByte_CNI();
			}

			return 0;
		}
	}

	// No data available
	return 0;
}

/**
 * Brief   This function:
           - Enables GPIO clock
           - Configures the UART pins on GPIO PA2 PA3
 * Param   None
 * Retval  None
 */
inline void ConfigureGPIO_UART1(void)
{
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // GPIO configuration for UART signals
    // 1. Select AF mode (10) on PA9 and PA10
    // 2. AF1 for UART signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
            | (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2))
            | (1 << (1 * 4)) | (1 << (2 * 4));
}

/**
 * Brief   This function configures the USART1:
 *         - asynchronous mode
 *         - 9600,8,n,1
 * Param   None
 * Retval  None
 */
inline void Configure_UART1(void)
{
    // Enable the peripheral clock UART
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure UART
    // 1. Oversampling by 16, 9600 baud
    // 2. 8 data bit, 1 start bit, 1 stop bit, no parity,
    //     transmitter enable, receiver enable
//    USART1->BRR = 480000 / 96;
    USART1->BRR = 480000 / 96;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE;

    // 3. Polling idle frame Transmission
    while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
    {
        /* Add time out here for a robust application */
    }

    // Configure IT
    // 4. Set priority for UART1_IRQn
    // 5. Enable UART1_IRQn
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);
}

char ReceiveByte_CNI(void)
{
    uint32_t tmptail;

    // Wait for incoming data
    while (UART1_RxHead == UART1_RxTail){;}

    // Calculate buffer index
    tmptail = (UART1_RxTail + 1) & UART1_RX_BUFFER_MASK;

    // Store new index
    UART1_RxTail = tmptail;

    // Return data
    return UART1_RxBuf[tmptail];
}

void TransmitByte_CNI(char data)
{
    uint32_t tmphead;

    // Calculate buffer index
    tmphead = (UART1_TxHead + 1) & UART1_TX_BUFFER_MASK;

    // Wait for free space in buffer
    while (tmphead == UART1_TxTail)
    {
        ;
    }

    // Store data in buffer
    UART1_TxBuf[tmphead] = data;

    // Store new index
    UART1_TxHead = tmphead;

    // Initiate a new transfer if this is not ongoing
    if((USART1->CR1 & USART_CR1_TCIE) == 0)
    {
        // Calculate buffer index
        uint32_t tmptail = (UART1_TxTail + 1) & UART1_TX_BUFFER_MASK;

        // Store new index
        UART1_TxTail = tmptail;

        // Start transmission
        USART1->TDR = UART1_TxBuf[tmptail];

        // enable TC interrupt
        USART1->CR1 |= USART_CR1_TCIE;
    }
}

/*
 * This function returns the number of unread bytes in the receive buffer
 */
unsigned char nUnreadBytes_CNI(void)
{
    if(UART1_RxHead == UART1_RxTail)
        return 0;
    else if(UART1_RxHead > UART1_RxTail)
        return (UART1_RxHead - UART1_RxTail);
    else
        return (UART1_RX_BUFFER_SIZE - UART1_RxTail + UART1_RxHead);
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/

/*
 * Brief   This function handles UART interrupt request.
 * Param   None
 * Retval  None
 */
void USART1_IRQHandler(void)
{
    if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
    {
        uint32_t tmptail;

        // Check if all data is transmitted
        if (UART1_TxHead != UART1_TxTail)
        {
            // Calculate buffer index
            tmptail = (UART1_TxTail + 1) & UART1_TX_BUFFER_MASK;

            // Store new index
            UART1_TxTail = tmptail;

            // Start transmission
            USART1->TDR = UART1_TxBuf[tmptail];
        }
        else
        {
            USART1->ICR |= USART_ICR_TCCF; // Clear TC flag
            USART1->CR1 &= ~USART_CR1_TCIE; // Disable TC interrupt
        }
    }
    else if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
    {
        uint8_t data = (uint8_t)(USART1->RDR); // Receive data, clear flag

        uint8_t tmphead = (UART1_RxHead + 1) & UART1_RX_BUFFER_MASK;

        UART1_RxHead = tmphead;

        if (tmphead == UART1_RxTail)
        {
            // ERROR! Receive buffer overflow
        }

        // Store received data in buffer
        UART1_RxBuf[tmphead] = data;
    }
    else if ((USART1->ISR & USART_ISR_ORE) == USART_ISR_ORE)
    {
    	// Ignore the data
    	USART1->ICR |= USART_ICR_ORECF; // Clear ORE flag
    }
    else
    {
        NVIC_DisableIRQ(USART1_IRQn); // Disable UART_IRQn
    }
}
