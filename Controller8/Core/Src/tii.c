/**
  ******************************************************************************
  * File    tii.c
  * Author  HAN University of Applied Sciences
  * Version V1.0
  * Date    23 January 2019
  * Brief   TII functions and definitions
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

#include "tii.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Static Variables */
static char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile char UART_RxHead;
static volatile char UART_RxTail;

static char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile char UART_TxHead;
static volatile char UART_TxTail;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * Brief   This function:
           - Enables GPIO clock
           - Configures the UART pins on GPIO PA2 PA3
 * Param   None
 * Retval  None
 */
inline void ConfigureGPIO_TII(void)
{
    // Enable the peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // GPIO configuration for UART signals
    // 1. Select AF mode (10) on PA2 and PA3
    // 2. AF1 for UART signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3))
            | (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3))
            | (1 << (2 * 4)) | (1 << (3 * 4));
}

/**
 * Brief   This function configures the USART2:
 *         - asynchronous mode
 *         - 9600,8,n,1
 * Param   None
 * Retval  None
 */
inline void Configure_TII(void)
{
    // Enable the peripheral clock UART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure UART
    // 1. Oversampling by 16, 9600 baud
    // 2. 8 data bit, 1 start bit, 1 stop bit, no parity,
    //     transmitter enable, receiver enable
    USART2->BRR = 833;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE;

    // 3. Polling idle frame Transmission
    while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC)
    {
        /* Add time out here for a robust application */
    }

    // Configure IT
    // 4. Set priority for UART_IRQn
    // 5. Enable UART_IRQn
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
}

char ReceiveByte_TII(void)
{
    uint32_t tmptail;

    // Wait for incoming data
    while (UART_RxHead == UART_RxTail){;}

    // Calculate buffer index
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;

    // Store new index
    UART_RxTail = tmptail;

    // Return data
    return UART_RxBuf[tmptail];
}

void TransmitByte_TII(char data)
{
    uint32_t tmphead;

    // Calculate buffer index
    tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    // Wait for free space in buffer
    while (tmphead == UART_TxTail)
    {
        ;
    }

    // Store data in buffer
    UART_TxBuf[tmphead] = data;

    // Store new index
    UART_TxHead = tmphead;

    // Initiate a new transfer if this is not ongoing
    if((USART2->CR1 & USART_CR1_TCIE) == 0)
    {
        // Calculate buffer index
        uint32_t tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;

        // Store new index
        UART_TxTail = tmptail;

        // Start transmission
        USART2->TDR = UART_TxBuf[tmptail];

        // enable TC interrupt
        USART2->CR1 |= USART_CR1_TCIE;
    }
}

/*
 * This function returns the number of unread bytes in the receive buffer
 */
unsigned char nUnreadBytes_TII(void)
{
    if(UART_RxHead == UART_RxTail)
        return 0;
    else if(UART_RxHead > UART_RxTail)
        return (UART_RxHead - UART_RxTail);
    else
        return (UART_RX_BUFFER_SIZE - UART_RxTail + UART_RxHead);
}

/*
 * This function gets a string of characters from the USART.
 * The string is placed in the array pointed to by str.
 *
 * - This function uses the function ReceiveByte() to get a byte
 *   from the UART.
 * - If the received byte is equal to '\r' (Control),
 *   the function returns. The '\r' is removed from the string.
 * - The array is terminated with '\0'.
 */
void ReceiveString_TII(char *str)
{
    uint8_t t = 0;

    while ((str[t] = ReceiveByte_TII()) != '\r')
    {
        t++;
    }
    str[t] = '\0';
}

/*
 * Transmits a string of characters to the USART.
 * The string must be terminated with '\0'.
 *
 * - This function uses the function TransmitByte() to
 *   transmit a byte via the UART
 * - Bytes are transmitted until the terminator
 *   character '\0' is detected. Then the function returns.
 */
void TransmitString_TII(char *str)
{
    while(*str)
    {
        TransmitByte_TII(*str++);
    }
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
void USART2_IRQHandler(void)
{
    if ((USART2->ISR & USART_ISR_TC) == USART_ISR_TC)
    {
        uint32_t tmptail;

        // Check if all data is transmitted
        if (UART_TxHead != UART_TxTail)
        {
            // Calculate buffer index
            tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;

            // Store new index
            UART_TxTail = tmptail;

            // Start transmission
            USART2->TDR = UART_TxBuf[tmptail];
        }
        else
        {
            USART2->ICR |= USART_ICR_TCCF; // Clear TC flag
            USART2->CR1 &= ~USART_CR1_TCIE; // Disable TC interrupt
        }
    }
    else if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
    {
        uint8_t data = (uint8_t)(USART2->RDR); // Receive data, clear flag

        uint8_t tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;

        UART_RxHead = tmphead;

        if (tmphead == UART_RxTail)
        {
            // ERROR! Receive buffer overflow
        }

        // Store received data in buffer
        UART_RxBuf[tmphead] = data;
    }
    else if ((USART2->ISR & USART_ISR_ORE) == USART_ISR_ORE)
    {
    	// Ignore the data
    	USART2->ICR |= USART_ICR_ORECF; // Clear ORE flag
    }
    else
    {
        NVIC_DisableIRQ(USART2_IRQn); // Disable UART_IRQn
    }
}
