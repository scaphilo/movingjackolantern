/*-----------------------------------------------------------------------------
 * Name:    Buttons_32F401Discovery.c
 * Purpose: Buttons interface for STM32F429 Discovery Kit
 * Rev.:    1.00
 *----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "MachineControlPCB_Buttons.h"

/* GPIO Pin identifier */
typedef struct _GPIO_PIN {
  GPIO_TypeDef *port;
  uint16_t      pin;
} GPIO_PIN;

/* Buttons GPIO Pins */
static const GPIO_PIN Buttons_PIN[] = {
  { GPIOH, GPIO_PIN_4},
  { GPIOH, GPIO_PIN_8}
};

#define Buttons_COUNT (sizeof(Buttons_PIN)/sizeof(GPIO_PIN))


/**
  \fn          int32_t Buttons_Initialize (void)
  \brief       Initialize Buttonss
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
int32_t Buttons_Initialize (void) {
	GPIO_InitTypeDef GPIO_InitStruct1;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();

  /* Configure GPIO pins: PH4 PH8 */
  GPIO_InitStruct1.Pin   = GPIO_PIN_4 | GPIO_PIN_8;
  GPIO_InitStruct1.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct1.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct1.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct1);
	
  return 0;
}

/**
  \fn          int32_t Buttons_Uninitialize (void)
  \brief       De-initialize Buttonss
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
int32_t Buttons_Uninitialize (void) {

  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_4 | GPIO_PIN_8);

  return 0;
}

/**
  \fn          int32_t Buttons_On (uint32_t num)
  \brief       Turn on requested Buttons
  \param[in]   num  Buttons number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
GPIO_PinState Buttons_Status (uint32_t num) {
  return HAL_GPIO_ReadPin(Buttons_PIN[num].port, Buttons_PIN[num].pin);
}

/**
  \fn          int32_t Buttons_Off (uint32_t num)
  \brief       Turn off requested Buttons
  \param[in]   num  Buttons number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/

