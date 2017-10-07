/*-----------------------------------------------------------------------------
 * Name:    Motor_32F401Discovery.c
 * Purpose: Motor interface for STM32F429 Discovery Kit
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
#include "MachineControlPCB_Motor.h"

/* GPIO Pin identifier */
typedef struct _GPIO_PIN {
  GPIO_TypeDef *port;
  uint16_t      pin;
} GPIO_PIN;

/* Motor GPIO Pins */
static const GPIO_PIN ActuatorReset_PIN[] = {
  { GPIOG, GPIO_PIN_10}
};

static const GPIO_PIN Motor_PIN[] = {
  { GPIOH, GPIO_PIN_10},
  { GPIOH, GPIO_PIN_11}
};

#define Motor_COUNT (sizeof(Motor_PIN)/sizeof(GPIO_PIN))


/**
  \fn          int32_t Motor_Initialize (void)
  \brief       Initialize Motors
  \returns
   - \b  0: function succeeded
   - \b -1: function faiMotor
*/
int32_t Motor_Initialize (void) {
  GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct2;

  /* GPIO Ports Clock Enable */
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();

  /* Configure GPIO pins: PG13 PG14 */
  GPIO_InitStruct.Pin   = GPIO_PIN_11 | GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	GPIO_InitStruct2.Pin   = GPIO_PIN_10;
  GPIO_InitStruct2.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct2.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct2);
	
  return 0;
}


/**
  \fn          int32_t Motor_Uninitialize (void)
  \brief       De-initialize Motors
  \returns
   - \b  0: function succeeded
   - \b -1: function faiMotor
*/
int32_t Motor_Uninitialize (void) {

  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_11 | GPIO_PIN_10);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_10);

  return 0;
}

int32_t ActuatorReset_Off (uint32_t num) {
  HAL_GPIO_WritePin(ActuatorReset_PIN[num].port, ActuatorReset_PIN[num].pin, GPIO_PIN_SET);
  return 0;
}

int32_t ActuatorReset_On (uint32_t num) {
  HAL_GPIO_WritePin(ActuatorReset_PIN[num].port, ActuatorReset_PIN[num].pin, GPIO_PIN_RESET);
  return 0;
}

/**
  \fn          int32_t Motor_On (uint32_t num)
  \brief       Turn on requested Motor
  \param[in]   num  Motor number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiMotor
*/
int32_t Motor_On (uint32_t num) {
  HAL_GPIO_WritePin(Motor_PIN[num].port, Motor_PIN[num].pin, GPIO_PIN_SET);
  return 0;
}

/**
  \fn          int32_t Motor_Off (uint32_t num)
  \brief       Turn off requested Motor
  \param[in]   num  Motor number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiMotor
*/
int32_t Motor_Off (uint32_t num) {
  HAL_GPIO_WritePin(Motor_PIN[num].port, Motor_PIN[num].pin, GPIO_PIN_RESET);
  return 0;
}

/**
  \fn          int32_t Motor_SetOut (uint32_t val)
  \brief       Write value to Motors
  \param[in]   val  value to be displayed on Motors
  \returns
   - \b  0: function succeeded
   - \b -1: function faiMotor
*/
int32_t Motor_SetOut (uint32_t val) {
  uint32_t n;

  for (n = 0; n < Motor_COUNT; n++) 
	{
    if (val & (1 << n)) 
			Motor_On (n);
    else                
			Motor_Off(n);
  }
  return 0;
}

/**
  \fn          uint32_t Motor_GetCount (void)
  \brief       Get number of Motors
  \return      Number of available Motors
*/
uint32_t Motor_GetCount (void) {
  return Motor_COUNT;
}
