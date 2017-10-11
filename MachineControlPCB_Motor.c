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
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "MachineControlPCB_Motor.h"

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

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
  { GPIOH, GPIO_PIN_9},
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
	GPIO_InitTypeDef GPIO_InitStruct3;
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_OC_InitTypeDef sConfigOC2;
  TIM_OC_InitTypeDef sConfigOC3;
	

  /* GPIO Ports Clock Enable */
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
	
  __HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_TIM12_CLK_ENABLE();
	
  GPIO_InitStruct.Pin   = GPIO_PIN_11 | GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	GPIO_InitStruct2.Pin   = GPIO_PIN_10;
  GPIO_InitStruct2.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct2.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct2);
	
	GPIO_InitStruct3.Pin   = GPIO_PIN_9;
  GPIO_InitStruct3.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct3.Pull  = GPIO_NOPULL;
  GPIO_InitStruct3.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct3.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct3);
	
	htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 8399;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim5);
	
	htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 8399;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim12);
	
/*	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);*/

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.Pulse = 4199;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	
	sConfigOC2.OCMode = TIM_OCMODE_PWM1;
  sConfigOC2.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC2.Pulse = 4199;
  sConfigOC2.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC2.OCFastMode = TIM_OCFAST_ENABLE;
	
  sConfigOC3.OCMode = TIM_OCMODE_PWM1;
  sConfigOC3.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC3.Pulse = 4199;
  sConfigOC3.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC3.OCFastMode = TIM_OCFAST_ENABLE;
	
	HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC2, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC3, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
	
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

  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9);
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
int32_t Motor_Upward (uint32_t num) {
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
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
int32_t Motor_Downward (uint32_t num) {
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
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
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
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
			Motor_Upward (n);
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
