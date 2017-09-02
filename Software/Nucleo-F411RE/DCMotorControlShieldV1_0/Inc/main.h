/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#ifdef __cplusplus
extern "C" {
#endif
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define TIM_CLOCK_SOURCE_HZ 100000000

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Param4_Pin GPIO_PIN_1
#define Param4_GPIO_Port GPIOC
#define Current_Pin GPIO_PIN_0
#define Current_GPIO_Port GPIOA
#define Param1_Pin GPIO_PIN_1
#define Param1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Param2_Pin GPIO_PIN_4
#define Param2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Param3_Pin GPIO_PIN_0
#define Param3_GPIO_Port GPIOB
#define EncErr_Pin GPIO_PIN_10
#define EncErr_GPIO_Port GPIOB
#define PWM_Pin GPIO_PIN_7
#define PWM_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_9
#define AIN2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SysLED_Pin GPIO_PIN_4
#define SysLED_GPIO_Port GPIOB
#define SVON_Pin GPIO_PIN_5
#define SVON_GPIO_Port GPIOB
#define SysPush_Pin GPIO_PIN_6
#define SysPush_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FreeRTOS_PERIOD_HZ      20000

#ifdef TIM_CLOCK_SOURCE_HZ
#undef TIM_CLOCK_SOURCE_HZ
#define TIM_CLOCK_SOURCE_HZ (SystemCoreClock/1)
#endif

extern void _Error_Handler(char *, int);
extern void MX_FREERTOS_Init(void);

#ifdef __cplusplus
}
#endif
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
