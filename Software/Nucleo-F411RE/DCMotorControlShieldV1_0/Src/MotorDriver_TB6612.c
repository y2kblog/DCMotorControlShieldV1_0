/**
 ******************************************************************************
 * @file    MotorDriver_TB6612.c
 * @brief   Source file of Motor driver IC TB6612
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdint.h>

/* Include user header files -------------------------------------------------*/
#include "stm32f4xx_ll_gpio.h"
#include "MotorDriver_TB6612.h"
#include "tim.h"
#include "main.h"

/* Private function macro ----------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/********** Hardware-specific parameters **********/
#define TB6612_htim             htim3
#define TB6612_PWM_CH           TIM_CHANNEL_2
#define TB6612_AIN1_GPIOPort    GPIOA
#define TB6612_AIN1_GPIOPinMask LL_GPIO_PIN_8
#define TB6612_AIN2_GPIOPort    GPIOA
#define TB6612_AIN2_GPIOPinMask LL_GPIO_PIN_9

/* Imported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef TB6612_htim;

/* Private types -------------------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/* Private struct/union tag --------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static inline void stopPWM(void);
static inline void setPWMduty(float);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief       Set voltage to motor
 * @param[in]   V Motor voltage
*/
inline void setMotorVoltage(float V)
{
    float Vout;

    if (V > 0.0f) {
        // CW(CCW)
        Vout = V;
        LL_GPIO_SetOutputPin(TB6612_AIN1_GPIOPort, TB6612_AIN1_GPIOPinMask);
        LL_GPIO_ResetOutputPin(TB6612_AIN2_GPIOPort, TB6612_AIN2_GPIOPinMask);
    } else {
        // CCW(CW)
        Vout = -V;
        LL_GPIO_ResetOutputPin(TB6612_AIN1_GPIOPort, TB6612_AIN1_GPIOPinMask);
        LL_GPIO_SetOutputPin(TB6612_AIN2_GPIOPort, TB6612_AIN2_GPIOPinMask);
    }

    if (Vout > Vm)
        Vout = Vm;
    setPWMduty(Vout / Vm);
}

/**
 * @brief       Stop motor
*/
inline void stopMotor(void)
{
    // Short brake
    LL_GPIO_SetOutputPin(TB6612_AIN1_GPIOPort, TB6612_AIN1_GPIOPinMask);
    LL_GPIO_SetOutputPin(TB6612_AIN2_GPIOPort, TB6612_AIN2_GPIOPinMask);
    stopPWM();
}
/***** Interrupt function prototypes *****/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief       Stop PWM
*/
static inline void stopPWM(void)
{
    HAL_StatusTypeDef status;
    status = HAL_TIM_PWM_Stop(&TB6612_htim, TB6612_PWM_CH);
    if (status != HAL_OK) {
        printf("HAL_TIM_PWM_Stop error : %d\r\n", status);
    }
}

/**
 * @brief       Set PWM duty
 * @param[in]   PWM ratio (0.0 ~ 1.0)
*/
static inline void setPWMduty(float ratio)
{
    if(ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = ratio * (TB6612_htim.Init.Period + 1) - 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_StatusTypeDef status;
    status = HAL_TIM_PWM_ConfigChannel(&TB6612_htim, &sConfigOC, TB6612_PWM_CH);
    if (status != HAL_OK) {
        printf("HAL_TIM_PWM_ConfigChannel error : %d\r\n", status);
    }
    status = HAL_TIM_PWM_Start(&TB6612_htim, TB6612_PWM_CH);
    if (status != HAL_OK) {
        printf("HAL_TIM_PWM_Start error : %d\r\n", status);
    }
}

/***************************************************************END OF FILE****/
