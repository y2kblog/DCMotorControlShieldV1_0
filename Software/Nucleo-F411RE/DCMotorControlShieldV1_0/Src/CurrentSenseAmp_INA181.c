/**
 ******************************************************************************
 * @file    CurrentSenseAmp_INA181.c
 * @brief   Source file of current sensing amplifier INA181
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/
#include "CurrentSenseAmp_INA181.h"
#include "adc.h"

/* Private function macro ----------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/********** Hardware-specific parameters **********/
#define V_OFFSET_DEFAULT    1.8f    ///< Default offset voltage of Vref Pin [V]
#define CUR_AMP_GAIN        20.0f   ///< Current sense amp gain [1]
#define R_SHUNT             0.05f   ///< Shunt resistance [Ohm]

/* Imported variables --------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/* Private struct/union tag --------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float CurrentPinOffsetVoltage = V_OFFSET_DEFAULT; ///< Offset voltage of Vref Pin when motor current is 0 [V]

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief       Read current response
 * @return      Current response
*/
float readCurrentResponse(void)
{
    static float DiffVoltage;
    static const float DiffVoltage2CurrentResponse = 1.0f / CUR_AMP_GAIN / R_SHUNT;

    DiffVoltage = CurrentPinVoltage - CurrentPinOffsetVoltage;
    return ((-1.0f) * DiffVoltage * DiffVoltage2CurrentResponse);
}

/* Private functions ---------------------------------------------------------*/
/***************************************************************END OF FILE****/
