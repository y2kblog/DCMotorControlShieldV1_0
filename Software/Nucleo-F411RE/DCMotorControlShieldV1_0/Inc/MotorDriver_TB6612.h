/**
 ******************************************************************************
 * @file    MotorDriver_TB6612.h
 * @brief   Header file of Motor driver IC TB6612
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORDRIVER_TB6612_H
#define __MOTORDRIVER_TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/********** Hardware-specific parameters **********/
#define Vm          4.6f        ///< Applied voltage of motor (5V - <Forward voltage of diode>) [V]

/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void setMotorVoltage(float);
void stopMotor(void);

#ifdef __cplusplus
}
#endif

#endif /*__MOTORDRIVER_TB6612_H */
/***************************************************************END OF FILE****/
