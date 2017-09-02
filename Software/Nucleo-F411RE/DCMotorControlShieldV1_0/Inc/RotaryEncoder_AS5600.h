/**
 ******************************************************************************
 * @file    RotaryEncoder_AS5600.h
 * @brief   Header file of 12-Bit programmable contactless potentiometer AS5600
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ROTARYENCODER_AS5600_H
#define __ROTARYENCODER_AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void initEncoder(void);
int readPositionResponse(float*);
void setPositionResponse(float, float*);

#ifdef __cplusplus
}
#endif

#endif /*__ROTARYENCODER_AS5600_H */
/***************************************************************END OF FILE****/
