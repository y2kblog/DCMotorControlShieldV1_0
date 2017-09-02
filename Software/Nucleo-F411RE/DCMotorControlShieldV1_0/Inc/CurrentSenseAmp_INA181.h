/**
 ******************************************************************************
 * @file    CurrentSenseAmp_INA181.h
 * @brief   Header file of current sensing amplifier INA181
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CURRENTSENSEAMP_INA181_H
#define __CURRENTSENSEAMP_INA181_H

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
float readCurrentResponse(void);

#ifdef __cplusplus
}
#endif

#endif /*__CURRENTSENSEAMP_INA181_H */
/***************************************************************END OF FILE****/
