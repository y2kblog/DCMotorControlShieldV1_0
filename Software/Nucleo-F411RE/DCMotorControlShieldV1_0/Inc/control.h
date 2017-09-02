/**
 ******************************************************************************
 * @file    control.h
 * @brief   Header file of control functions
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/**************** System parameters ***************/
#define dt_minor    0.000050f       ///< Sampling time of minor loop [sec]
#define dt_major    0.000200f       ///< Sampling time of major loop [sec]
#define DIVERGENCE_THRESHOLD_MS 300 ///< Threshold time to detect divergence [msec]
/**************************************************/

/********** Hardware-specific parameters **********/
#define Ktn         0.001159f       ///< Nominal torque constant of motor (Mabuchi FA-130RA-2270) [Nm/A]
#define Mn          0.0000005f      ///< Nominal Inertia [Nm/s^2*rad]
#define Rn          0.6818f         ///< Nominal resistance (Mabuchi FA-130RA-2270) [Ohm]
#define Ln          0.000340f       ///< Nominal inductance (Mabuchi FA-130RA-2270) [H]
/**************************************************/

/************ Default constol parameters *************/
// Position control gains
#define Kp_p_DEFAULT    4900.0f     ///< Proportional gain of position control [s^2]
#define Ki_p_DEFAULT    6000.0f     ///< Integral     gain of position control [s^3]
#define Kd_p_DEFAULT    140.0f      ///< Differential gain of position control [s]

// Velocity control gains
#define Kp_v_DEFAULT    200.0f      ///< Proportional gain of velocity control [s]
#define Ki_v_DEFAULT    10000.0f    ///< Integral     gain of velocity control [s^2]

// Current control gains
#define Kp_c_DEFAULT    0.5f        ///< Proportional gain of current control [V/A]
#define Ki_c_DEFAULT    10.0f       ///< Integral     gain of current control [sV/A]

// Cutoff frequency
#define Gpd_DEFAULT    1000.0f      ///< Cutoff frequency of pseudo-differential for velocity calculation [rad/sec]
/*****************************************************/

/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
/***************************************************************END OF FILE****/
