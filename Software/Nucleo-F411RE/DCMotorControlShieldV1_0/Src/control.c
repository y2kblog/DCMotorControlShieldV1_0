/**
 ******************************************************************************
 * @file    control.c
 * @brief   Source file of control functions
 * @version 1.0
 *
 * @par License
 *      This software is released under the MIT License, see LICENSE.txt.
 * @par ChangeLog
 * - 1.0 : Initial Version
 ******************************************************************************
 */

/**
 * @mainpage DC motor control shield V1.0 Documentation
 * DC motor control shield V1.0 is Nucleo-64 shield which can control position/velocity/torque of brushed DC motor FA-130RA.
 *
 * @section main_sec Key functions
 * Most users only need to refer or change the following functions (these are related to motor control performance or output data).
 *
 *  - static void MajorControlLoop(void)\n
 *    Major control loop function (Period : 200[us])\n
 *    Users can change control mode (Position/Velociry/Torque), command or gains.\n
 *    Also, users can change current control gains.\n
 *    \n
 *  - static void MinorControlLoop(void)\n
 *    Minor control loop function (Period : 50[us])\n
 *    \n
 *  - static void PositionControl(float PosCmd, float VelCmd, float P_Gain, float I_Gain, float D_Gain)\n\n
 *  - static void VelocityControl(float VelCmd, float P_Gain, float I_Gain)\n\n
 *  - static void TorqueControl(float Command)\n\n
 *  - static void configCurrentControl(bool isEnabled, float P_Gain, float I_Gain)\n
 *    \n
 *  - void SerialCommunicationTask(void const *argument)\n
 *    Low priority task that communicates with UART\n
 *    Users can send parameters such as command, response, and gains to the PC via ST-Link on Nucleo.\n
 *    User can display or save these parameters in real time by using a terminal
 *     (e.g. [Tera Term](https://osdn.net/projects/ttssh2/)) or a serial plotter (e.g. [Arduino IDE](https://www.arduino.cc/en/Main/Software), [CPLT](http://www.datatecno.co.jp/cplt/cplt-download.htm)).
 */

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Include user header files -------------------------------------------------*/
#include "control.h"
#include "stm32f4xx_ll_gpio.h"
#include "adc.h"
#include "CurrentSenseAmp_INA181.h"
#include "RotaryEncoder_AS5600.h"
#include "MotorDriver_TB6612.h"
//#include "DOB.h" // Disturbance observer (Not implemented)

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

/* Imported variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

/* Private function macro ----------------------------------------------------*/
#define enableControl()  (isEnabled_Control = true)
#define disableControl() (isEnabled_Control = false)

/* Private macro -------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private types -------------------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/**
 * @enum ControlMode
 * Control mode
 */
static enum
{
    None_ControlMode = 0,   ///< None (default)
    PositionControlMode,    ///< Position control
    VelocityControlMode,    ///< Velocity control
    TorqueControlMode       ///< Torque control
} ControlMode = None_ControlMode;

/* Private struct/union tag --------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Variables for motion control
static bool isEnabled_Control = true;
static bool isEnabled_CurrentControl = true;
static bool hasDiverged = false;
static float time_sec;
static float PositionCmd, PositionRes, PositionErr, PositionErrInt;
static float VelocityCmd, VelocityRes, VelocityErr, VelocityErrInt, VelocityResInt;
static float TorqueCmd;
static float AccelerationRef, CurrentRef;
static float CurrentCmd, CurrentRes, CurrentErr, CurrentErrInt;
static float VoltageRef;

// Gain
static float Kp_p = Kp_p_DEFAULT, Ki_p = Ki_p_DEFAULT, Kd_p = Kd_p_DEFAULT;
static float Kp_v = Kp_v_DEFAULT, Ki_v = Ki_v_DEFAULT;
static float Kp_c = Kp_c_DEFAULT, Ki_c = Ki_c_DEFAULT;
static float Gpd  = Gpd_DEFAULT;

static bool isSvonSwOn = false, isSvonSwOn_prev = false;
static bool isSysBtnPushed = false, isSysBtnPushed_prev = false;

static bool needsOutputInfo = false;

/* Private function prototypes -----------------------------------------------*/
// Control variables
static void resetControlVariables(void);

// Control loop
static inline void MajorControlLoop(void);
static inline void MinorControlLoop(void);

// Control
static inline void PositionControl(float, float, float, float, float);
static inline void VelocityControl(float, float, float);
static inline void TorqueControl(float);

static inline void configCurrentControl(bool, float, float);
static inline bool validateDivergence(void);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief       Low priority task that communicates with UART
 * @param       argument Task parameters
*/
void SerialCommunicationTask(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t DelayTime_ms = 50;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, DelayTime_ms * 5);

        if (isEnabled_Control) {
            // Continuous information output
            switch (ControlMode) {
                case PositionControlMode:
                    printf("%.4f,%.4f\r\n", PositionCmd, PositionRes);
                    break;
                case VelocityControlMode:
                    printf("%.4f,%.4f\r\n", VelocityCmd, VelocityRes);
                    break;
                case TorqueControlMode:
                    break;
                default:
                    break;
            }
        }

        if (needsOutputInfo) {
            // Output info when SVON switch is off and Sys button is pushed
            printf("Info:");
            switch (ControlMode) {
                case PositionControlMode:
                    printf("P:%g,I:%g,D:%g", Kp_p, Ki_p, Kd_p);
                    break;
                case VelocityControlMode:
                    printf("P:%g,I:%g", Kp_v, Ki_v);
                    break;
                case TorqueControlMode:
                    break;
                default:
                    break;
            }
            printf("\r\n");
            needsOutputInfo = false;
        }
    }
}

/**
 * @brief       High priority task that executes major loop control sequence
 * @param       argument Task parameters
*/
void MajorLoopTask(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Initialization
    initEncoder();
    if (LL_GPIO_IsInputPinSet(SVON_GPIO_Port, SVON_Pin))
        isSvonSwOn = isSvonSwOn_prev = true;
    else
        isSvonSwOn = isSvonSwOn_prev = false;

    if (LL_GPIO_IsInputPinSet(SysPush_GPIO_Port, SysPush_Pin))
        isSysBtnPushed = isSysBtnPushed_prev = false;
    else
        isSysBtnPushed = isSysBtnPushed_prev = true;

    resetControlVariables();
    configCurrentControl(true, Kp_c_DEFAULT, Ki_c_DEFAULT);
    setPositionResponse(0.0f, &VelocityResInt);
    enableControl();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, 4);

        /***** "SVON" Switch *****/
        if (LL_GPIO_IsInputPinSet(SVON_GPIO_Port, SVON_Pin))
            isSvonSwOn = true;
        else
            isSvonSwOn = false;
        if(isSvonSwOn)
            enableControl();
        else
            disableControl();
        // OFF -> ON
        if (!isSvonSwOn_prev && isSvonSwOn) {
            resetControlVariables();
        }
        isSvonSwOn_prev = isSvonSwOn;


        /***** "Sys" push button *****/
        if (LL_GPIO_IsInputPinSet(SysPush_GPIO_Port, SysPush_Pin))
            isSysBtnPushed = false;
        else
            isSysBtnPushed = true;
        // OFF -> ON
        if (!isSysBtnPushed_prev && isSysBtnPushed) {
            if (isSvonSwOn) {
                if (hasDiverged) {
                    // reset divergence flag
                    resetControlVariables();
                    setPositionResponse(0.0f, &VelocityResInt);
                    hasDiverged = false;
                    enableControl();
                }
            } else {
                needsOutputInfo = true;
            }
        }
        isSysBtnPushed_prev = isSysBtnPushed;


        if (isEnabled_Control) {
            // check Divergence
            if (!hasDiverged) {
                hasDiverged = validateDivergence();
            }
        }

        if (hasDiverged) {
            LL_GPIO_SetOutputPin(SysLED_GPIO_Port, SysLED_Pin);
            disableControl();
        } else {
            LL_GPIO_ResetOutputPin(SysLED_GPIO_Port, SysLED_Pin);
        }

        if (!isEnabled_Control) {
            stopMotor();
            continue;
        }

        MajorControlLoop();
    }
}

/**
 * @brief       Realtime task that executes minor loop control sequence
 * @param       argument Task parameters
*/
void MinorLoopTask(void const * argument)
{
    // Initialization
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC1Value, 5) != HAL_OK) {
        Error_Handler();
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint64_t cnt = 0;
    time_sec = 0.0f;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, 1);

        if (isEnabled_Control) {
            time_sec = (float) cnt * dt_minor;
            cnt++;

            MinorControlLoop();
        }
    }
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief       reset All variables for control
*/
static inline void resetControlVariables(void)
{
    PositionErr = 0.0f;
    VelocityErr = 0.0f;
    TorqueCmd = 0.0f;
    AccelerationRef = 0.0f, CurrentRef = 0.0f;
    CurrentErr = 0.0f;

    PositionErrInt = 0.0f;
    VelocityErrInt = 0.0f;
    VelocityResInt = PositionRes;
    CurrentErrInt = 0.0f;
    VoltageRef = 0.0f;

    //resetDOBVariables(); // reset Disturbance observer variables (Not implemented)
}

/**
 * @brief       Major control loop (Period : 200[us])
*/
static inline void MajorControlLoop(void)
{
    // Command
    float fmodf_time = fmodf(time_sec, 2.5f);
    float PosCmd, VelCmd;
    if (fmodf_time < 1.0f) {
        PosCmd = 1.0f * sinf(1.0f * 2.0f * M_PI * fmodf_time);
        VelCmd = 1.0f * 2.0f * M_PI * 1.0f * cosf(1.0f * 2.0f * M_PI * fmodf_time);
    } else if (fmodf_time < 1.5f) {
        PosCmd = 0.0f;
        VelCmd = 0.0f;
    } else if (fmodf_time < 2.0f) {
        PosCmd = 1.0f;
        VelCmd = 0.0f;
    } else {
        PosCmd = 0.0f;
        VelCmd = 0.0f;
    }
    PositionControl(PosCmd * (0.1f+Param1), VelCmd * (0.1f+Param1),
            (0.5f+Param2) * Kp_p_DEFAULT,
            (0.5f+Param3) * Ki_p_DEFAULT,
            (0.5f+Param4) * Kd_p_DEFAULT);
    //PositionControl(PosCmd, VelCmd, Kp_p_DEFAULT, Ki_p_DEFAULT, Kd_p_DEFAULT);
    //VelocityControl(10.0f, Kp_v_DEFAULT, Ki_v_DEFAULT);
    //TorqueControl(0.0002f);


    // Obtain position response
    if (readPositionResponse(&PositionRes)) {
        return;     // Error
    }

    // Velocity response calculation via pseudo-differential
    VelocityResInt += VelocityRes * dt_major;
    VelocityRes = Gpd * (PositionRes - VelocityResInt);

    static const float inv_Mn = 1.0 / Mn;
    // Major loop controller
    switch (ControlMode) {
        case PositionControlMode:
            PositionErr = PositionCmd - PositionRes;
            VelocityErr = VelocityCmd - VelocityRes;
            PositionErrInt += PositionErr * dt_major;
            AccelerationRef = Kp_p * PositionErr + Kd_p * VelocityErr + Ki_p * PositionErrInt;
            break;
        case VelocityControlMode:
            VelocityErr = VelocityCmd - VelocityRes;
            VelocityErrInt += VelocityErr * dt_major;
            AccelerationRef = Kp_v * VelocityErr + Ki_v * VelocityErrInt;
            break;
        case TorqueControlMode:
            AccelerationRef = TorqueCmd * inv_Mn;
            break;
        default:
            break;
    }
    static const float Acceleration2Current = Mn / Ktn;
    CurrentRef = AccelerationRef * Acceleration2Current;

    CurrentCmd = CurrentRef;
    //CurrentCmd = DOB(CurrentRef, VelocityRes); // Execute disturbance observer (Not implemented)

    if (!isEnabled_CurrentControl) {
        VoltageRef = CurrentCmd * Rn;
        setMotorVoltage(VoltageRef);
    }
}

/**
 * @brief       Major control loop (Period : 50[us])
*/
static inline void MinorControlLoop(void)
{
    // read current response
    CurrentRes = readCurrentResponse();

    if(!isEnabled_CurrentControl)
        return;

    // Minor loop controller (PI current control)
    CurrentErr = CurrentCmd - CurrentRes;
    CurrentErrInt += CurrentErr * dt_minor;
    VoltageRef = Kp_c * CurrentErr + Ki_c * CurrentErrInt;

    // Output voltage
    setMotorVoltage(VoltageRef);
}

/**
 * @brief       Set position control mode and config parameters
 * @param[in]   PosCmd Position command
 * @param[in]   VelCmd Velocity command (This value should be a differential value of PosCmd)
 * @param[in]   P_Gain Position proportional gain
 * @param[in]   I_Gain Position integral gain
 * @param[in]   D_Gain Position differential
*/
static inline void PositionControl(float PosCmd, float VelCmd, float P_Gain, float I_Gain, float D_Gain)
{
    ControlMode = PositionControlMode;
    PositionCmd = PosCmd;
    VelocityCmd = VelCmd;
    Kp_p = P_Gain;
    Ki_p = I_Gain;
    Kd_p = D_Gain;
}

/**
 * @brief       Set velocity control mode and config parameters
 * @param[in]   VelCmd Velocity command
 * @param[in]   P_Gain Velocity proportional gain
 * @param[in]   I_Gain Velocity integral gain
*/
static inline void VelocityControl(float VelCmd, float P_Gain, float I_Gain)
{
    ControlMode = VelocityControlMode;
    VelocityCmd = VelCmd;
    Kp_v = P_Gain;
    Ki_v = I_Gain;
}

/**
 * @brief       Set torque control mode and config parameters
 * @param[in]   Command Torque command
*/
static inline void TorqueControl(float Command)
{
    ControlMode = TorqueControlMode;
    TorqueCmd = Command;
}

/**
 * @brief       Config current control
 * @param[in]   isEnabled Enable or disable current control (true : enable, false : disable)
 * @param[in]   P_Gain Current proportional gain
 * @param[in]   I_Gain Current integral gain
*/
static inline void configCurrentControl(bool isEnabled, float P_Gain, float I_Gain)
{
    isEnabled_CurrentControl = isEnabled;
    if (isEnabled_CurrentControl) {
        Kp_c = P_Gain;
        Ki_c = I_Gain;
    }
}

/**
 * @brief       Validate if control state is stable
 * @param[in]   isEnabled Enable or disable current control (true : enable, false : disable)
 * @param[in]   P_Gain Current proportional gain
 * @param[in]   I_Gain Current integral gain
 * @retval      true : NG (Control has diverged)
 * @retval      false : OK
*/
static inline bool validateDivergence(void)
{
    static bool isSaturated = false;
    static uint32_t SaturatedTimeCount = 0;

    if ((VoltageRef > Vm) || (VoltageRef < -Vm)) {
        isSaturated = true;
    } else {
        isSaturated = false;
    }

    if (isSaturated) {
        SaturatedTimeCount++;
    } else {
        SaturatedTimeCount = 0;
    }

    if (SaturatedTimeCount >= (DIVERGENCE_THRESHOLD_MS * 5)) {
        SaturatedTimeCount = 0;
        return true;
    }
    return false;
}

/***************************************************************END OF FILE****/
