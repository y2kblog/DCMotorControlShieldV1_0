/**
 ******************************************************************************
 * @file    RotaryEncoder_AS5600.c
 * @brief   Source file of 12-Bit programmable contactless potentiometer AS5600
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
#include <stdbool.h>

/* Include user header files -------------------------------------------------*/
#include "RotaryEncoder_AS5600.h"
#include "i2c.h"

/* Private function macro ----------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/********** Hardware-specific parameters **********/
#define AS5600_RESOLUTION_PPR   4096
#define AS5600_DEV_ADDRESS      (0x36<<1)
#define AS5600_REG_ZMCO         0x00
#define AS5600_REG_ZPOS         0x01
#define AS5600_REG_MPOS         0x03
#define AS5600_REG_MANG         0x05
#define AS5600_REG_CONF         0x07
#define AS5600_REG_RAW_ANGLE    0x0C
#define AS5600_REG_ANGLE        0x0E
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE    0x1B
#define AS5600_REG_BURN         0xFF

#define AS5600_hi2c                     hi2c1
#define AS5600_I2C_MemRxCpltCallback    I2C1_MemRxCpltCallback
#define AS5600_I2C_ErrorCallback        I2C1_ErrorCallback

// Timeout
#define AS5600_I2C_TIMEOUT_MS   5000

/* Imported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef AS5600_hi2c;

/* Private types -------------------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/* Private struct/union tag --------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t Encoder_Buff[2];
static volatile bool hasError_I2C = false;
static volatile uint16_t AbsoluteAngleCount, AbsoluteAngleCountPrev;
static volatile int64_t AbsoluteCountSum, AbsoluteCountSum_offset;

// constant variables to reduce calculation time
static const float AbsoluteAngleCount2PositionRes = 2.0f * 3.14159265358979323846f / (float) AS5600_RESOLUTION_PPR;

/* Private function prototypes -----------------------------------------------*/
static inline void updateRawAngleCount(uint16_t*, uint16_t*, int64_t*);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief       Initialize encoder AS5600
*/
void initEncoder(void)
{
    HAL_StatusTypeDef status;
    uint8_t AS5600_status;
AS5600_init_start:
    // Read AS5600 status register
    status = HAL_I2C_Mem_Read(&AS5600_hi2c, AS5600_DEV_ADDRESS, AS5600_REG_STATUS,
            I2C_MEMADD_SIZE_8BIT, &AS5600_status, 1, AS5600_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("HAL_I2C_Mem_Read error\r\n");

        if (HAL_I2C_DeInit(&AS5600_hi2c) != HAL_OK) {
            printf("HAL_I2C_DeInit error\r\n");
        }
        if (HAL_I2C_Init(&AS5600_hi2c) != HAL_OK) {
            printf("HAL_I2C_Init error\r\n");
        }
        goto AS5600_init_start;
    }
    AS5600_status &= 0x38;
    if (AS5600_status != 0x20) {
        HAL_GPIO_WritePin(EncErr_GPIO_Port, EncErr_Pin, GPIO_PIN_SET);
        printf("Magnet error : 0x%X, ", AS5600_status);
        if (!(AS5600_status & 0x20))
            printf("Magnet was not detected\r\n");
        if (AS5600_status & 0x10)
            printf("Magnet too weak\r\n");
        if (AS5600_status & 0x08)
            printf("Magnet too strong\r\n");
        for (;;);
    } else {
        //printf("Magnet : OK\r\n");
    }

    // Set AS5600 configuration (WD = 0b0, FTH = 0b001, SF = 0b11, PM=0b00, HYST=0b00, OUTS=0b00, PWMF=0b00)
    const uint8_t AS5600_CONF[2] = { 0x07, 0x00 };
    status = HAL_I2C_Mem_Read(&AS5600_hi2c, AS5600_DEV_ADDRESS, AS5600_REG_CONF, I2C_MEMADD_SIZE_8BIT,
            (uint8_t*)Encoder_Buff, 2, AS5600_I2C_TIMEOUT_MS);
    assert_param(status == HAL_OK);
    if (status != HAL_OK)
        printf("HAL_I2C_Mem_Read error : %d\r\n", status);
    Encoder_Buff[0] &= 0x3F;
    if ((Encoder_Buff[0] != AS5600_CONF[0]) || (Encoder_Buff[1] != AS5600_CONF[1])) {
        status = HAL_I2C_Mem_Write(&AS5600_hi2c, AS5600_DEV_ADDRESS, AS5600_REG_CONF, I2C_MEMADD_SIZE_8BIT,
                (uint8_t *)AS5600_CONF, 2, AS5600_I2C_TIMEOUT_MS);
        if (status != HAL_OK)
            printf("HAL_I2C_Mem_Write error : %d\r\n", status);
    }

    // Set current position as origin(PositionRes = 0)
    status = HAL_I2C_Mem_Read(&AS5600_hi2c, AS5600_DEV_ADDRESS, AS5600_REG_RAW_ANGLE,
            I2C_MEMADD_SIZE_8BIT, (uint8_t*)Encoder_Buff, 2, AS5600_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
        printf("HAL_I2C_Mem_Read_DMA error : %d\r\n", status);
    AbsoluteAngleCount = (uint16_t) Encoder_Buff[0] << 8 | (uint16_t) Encoder_Buff[1];
    AbsoluteAngleCount &= 0x0FFF;
    updateRawAngleCount((uint16_t *)&AbsoluteAngleCount, (uint16_t*)&AbsoluteAngleCountPrev, (int64_t*)&AbsoluteCountSum);
    AbsoluteCountSum_offset = AbsoluteCountSum;
}


/**
 * @brief       Read position response
 * @param[in,out] pPosRes Pointer of position response
 * @retval      0 Success to read, position response is stored to pPosRes
 * @retval      otherwise Failed to read
*/
int readPositionResponse(float* pPosRes)
{
    static float PositionRes_buf;
    PositionRes_buf = AbsoluteAngleCount2PositionRes * (float) (AbsoluteCountSum - AbsoluteCountSum_offset);

    // Preparation for reading the position response in the next control loop
    if (hasError_I2C) {
        printf("I2C Error CallBack:0x%X,0x%X\r\n",
                (unsigned int)HAL_I2C_GetState(&AS5600_hi2c), (unsigned int)HAL_I2C_GetError(&AS5600_hi2c));
        // reset I2C bus
        if (HAL_I2C_DeInit(&AS5600_hi2c) != HAL_OK) {
            printf("HAL_I2C_DeInit error\r\n");
        }
        if (HAL_I2C_Init(&AS5600_hi2c) != HAL_OK) {
            printf("HAL_I2C_Init error\r\n");
        }
        hasError_I2C = false;
        return 1;
    } else {
        static HAL_StatusTypeDef status;
        if ((HAL_I2C_GetState(&AS5600_hi2c) == HAL_I2C_STATE_READY) && (HAL_I2C_GetError(&AS5600_hi2c) == HAL_I2C_ERROR_NONE)) {
            status = HAL_I2C_Mem_Read_DMA(&AS5600_hi2c, AS5600_DEV_ADDRESS, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT,
                    (uint8_t *)Encoder_Buff, 2);
            if (status != HAL_OK) {
                return 2;
            }
        } else {
            return 3;
        }
    }
    *pPosRes = PositionRes_buf;
    return 0;
}

/**
 * @brief       Set the present motor position response to the desired value
 * @param[in]   Position Desired position response
 * @param[in,out] pVelResInt Pointer of VelocityResInt, which is changed to avoid destabilization
*/
void setPositionResponse(float Position, float* pVelResInt)
{
    AbsoluteCountSum_offset = AbsoluteCountSum - (int64_t) (Position / AbsoluteAngleCount2PositionRes);
    *pVelResInt = Position;  // To avoid unstable
}

/***** Interrupt function prototypes *****/
/**
 * @brief       When non-blocking mode memory read of AS5600 is completed, this function is called
 * @param[in,out] hi2c Pointer of I2C handler
*/
void AS5600_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AbsoluteAngleCount = (uint16_t) Encoder_Buff[0] << 8 | (uint16_t) Encoder_Buff[1];
    AbsoluteAngleCount &= 0x0FFF;
    updateRawAngleCount((uint16_t *) &AbsoluteAngleCount, (uint16_t*) &AbsoluteAngleCountPrev, (int64_t*) &AbsoluteCountSum);
}

/**
 * @brief       When error is occured, this function is called
 * @param[in,out] hi2c Pointer of I2C handler
*/
void AS5600_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    hasError_I2C = true;
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Get angular value corresponding to multiple rotations from raw angular value of single rotation which is read from encoder
 * @param[in,out] nowCount  Pointer of present raw angle count value
 * @param[in,out] prevCount Pointer of previous raw angle count value
 * @param[in,out] CountSum Pointer of sum count value
*/
static inline void updateRawAngleCount(uint16_t* nowCount, uint16_t* prevCount, int64_t* CountSum)
{
    const uint16_t enc_ppr_Quarter   = AS5600_RESOLUTION_PPR / 4;
    const uint16_t enc_ppr_3Quarters = AS5600_RESOLUTION_PPR * 3 / 4;

    if ((*prevCount > enc_ppr_3Quarters) && (*nowCount < enc_ppr_Quarter))
        *CountSum -= ((int32_t) *nowCount - (int32_t) *prevCount) + (int32_t) AS5600_RESOLUTION_PPR;
    else if ((*prevCount < enc_ppr_Quarter) && (*nowCount > enc_ppr_3Quarters))
        *CountSum -= ((int32_t) *nowCount - (int32_t) *prevCount) - (int32_t) AS5600_RESOLUTION_PPR;
    else
        *CountSum -= ((int32_t) *nowCount - (int32_t) *prevCount);

    *prevCount = *nowCount;
}


/***************************************************************END OF FILE****/
