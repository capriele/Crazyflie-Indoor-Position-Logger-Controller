/**
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bmi160_defs.h
 * @date	Dec 1, 2016
 * @version 3.0.0
 * @brief
 *
 */

/*!
 * @defgroup bmi160_defs
 * @brief
 * @{*/

#ifndef BMI160_DEFS_H_
#define BMI160_DEFS_H_

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*************************** Common macros   *****************************/
#if (LONG_MAX) > 0x7fffffff
#define __have_long64 1
#elif (LONG_MAX) == 0x7fffffff
#define __have_long32 1
#endif

#if !defined(UINT8_C)
#define INT8_C(x)       x
#if (INT_MAX) > 0x7f
#define UINT8_C(x)      x
#else
#define UINT8_C(x)      x##U
#endif
#endif

#if !defined(UINT16_C)
#define INT16_C(x)      x
#if (INT_MAX) > 0x7fff
#define UINT16_C(x)     x
#else
#define UINT16_C(x)     x##U
#endif
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#if __have_long32
#define INT32_C(x)      x##L
#define UINT32_C(x)     x##UL
#else
#define INT32_C(x)      x
#define UINT32_C(x)     x##U
#endif
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#if __have_long64
#define INT64_C(x)      x##L
#define UINT64_C(x)     x##UL
#else
#define INT64_C(x)      x##LL
#define UINT64_C(x)     x##ULL
#endif
#endif

/*************************** Sensor macros   *****************************/
/* Test for an endian machine */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define LITTLE_ENDIAN   1
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define BIG_ENDIAN   1
#else
#error "Code does not support Endian format of the processor"
#endif


/** Mask definitions */
#define BMI160_ACCEL_BW_MASK                    UINT8_C(0x70)
#define BMI160_ACCEL_ODR_MASK                   UINT8_C(0x0F)
#define BMI160_ACCEL_UNDERSAMPLING_MASK         UINT8_C(0x80)
#define BMI160_ACCEL_RANGE_MASK                 UINT8_C(0x0F)
#define BMI160_GYRO_BW_MASK                     UINT8_C(0x30)
#define BMI160_GYRO_ODR_MASK                    UINT8_C(0x0F)
#define BMI160_GYRO_RANGE_MSK                   UINT8_C(0x07)

/** Mask definitions for INT_EN registers */
#define BMI160_ANY_MOTION_X_INT_EN_MASK         UINT8_C(0x01)
#define BMI160_HIGH_G_X_INT_EN_MASK             UINT8_C(0x01)
#define BMI160_NO_MOTION_X_INT_EN_MASK          UINT8_C(0x01)
#define BMI160_ANY_MOTION_Y_INT_EN_MASK         UINT8_C(0x02)
#define BMI160_HIGH_G_Y_INT_EN_MASK             UINT8_C(0x02)
#define BMI160_NO_MOTION_Y_INT_EN_MASK          UINT8_C(0x02)
#define BMI160_ANY_MOTION_Z_INT_EN_MASK         UINT8_C(0x04)
#define BMI160_HIGH_G_Z_INT_EN_MASK             UINT8_C(0x04)
#define BMI160_NO_MOTION_Z_INT_EN_MASK          UINT8_C(0x04)
#define BMI160_SIG_MOTION_INT_EN_MASK           UINT8_C(0x07)
#define BMI160_STEP_DETECT_INT_EN_MASK          UINT8_C(0x08)
#define BMI160_DOUBLE_TAP_INT_EN_MASK           UINT8_C(0x10)
#define BMI160_SINGLE_TAP_INT_EN_MASK           UINT8_C(0x20)
#define BMI160_FIFO_FULL_INT_EN_MASK            UINT8_C(0x20)
#define BMI160_ORIENT_INT_EN_MASK               UINT8_C(0x40)
#define BMI160_FIFO_WATERMARK_INT_EN_MASK       UINT8_C(0x40)
#define BMI160_LOW_G_INT_EN_MASK                UINT8_C(0x08)
#define BMI160_STEP_DETECT_EN_MASK              UINT8_C(0x08)
#define BMI160_FLAT_INT_EN_MASK                 UINT8_C(0x80)
#define BMI160_DATA_RDY_INT_EN_MASK             UINT8_C(0x10)

/** Mask definitions for INT_OUT_CTRL register */
#define BMI160_INT1_EDGE_CTRL_MASK              UINT8_C(0x01)
#define BMI160_INT1_OUTPUT_MODE_MASK            UINT8_C(0x04)
#define BMI160_INT1_OUTPUT_EN_MASK              UINT8_C(0x08)
#define BMI160_INT2_EDGE_CTRL_MASK              UINT8_C(0x10)
#define BMI160_INT2_OUTPUT_MODE_MASK            UINT8_C(0x40)
#define BMI160_INT2_OUTPUT_EN_MASK              UINT8_C(0x80)

/** Mask definitions for INT_LATCH register */
#define BMI160_INT1_INPUT_EN_MASK               UINT8_C(0x10)
#define BMI160_INT2_INPUT_EN_MASK               UINT8_C(0x20)
#define BMI160_INT_LATCH_MASK                   UINT8_C(0x0F)

/** Mask definitions for INT_MAP register */
#define BMI160_INT1_LOW_G_MASK                  UINT8_C(0x01)
#define BMI160_INT1_HIGH_G_MASK                 UINT8_C(0x02)
#define BMI160_INT1_SLOPE_MASK                  UINT8_C(0x04)
#define BMI160_INT1_NO_MOTION_MASK              UINT8_C(0x08)
#define BMI160_INT1_DOUBLE_TAP_MASK             UINT8_C(0x10)
#define BMI160_INT1_SINGLE_TAP_MASK             UINT8_C(0x20)
#define BMI160_INT1_FIFO_FULL_MASK              UINT8_C(0x20)
#define BMI160_INT1_FIFO_WATREMARK_MASK         UINT8_C(0x40)
#define BMI160_INT1_ORIENT_MASK                 UINT8_C(0x40)
#define BMI160_INT1_FLAT_MASK                   UINT8_C(0x80)
#define BMI160_INT1_DATA_READY_MASK             UINT8_C(0x80)
#define BMI160_INT2_LOW_G_MASK                  UINT8_C(0x01)
#define BMI160_INT1_STEP_DETECT_MASK            UINT8_C(0x01)
#define BMI160_INT2_STEP_DETECT_MASK            UINT8_C(0x02)
#define BMI160_INT2_HIGH_G_MASK                 UINT8_C(0x02)
#define BMI160_INT2_FIFO_FULL_MASK              UINT8_C(0x02)
#define BMI160_INT2_FIFO_WATREMARK_MASK         UINT8_C(0x04)
#define BMI160_INT2_SLOPE_MASK                  UINT8_C(0x04)
#define BMI160_INT2_DATA_READY_MASK             UINT8_C(0x08)
#define BMI160_INT2_NO_MOTION_MASK              UINT8_C(0x08)
#define BMI160_INT2_DOUBLE_TAP_MASK             UINT8_C(0x10)
#define BMI160_INT2_SINGLE_TAP_MASK             UINT8_C(0x20)
#define BMI160_INT2_ORIENT_MASK                 UINT8_C(0x40)
#define BMI160_INT2_FLAT_MASK                   UINT8_C(0x80)

/** Mask definitions for INT_DATA register */
#define BMI160_TAP_SRC_INT_MASK                 UINT8_C(0x08)
#define BMI160_LOW_HIGH_SRC_INT_MASK            UINT8_C(0x80)
#define BMI160_MOTION_SRC_INT_MASK              UINT8_C(0x80)

/** Mask definitions for INT_MOTION register */
#define BMI160_SLOPE_INT_DUR_MASK               UINT8_C(0x03)
#define BMI160_NO_MOTION_INT_DUR_MASK           UINT8_C(0xFC)
#define BMI160_NO_MOTION_SEL_BIT_MASK           UINT8_C(0x01)

/** Mask definitions for INT_TAP register */
#define BMI160_TAP_DUR_MASK                     UINT8_C(0x07)
#define BMI160_TAP_SHOCK_DUR_MASK               UINT8_C(0x40)
#define BMI160_TAP_QUIET_DUR_MASK               UINT8_C(0x80)
#define BMI160_TAP_THRES_MASK                   UINT8_C(0x1F)

/** Mask definitions for INT_FLAT register */
#define BMI160_FLAT_THRES_MASK                  UINT8_C(0x3F)
#define BMI160_FLAT_HOLD_TIME_MASK              UINT8_C(0x30)
#define BMI160_FLAT_HYST_MASK                   UINT8_C(0x07)

/** Mask definitions for INT_LOWHIGH register */
#define BMI160_LOW_G_HYST_MASK                  UINT8_C(0x03)
#define BMI160_LOW_G_LOW_MODE_MASK              UINT8_C(0x04)
#define BMI160_HIGH_G_HYST_MASK                 UINT8_C(0xC0)

/** Mask definitions for INT_SIG_MOTION register */
#define BMI160_SIG_MOTION_SEL_MASK              UINT8_C(0x02)
#define BMI160_SIG_MOTION_SKIP_MASK             UINT8_C(0x0C)
#define BMI160_SIG_MOTION_PROOF_MASK            UINT8_C(0x30)

/** Mask definitions for INT_ORIENT register */
#define BMI160_ORIENT_MODE_MASK                 UINT8_C(0x03)
#define BMI160_ORIENT_BLOCK_MASK                UINT8_C(0x0C)
#define BMI160_ORIENT_HYST_MASK                 UINT8_C(0xF0)
#define BMI160_ORIENT_THETA_MASK                UINT8_C(0x3F)
#define BMI160_ORIENT_UD_ENABLE                 UINT8_C(0x40)
#define BMI160_AXES_EN_MASK                     UINT8_C(0x80)

/** Mask definitions for FIFO_CONFIG register */
#define BMI160_FIFO_GYRO_EN_MASK                UINT8_C(0x80)
#define BMI160_FIFO_ACCEL_EN_MASK               UINT8_C(0x40)
#define BMI160_FIFO_MAG_EN_MASK                 UINT8_C(0x20)
#define BMI160_FIFO_TAG_INT1_EN_MASK            UINT8_C(0x08)
#define BMI160_FIFO_TAG_INT2_EN_MASK            UINT8_C(0x04)

/** Mask definitions for STEP_CONF register */
#define BMI160_STEP_COUNT_EN_BIT_MASK           UINT8_C(0x08)
#define BMI160_STEP_DETECT_MIN_THRES_MASK       UINT8_C(0x18)
#define BMI160_STEP_DETECT_STEPTIME_MIN_MASK    UINT8_C(0x07)


/* Enable/disable bit value */
#define BMI160_ENABLE                           UINT8_C(0x01)
#define BMI160_DISABLE                          UINT8_C(0x00)

/* Latch Duration */
#define BMI160_LATCH_DUR_NONE                   UINT8_C(0x00)
#define BMI160_LATCH_DUR_312_5_MICRO_SEC        UINT8_C(0x01)
#define BMI160_LATCH_DUR_625_MICRO_SEC          UINT8_C(0x02)
#define BMI160_LATCH_DUR_1_25_MILLI_SEC         UINT8_C(0x03)
#define BMI160_LATCH_DUR_2_5_MILLI_SEC          UINT8_C(0x04)
#define BMI160_LATCH_DUR_5_MILLI_SEC            UINT8_C(0x05)
#define BMI160_LATCH_DUR_10_MILLI_SEC           UINT8_C(0x06)
#define BMI160_LATCH_DUR_20_MILLI_SEC           UINT8_C(0x07)
#define BMI160_LATCH_DUR_40_MILLI_SEC           UINT8_C(0x08)
#define BMI160_LATCH_DUR_80_MILLI_SEC           UINT8_C(0x09)
#define BMI160_LATCH_DUR_160_MILLI_SEC          UINT8_C(0x0A)
#define BMI160_LATCH_DUR_320_MILLI_SEC          UINT8_C(0x0B)
#define BMI160_LATCH_DUR_640_MILLI_SEC          UINT8_C(0x0C)
#define BMI160_LATCH_DUR_1_28_SEC               UINT8_C(0x0D)
#define BMI160_LATCH_DUR_2_56_SEC               UINT8_C(0x0E)
#define BMI160_LATCHED                          UINT8_C(0x0F)

/** Register map */
#define BMI160_CHIP_ID_ADDR              UINT8_C(0x00)
#define BMI160_ACCEL_CONFIG_ADDR         UINT8_C(0x40) /* Accel Config Address for Output data rate, Undersampling and Bandwidth */
#define BMI160_ACCEL_RANGE_ADDR          UINT8_C(0x41)
#define BMI160_GYRO_CONFIG_ADDR          UINT8_C(0x42) /* Gyro Config Address for Output data rate, Undersampling and Bandwidth */
#define BMI160_GYRO_RANGE_ADDR           UINT8_C(0x43)
#define BMI160_COMMAND_REG_ADDR          UINT8_C(0x7E)
#define BMI160_GYRO_DATA_ADDR            UINT8_C(0x0C)
#define BMI160_ACCEL_DATA_ADDR           UINT8_C(0x12)

#define BMI160_FIFO_CONFIG_0_ADDR        UINT8_C(0x46)
#define BMI160_FIFO_CONFIG_1_ADDR        UINT8_C(0x47)
#define BMI160_INT_ENABLE_0_ADDR         UINT8_C(0x50)
#define BMI160_INT_ENABLE_1_ADDR         UINT8_C(0x51)
#define BMI160_INT_ENABLE_2_ADDR         UINT8_C(0x52)
#define BMI160_INT_OUT_CTRL_ADDR         UINT8_C(0x53)
#define BMI160_INT_LATCH_ADDR            UINT8_C(0x54)
#define BMI160_INT_MAP_0_ADDR            UINT8_C(0x55)
#define BMI160_INT_MAP_1_ADDR            UINT8_C(0x56)
#define BMI160_INT_MAP_2_ADDR            UINT8_C(0x57)
#define BMI160_INT_DATA_0_ADDR           UINT8_C(0x58)
#define BMI160_INT_DATA_1_ADDR           UINT8_C(0x59)
#define BMI160_INT_LOWHIGH_0_ADDR        UINT8_C(0x5A)
#define BMI160_INT_LOWHIGH_1_ADDR        UINT8_C(0x5B)
#define BMI160_INT_LOWHIGH_2_ADDR        UINT8_C(0x5C)
#define BMI160_INT_LOWHIGH_3_ADDR        UINT8_C(0x5D)
#define BMI160_INT_LOWHIGH_4_ADDR        UINT8_C(0x5E)
#define BMI160_INT_MOTION_0_ADDR         UINT8_C(0x5F)
#define BMI160_INT_MOTION_1_ADDR         UINT8_C(0x60)
#define BMI160_INT_MOTION_2_ADDR         UINT8_C(0x61)
#define BMI160_INT_MOTION_3_ADDR         UINT8_C(0x62)
#define BMI160_INT_TAP_0_ADDR            UINT8_C(0x63)
#define BMI160_INT_TAP_1_ADDR            UINT8_C(0x64)
#define BMI160_INT_ORIENT_0_ADDR         UINT8_C(0x65)
#define BMI160_INT_ORIENT_1_ADDR         UINT8_C(0x66)
#define BMI160_INT_FLAT_0_ADDR           UINT8_C(0x67)
#define BMI160_INT_FLAT_1_ADDR           UINT8_C(0x68)
#define BMI160_INT_STEP_CONFIG_0_ADDR    UINT8_C(0x7A)
#define BMI160_INT_STEP_CONFIG_1_ADDR    UINT8_C(0x7B)
#define BMI160_SPI_COMM_TEST_ADDR        UINT8_C(0x7F)

/** Error code definitions */
#define BMI160_OK                        INT8_C(0)

/* Errors */
#define BMI160_E_NULL_PTR                INT8_C(-1)
#define BMI160_E_COM_FAIL                INT8_C(-2)
#define BMI160_E_DEV_NOT_FOUND           INT8_C(-3)
#define BMI160_E_OUT_OF_RANGE            INT8_C(-4)
#define BMI160_E_INVALID_INPUT           INT8_C(-5)

/** BMI160 unique chip identifier */
#define BMI160_CHIP_ID                   UINT8_C(0xD1)

/** Soft reset command */
#define BMI160_SOFT_RESET_CMD            UINT8_C(0xb6)
#define BMI160_SOFT_RESET_DELAY_MS       UINT8_C(15)

/* Delay in ms settings */
#define BMI160_ACCEL_DELAY_MS            UINT8_C(5)
#define BMI160_GYRO_DELAY_MS             UINT8_C(81)
#define BMI160_ONE_MS_DELAY              UINT8_C(1)

/** Power mode settings */
/* Accel power mode */
#define BMI160_ACCEL_NORMAL_MODE         UINT8_C(0x11)
#define BMI160_ACCEL_LOWPOWER_MODE       UINT8_C(0x12)
#define BMI160_ACCEL_SUSPEND_MODE        UINT8_C(0x10)

/* Gyro power mode */
#define BMI160_GYRO_SUSPEND_MODE         UINT8_C(0x14)
#define BMI160_GYRO_NORMAL_MODE          UINT8_C(0x15)
#define BMI160_GYRO_FASTSTARTUP_MODE     UINT8_C(0x17)

/** Range settings */
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G            UINT8_C(0x03)
#define BMI160_ACCEL_RANGE_4G            UINT8_C(0x05)
#define BMI160_ACCEL_RANGE_8G            UINT8_C(0x08)
#define BMI160_ACCEL_RANGE_16G           UINT8_C(0x0C)

/* Gyro Range */
#define BMI160_GYRO_RANGE_2000_DPS       UINT8_C(0x00)
#define BMI160_GYRO_RANGE_1000_DPS       UINT8_C(0x01)
#define BMI160_GYRO_RANGE_500_DPS        UINT8_C(0x02)
#define BMI160_GYRO_RANGE_250_DPS        UINT8_C(0x03)
#define BMI160_GYRO_RANGE_125_DPS        UINT8_C(0x04)

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1        UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2        UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4      UINT8_C(0x02)
#define BMI160_ACCEL_BW_CIC_AVG8         UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16        UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32        UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64        UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128       UINT8_C(0x07)

#define BMI160_GYRO_BW_OSR4_MODE         UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE         UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE       UINT8_C(0x02)
#define BMI160_GYRO_BW_CIC_MODE          UINT8_C(0x03)

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMI160_ACCEL_ODR_RESERVED        UINT8_C(0x00)
#define BMI160_ACCEL_ODR_0_78HZ          UINT8_C(0x01)
#define BMI160_ACCEL_ODR_1_56HZ          UINT8_C(0x02)
#define BMI160_ACCEL_ODR_3_12HZ          UINT8_C(0x03)
#define BMI160_ACCEL_ODR_6_25HZ          UINT8_C(0x04)
#define BMI160_ACCEL_ODR_12_5HZ          UINT8_C(0x05)
#define BMI160_ACCEL_ODR_25HZ            UINT8_C(0x06)
#define BMI160_ACCEL_ODR_50HZ            UINT8_C(0x07)
#define BMI160_ACCEL_ODR_100HZ           UINT8_C(0x08)
#define BMI160_ACCEL_ODR_200HZ           UINT8_C(0x09)
#define BMI160_ACCEL_ODR_400HZ           UINT8_C(0x0A)
#define BMI160_ACCEL_ODR_800HZ           UINT8_C(0x0B)
#define BMI160_ACCEL_ODR_1600HZ          UINT8_C(0x0C)
#define BMI160_ACCEL_ODR_RESERVED0       UINT8_C(0x0D)
#define BMI160_ACCEL_ODR_RESERVED1       UINT8_C(0x0E)
#define BMI160_ACCEL_ODR_RESERVED2       UINT8_C(0x0F)

/* Gyro Output data rate */
#define BMI160_GYRO_ODR_RESERVED         UINT8_C(0x00)
#define BMI160_GYRO_ODR_25HZ             UINT8_C(0x06)
#define BMI160_GYRO_ODR_50HZ             UINT8_C(0x07)
#define BMI160_GYRO_ODR_100HZ            UINT8_C(0x08)
#define BMI160_GYRO_ODR_200HZ            UINT8_C(0x09)
#define BMI160_GYRO_ODR_400HZ            UINT8_C(0x0A)
#define BMI160_GYRO_ODR_800HZ            UINT8_C(0x0B)
#define BMI160_GYRO_ODR_1600HZ           UINT8_C(0x0C)
#define BMI160_GYRO_ODR_3200HZ           UINT8_C(0x0D)

/* Maximum limits definition */
#define BMI160_ACCEL_ODR_MAX             UINT8_C(15)
#define BMI160_ACCEL_BW_MAX              UINT8_C(2)
#define BMI160_ACCEL_RANGE_MAX           UINT8_C(12)
#define BMI160_GYRO_ODR_MAX              UINT8_C(13)
#define BMI160_GYRO_BW_MAX               UINT8_C(2)
#define BMI160_GYRO_RANGE_MAX            UINT8_C(4)

/** Interface settings */
#define BMI160_SPI_INTF                  UINT8_C(1)
#define BMI160_I2C_INTF                  UINT8_C(0)
#define BMI160_SPI_RD_MASK               UINT8_C(0x80)
#define BMI160_SPI_WR_MASK               UINT8_C(0x7F)

/** BMI160 I2C address */
#define BMI160_I2C_ADDR                  UINT8_C(0x68)

/********************************************************/
/* type definitions */
typedef int8_t (*bmi160_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

typedef void (*bmi160_delay_fptr_t)(uint32_t period);


/*************************** Data structures *****************************/

struct bmi160_sensor_data
{
     /*! X-axis sensor data */
     int16_t x;
     /*! Y-axis sensor data */
     int16_t y;
     /*! Z-axis sensor data */
     int16_t z;
};

enum bmi160_select_sensor
{
    BMI160_ACCEL_ONLY = 0,
    BMI160_GYRO_ONLY,
    BMI160_BOTH_ACCEL_AND_GYRO
};

struct bmi160_cfg
{
    /*! power mode */
    uint8_t power;
    /*! output data rate */
    uint8_t odr;
    /*! range */
    uint8_t range;
    /*! bandwidth */
    uint8_t bw;
};


enum bmi160_int_channel
{
    BMI160_INT_CHANNEL_1, /* interrupt Channel 1 */
    BMI160_INT_CHANNEL_2 /* interrupt Channel 2 */
};


enum bmi160_int_types
{
    BMI160_ACC_SLOPE_INT, /* Slope/Any-motion interrupt */
    BMI160_ACC_SIG_MOTION_INT, /* Significant motion interrupt */
    BMI160_STEP_DETECT_INT, /* Step detector interrupt */
    BMI160_ACC_DOUBLE_TAP_INT, /* double tap interrupt */
    BMI160_ACC_SINGLE_TAP_INT, /* single tap interrupt */
    BMI160_ACC_ORIENT_INT, /* orientation interrupt */
    BMI160_ACC_FLAT_INT, /* flat interrupt */
    BMI160_ACC_HIGH_G_INT, /* high-g interrupt */
    BMI160_ACC_LOW_G_INT, /* low-g interrupt */
    BMI160_ACC_SLOW_NO_MOTION_INT, /* slow/no-motion interrupt */
    BMI160_ACC_GYRO_DATA_RDY_INT, /* data ready interrupt  */
};


struct bmi160_acc_tap_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! tap threshold */
    uint16_t tap_thr : 5;
    /*! tap shock */
    uint16_t tap_shock : 1;
    /*! tap quiet */
    uint16_t tap_quiet : 1;
    /*! tap sample */
    uint16_t tap_sample : 2;
    /*! tap duration */
    uint16_t tap_dur : 3;
#elif BIG_ENDIAN == 1
    /*! tap duration */
    uint16_t tap_dur : 3;
    /*! tap sample */
    uint16_t tap_sample : 2;
    /*! tap quiet */
    uint16_t tap_quiet : 1;
    /*! tap shock */
    uint16_t tap_shock : 1;
    /*! tap threshold */
    uint16_t tap_thr : 5;
#endif
};


struct bmi160_acc_slop_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! slope interrupt x */
    uint8_t slope_x : 1;
    /*! slope interrupt y */
    uint8_t slope_y : 1;
    /*! slope interrupt z */
    uint8_t slope_z : 1;
    /*! slope duration */
    uint8_t slope_dur : 2;
    /*! slope threshold */
    uint8_t slope_thr;
#elif BIG_ENDIAN == 1
    /*! slope threshold */
    uint8_t slope_thr;
    /*! slope duration */
    uint8_t slope_dur : 2;
    /*! slope interrupt z */
    uint8_t slope_z : 1;
    /*! slope interrupt y */
    uint8_t slope_y : 1;
    /*! slope interrupt x */
    uint8_t slope_x : 1;
#endif
};


struct bmi160_acc_sig_mot_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;
    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;
    /*! sig-motion threshold */
    uint8_t sig_mot_thres;
#elif BIG_ENDIAN == 1
    /*! sig-motion threshold */
    uint8_t sig_mot_thres;
    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;
    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;
#endif
};


struct bmi160_acc_step_detect_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! minimum threshold */
    uint8_t min_threshold : 2;
    /*! minimal detectable step time */
    uint8_t steptime_min : 3;
    /*! enable normal mode setting */
    uint8_t normal_mode_en : 1;
    /*! enable sensitive mode setting */
    uint8_t sensitive_mode_en : 1;
    /*! enable robust mode setting */
    uint8_t robust_mode_en : 1;
#elif BIG_ENDIAN == 1
    /*! enable robust mode setting */
    uint8_t robust_mode_en : 1;
    /*! enable sensitive mode setting */
    uint8_t sensitive_mode_en : 1;
    /*! enable normal mode setting */
    uint8_t normal_mode_en : 1;
    /*! minimal detectable step time */
    uint8_t steptime_min : 3;
    /*! minimum threshold */
    uint8_t min_threshold : 2;
#endif
};


struct bmi160_acc_no_motion_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! no motion interrupt x */
    uint16_t no_motion_x :1;
    /*! no motion interrupt y */
    uint16_t no_motion_y :1;
    /*! no motion interrupt z */
    uint16_t no_motion_z :1;
    /*! no motion duration */
    uint16_t no_motion_dur : 6;
    /*! no motion sel , �1� - enable no-motion ,�0�- enable slow-motion */
    uint16_t no_motion_sel : 1;
    /*! no motion threshold */
    uint8_t no_motion_thres;
#elif BIG_ENDIAN == 1
    /*! no motion threshold */
    uint8_t no_motion_thres;
    /*! no motion sel , �1� - enable no-motion ,�0�- enable slow-motion */
    uint16_t no_motion_sel : 1;
    /*! no motion duration */
    uint16_t no_motion_dur : 6;
    /* no motion interrupt z */
    uint16_t no_motion_z :1;
    /*! no motion interrupt y */
    uint16_t no_motion_y :1;
    /*! no motion interrupt x */
    uint16_t no_motion_x :1;
#endif
};


struct bmi160_acc_orient_int_cfg
{
#if LITTLE_ENDIAN == 1
      /*! thresholds for switching between the different orientations */
      uint16_t orient_mode : 2;
      /*! blocking_mode */
      uint16_t orient_blocking : 2;
      /*! Orientation interrupt hysteresis */
      uint16_t orient_hyst : 4;
      /*! Orientation interrupt theta */
      uint16_t orient_theta : 6;
      /*! Enable/disable Orientation interrupt */
      uint16_t orient_ud_en : 1;
      /*! exchange x- and z-axis in algorithm ,�0� - z, �1� - x */
      uint16_t axes_ex : 1;
#elif BIG_ENDIAN == 1
      /*! exchange x- and z-axis in algorithm ,�0� - z, �1� - x */
      uint16_t axes_ex : 1;
      /*! Enable/disable Orientation interrupt */
      uint16_t orient_ud_en : 1;
      /*! Orientation interrupt theta */
      uint16_t orient_theta : 6;
      /*! Orientation interrupt hysteresis */
      uint16_t orient_hyst : 4;
      /*! blocking_mode */
      uint16_t orient_blocking : 2;
      /*! thresholds for switching between the different orientations */
      uint16_t orient_mode : 2;
#endif
};


struct bmi160_acc_flat_detect_int_cfg
{
#if LITTLE_ENDIAN == 1
      /*! flat threshold */
      uint16_t flat_theta : 6;
      /*! flat interrupt hysteresis */
      uint16_t flat_hy : 3;
      /*! delay time for which the flat value must remain stable for the flat interrupt to be generated */
      uint16_t flat_hold_time : 2;
#elif BIG_ENDIAN == 1
      /*! delay time for which the flat value must remain stable for the flat interrupt to be generated */
      uint16_t flat_hold_time : 2;
      /*! flat interrupt hysteresis */
      uint16_t flat_hy : 3;
      /*! flat threshold */
      uint16_t flat_theta : 6;
#endif
};


struct bmi160_acc_low_g_int_cfg
{
#if LITTLE_ENDIAN == 1
       /*! low-g interrupt trigger delay */
       uint8_t low_dur;
       /*! low-g interrupt trigger threshold */
       uint8_t low_thres;
       /*! hysteresis of low-g interrupt */
       uint8_t low_hyst : 2;
       /*! �0� - single-axis mode ,�1� - axis-summing mode */
       uint8_t low_mode : 1;
#elif BIG_ENDIAN == 1
       /*! �0� - single-axis mode ,�1� - axis-summing mode */
       uint8_t low_mode : 1;
       /*! hysteresis of low-g interrupt */
       uint8_t low_hyst : 2;
       /*! low-g interrupt trigger threshold */
       uint8_t low_thres;
       /*! low-g interrupt trigger delay */
       uint8_t low_dur;
#endif
};


struct bmi160_acc_high_g_int_cfg
{
#if LITTLE_ENDIAN == 1
       /*! High-g interrupt x  */
       uint8_t high_g_x : 1;
       /*! High-g interrupt y  */
       uint8_t high_g_y : 1;
       /*! High-g interrupt z  */
       uint8_t high_g_z : 1;
       /*! High-g hysteresis  */
       uint8_t high_hy : 2;
       /*! High-g threshold */
       uint8_t high_thres;
       /*! High-g duration */
       uint8_t high_dur;
#elif BIG_ENDIAN == 1
       /*! High-g duration */
       uint8_t high_dur;
       /*! High-g threshold */
       uint8_t high_thres;
       /*! High-g hysteresis  */
       uint8_t high_hy : 2;
       /*! High-g interrupt z  */
       uint8_t high_g_z : 1;
       /*! High-g interrupt y  */
       uint8_t high_g_y : 1;
       /*! High-g interrupt x  */
       uint8_t high_g_x : 1;
#endif
};


struct bmi160_fifo_int_cfg
{
#if LITTLE_ENDIAN == 1
    /*! enable/disable gyro data to store them in fifo */
    uint8_t fifo_gyr_en : 1;
    /*! enable/disable accel data to store them in fifo */
    uint8_t fifo_acc_en : 1;
    /*! enable/disable mag data to store them in fifo */
    uint8_t fifo_mag_en : 1;
    /*! fifo watermark level */
    uint8_t watermark;
#elif BIG_ENDIAN == 1
    /*! fifo watermark level */
    uint8_t watermark;
    /*! enable/disable mag data to store them in fifo */
    uint8_t fifo_mag_en : 1;
    /*! enable/disable accel data to store them in fifo */
    uint8_t fifo_acc_en : 1;
    /*! enable/disable gyro data to store them in fifo */
    uint8_t fifo_gyr_en : 1;
#endif
};


struct bmi160_intr_pin_sett
{
#if LITTLE_ENDIAN == 1
    /*! To enable either INT1 or INT2 pin as output. 0- output disabled ,1- output enabled */
    uint16_t output_en : 1;
    /*! �0� - push-pull �1�- open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;
    /*! �0� - active low , �1� - active high level .if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;
    /*! �0� - level trigger , �1� - edge trigger  */
    uint16_t edge_ctrl : 1;
    /*! To enable either INT1 or INT2 pin as input . �0� - input disabled ,�1� - input enabled */
    uint16_t input_en : 1;
    /*! latch duration*/
    uint16_t latch_dur : 4;
#elif BIG_ENDIAN == 1
    /*! latch duration*/
    uint16_t latch_dur : 4;
    /*! Latched,non-latched or temporary interrupt modes */
    uint16_t input_en : 1;
    /*! �1� - edge trigger, �0� - level trigger */
    uint16_t edge_ctrl : 1;
    /*! �0� - active low , �1� - active high level .if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;
    /*! �0� - push-pull , �1� - open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;
    /*! To enable either INT1 or INT2 pin as output. �0� - output disabled , �1� - output enabled */
    uint16_t output_en : 1;
#endif
};


union bmi160_int_type_cfg
{
    /*! Tap interrupt structure */
    struct bmi160_acc_tap_int_cfg   acc_tap_int;
    /*! Slope interrupt structure */
    struct bmi160_acc_slop_int_cfg   acc_slope_int;
    /*! Significant motion interrupt structure */
    struct bmi160_acc_sig_mot_int_cfg   acc_sig_motion_int;
    /*! Step detector interrupt structure */
    struct bmi160_acc_step_detect_int_cfg   acc_step_detect_int;
    /*! No motion interrupt structure */
    struct bmi160_acc_no_motion_int_cfg   acc_no_motion_int;
    /*! Orientation interrupt structure */
    struct bmi160_acc_orient_int_cfg    acc_orient_int;
    /*! Flat interrupt structure */
    struct bmi160_acc_flat_detect_int_cfg  acc_flat_int;
    /*! Low-g interrupt structure */
    struct bmi160_acc_low_g_int_cfg   acc_low_g_int;
    /*! High-g interrupt structure */
    struct bmi160_acc_high_g_int_cfg   acc_high_g_int;
    /*! Fifo full/Watermark interrupt structure */
    struct bmi160_fifo_int_cfg  fifo_int;
};


struct bmi160_intr_sett
{
    /*! Interrupt channel */
    enum bmi160_int_channel   int_channel;
    /*! Select Interrupt */
    enum bmi160_int_types   int_type;
    /*! Structure configuring Interrupt pins */
    struct bmi160_intr_pin_sett   int_pin_sett;
    /*! Union configures required interrupt */
    union bmi160_int_type_cfg   int_type_cfg;
};


struct bmi160_dev
{
    /*! Chip Id */
    uint8_t chip_id;
    /*! Device Id */
    uint8_t id;
    /*! �0� - I2C , �1� - SPI Interface */
    uint8_t interface;
    /*! Mag manual/auto mode status */
    uint8_t mag_manual_enable;
    /*! Structure to configure Accel sensor */
    struct bmi160_cfg  accel_cfg;
    /*! Pointer to hold previous/old accel config parameters.This is used at driver level
     * to prevent overwriting of same data,hence user does not change it in the code */
    void *prev_accel_cfg_ptr;
    /*! Structure to configure Gyro sensor */
    struct bmi160_cfg  gyro_cfg;
    /*! Pointer to hold previous/old gyro config parameters.This is used at driver level
     * to prevent overwriting of same data,hence user does not change it in the code */
    void *prev_gyro_cfg_ptr;
    /*! Read function pointer */
    bmi160_com_fptr_t read;
    /*! Write function pointer */
    bmi160_com_fptr_t write;
    /*!  Delay function pointer */
    bmi160_delay_fptr_t delay_ms;
};


/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* BMI160_DEFS_H_ */
