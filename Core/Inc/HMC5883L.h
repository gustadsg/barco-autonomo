/*
 * HMC588L.h
 *
 *  Created on: Oct 17, 2023
 *      Author: Gustavo da Silva Gomes
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "inttypes.h"
#include "math.h"
#include "stm32f4xx_hal.h"

// Register Default Bits
// Register A
#define HMC5883L_REG_BIT_CRA7 0x0

// Register B
#define HMC5883L_REG_BIT_CRB4 0x0
#define HMC5883L_REG_BIT_CRB3 0x0
#define HMC5883L_REG_BIT_CRB2 0x0
#define HMC5883L_REG_BIT_CRB1 0x0
#define HMC5883L_REG_BIT_CRB0 0x0

// Register Mode
#define HMC5883L_REG_BIT_MR7 0X0
#define HMC5883L_REG_BIT_MR6 0X0
#define HMC5883L_REG_BIT_MR5 0X0
#define HMC5883L_REG_BIT_MR4 0X0
#define HMC5883L_REG_BIT_MR3 0X0
#define HMC5883L_REG_BIT_MR2 0X0

// Register Addresses
// Configuration Registers
#define HMC5883L_REG_ADDR_A 0x00
#define HMC5883L_REG_ADDR_B 0x01
#define HMC5883L_REG_ADDR_MODE 0x02

// Data Output Registers
// x
#define HMC5883L_REG_ADDR_X_H 0x03
#define HMC5883L_REG_ADDR_X_L 0x04
//y
#define HMC5883L_REG_ADDR_Y_H 0x07
#define HMC5883L_REG_ADDR_Y_L 0x08
//z
#define HMC5883L_REG_ADDR_Z_H 0x05
#define HMC5883L_REG_ADDR_Z_L 0x06

// Info Registers
#define HMC5883L_REG_ADDR_STATUS 0x09
#define HMC5883L_REG_ADDR_ID_A 0x0A
#define HMC5883L_REG_ADDR_ID_B 0x0B
#define HMC5883L_REG_ADDR_ID_C 0x0C

// IDENTIFICATION REGISTERS EXPECTED VALUES
#define HMC5883L_ID_A_EXPECTED 0b01001000
#define HMC5883L_ID_B_EXPECTED 0b00110100
#define HMC5883L_ID_C_EXPECTED 0b00110011

#define HMC5883L_DEVICE_ADDR 0x1E << 1

#define HMC5883L_BYTE_SZ 1

#define __HMC5883L_AXIS_X 0
#define __HMC5883L_AXIS_Y 2
#define __HMC5883L_AXIS_Z 1

#define __HMC5883L_NUM_CALIBRATION_POINTS 300

#define __HMC5883L_MODE_STABILIZATION_TIME_MS 6

typedef enum {
	HMC5883L_SAMPLES_1,
	HMC5883L_SAMPLES_2,
	HMC5883L_SAMPLES_4,
	HMC5883L_SAMPLES_8
} HMC5883L_SamplesNum_t;

typedef enum {
	HMC5883L_DOR_0_75,
	HMC5883L_DOR_1_5,
	HMC5883L_DOR_3,
	HMC5883L_DOR_7_5,
	HMC5883L_DOR_15,
	HMC5883L_DOR_30,
	HMC5883L_DOR_75
} HMC5883L_DataOutputRate_t;

typedef enum {
	HMC5883L_MESUAREMENT_NORMAL,
	HMC5883L_MESUAREMENT_POSITIVE,
	HMC5883L_MESUAREMENT_NEGATIVE
} HMC5883L_MesuarementMode_t;

typedef enum {
	HMC5883L_GAIN_0_88,
	HMC5883L_GAIN_1_3,
	HMC5883L_GAIN_1_9,
	HMC5883L_GAIN_2_5,
	HMC5883L_GAIN_4_0,
	HMC5883L_GAIN_4_7,
	HMC5883L_GAIN_5_6,
	HMC5883L_GAIN_8_1
} HMC5883L_Gain_t;

typedef enum {
	HMC5883L_CONTINUOUS_MODE,
	HMC5883L_SINGLE_MODE,
	HMC5883L_IDLE_MODE,
	HMC5883L_IDLE_MODE2
} HMC5883L_OperatingMode_t;

typedef struct {
	int16_t x_offset;
	int16_t y_offset;
	int16_t z_offset;
} HMC5883L_Calibration_t;

typedef struct {
	I2C_HandleTypeDef* handle;
	HMC5883L_SamplesNum_t samplesNum;
	HMC5883L_DataOutputRate_t dataOutputRate;
	HMC5883L_MesuarementMode_t measurementMode;
	HMC5883L_Gain_t gain;
	HMC5883L_OperatingMode_t operatingMode;
	HMC5883L_Calibration_t calibration;
} HMC5883L_Config_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	float degrees;
	float radians;
} HMC5883L_Data_t;

typedef enum {
	READY = 1,
	LOCKED
} HMC5883L_Status_t;

HAL_StatusTypeDef HMC5883L_Init(HMC5883L_Config_t config);
HAL_StatusTypeDef HMC5883L_Read(HMC5883L_Config_t config, HMC5883L_Data_t *data);
void HMC5883L_GetCalibrationData(HMC5883L_Config_t config, UART_HandleTypeDef *huart);
HAL_StatusTypeDef __HMC5883L_SetDataAxis(HMC5883L_Data_t *data, uint8_t axis, int16_t axisData);
HAL_StatusTypeDef __HMC5883L_GetStatus(HMC5883L_Config_t config, HMC5883L_Status_t *status);
void __HMC5883L_SetDataAngles(HMC5883L_Data_t *data);
HAL_StatusTypeDef __HMC583L_VerifyIdentity(HMC5883L_Config_t config);

#endif /* INC_HMC5883L_H_ */
