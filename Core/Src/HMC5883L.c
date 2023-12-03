/*
 * HMC5883L.c
 *
 *  Created on: Oct 17, 2023
 *      Author: Gustavo da Silva Gomes
 */

#include "HMC5883L.h"

HAL_StatusTypeDef HMC5883L_Init(HMC5883L_Config_t config) {
	uint8_t registerA = HMC5883L_REG_BIT_CRA7;
	registerA <<= 2;
	registerA |= config.samplesNum;
	registerA <<= 3;
	registerA |= config.dataOutputRate;
	registerA <<= 2;
	registerA |= config.measurementMode;

	uint8_t registerB = config.gain;
	registerB <<= 5;

	uint8_t registerMode = 0x00;
	registerMode |= config.operatingMode;
	HAL_Delay(__HMC5883L_MODE_STABILIZATION_TIME_MS);

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_DEVICE_ADDR, HMC5883L_REG_ADDR_A, I2C_MEMADD_SIZE_8BIT, &registerA, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_DEVICE_ADDR, HMC5883L_REG_ADDR_B, I2C_MEMADD_SIZE_8BIT, &registerB, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_DEVICE_ADDR, HMC5883L_REG_ADDR_MODE, I2C_MEMADD_SIZE_8BIT, &registerMode, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef HMC5883L_Read(HMC5883L_Config_t config, HMC5883L_Data_t *data) {
	HAL_StatusTypeDef status;
	HMC5883L_Status_t readyOrLocked = LOCKED;

	// wait until data registers are unlocked
	while(readyOrLocked == LOCKED) {
		status = __HMC5883L_GetStatus(config, &readyOrLocked);
		if(status != HAL_OK) return status;
		if(readyOrLocked == LOCKED) HAL_Delay(10);
	}

	uint8_t addrsLow[3] = {HMC5883L_REG_ADDR_X_L, HMC5883L_REG_ADDR_Z_L , HMC5883L_REG_ADDR_Y_L};
	uint8_t addrsHigh[3] = {HMC5883L_REG_ADDR_X_H, HMC5883L_REG_ADDR_Z_H, HMC5883L_REG_ADDR_Y_H};
	uint8_t axisArr[3] = {__HMC5883L_AXIS_X, __HMC5883L_AXIS_Z, __HMC5883L_AXIS_Y};

	uint8_t dataHigh = 0, dataLow = 0;
	int16_t axisData = 0;

	int16_t offsetData[3];
	offsetData[__HMC5883L_AXIS_X] = config.calibration.x_offset;
	offsetData[__HMC5883L_AXIS_Z] = config.calibration.z_offset;
	offsetData[__HMC5883L_AXIS_Y] = config.calibration.y_offset;

	for(int i=0; i<3; i++) {
		// get most significant bits
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_DEVICE_ADDR, addrsHigh[i], I2C_MEMADD_SIZE_8BIT, &dataHigh, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
		if(status != HAL_OK) return status;
		// get less significant bits
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_DEVICE_ADDR, addrsLow[i], I2C_MEMADD_SIZE_8BIT, &dataLow, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
		if(status != HAL_OK) return status;

		axisData = dataHigh;
		axisData <<= 8;
		axisData |= dataLow;

		axisData -= offsetData[i];

		status = __HMC5883L_SetDataAxis(data, axisArr[i], axisData);
		if(status != HAL_OK) return status;
	}

	__HMC5883L_SetDataAngles(data);

	return HAL_OK;
}

/**
 * Creates CSV format data, with collected points of x, y and z data. This data can be used with other software,
 * such as Matlab, Excel, R, etc, to get the center point of the measured data and apply the found offsets to the
 * HMC5883LCalibration_t in the HMC5883LConfig_t struct.
 */
void HMC5883L_GetCalibrationData(HMC5883L_Config_t config, UART_HandleTypeDef *huart) {
	volatile int32_t num_collected = 0;
	HMC5883L_Data_t data = {0,0,0,0,0};

	// CSV header
	char transmitStr[22] = "x, y, z\n";
	HAL_UART_Transmit(huart, (uint8_t*)transmitStr, strlen(transmitStr), HAL_MAX_DELAY);

	// CSV collected data
	while(num_collected < __HMC5883L_NUM_CALIBRATION_POINTS) {
		HMC5883L_Read(config, &data);
		sprintf(transmitStr,"%i, %i, %i\n", data.x, data.y, data.z);

		HAL_UART_Transmit(huart, (uint8_t*)transmitStr, strlen(transmitStr), HAL_MAX_DELAY);
		HAL_Delay(20);
		num_collected++;
	}
}


HAL_StatusTypeDef __HMC5883L_SetDataAxis(HMC5883L_Data_t *data, uint8_t axis, int16_t axisData) {
	switch(axis) {
		case __HMC5883L_AXIS_X:
			data->x = axisData;
			return HAL_OK;
		case __HMC5883L_AXIS_Y:
			data->y = axisData;
			return HAL_OK;
		case __HMC5883L_AXIS_Z:
			data->z = axisData;
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}

void __HMC5883L_SetDataAngles(HMC5883L_Data_t *data) {
	data->radians = atan2f(data->x, data->y);
	data->degrees = data->radians * (180.0/M_PI);
}

HAL_StatusTypeDef __HMC5883L_GetStatus(HMC5883L_Config_t config, HMC5883L_Status_t *registerStatus) {
	HAL_StatusTypeDef status;
	uint8_t statusRegisterData = 0;

	status = HAL_I2C_Mem_Read(config.handle, HMC5883L_DEVICE_ADDR, HMC5883L_REG_ADDR_STATUS, I2C_MEMADD_SIZE_8BIT, &statusRegisterData, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	// clear the six most significant bits, since they are not used
	statusRegisterData &= 0b00000011;

	*registerStatus = statusRegisterData;

	// the only possible values read are READY and LOCKED, otherwise an error occurred in the communication
	return (statusRegisterData == READY || statusRegisterData == LOCKED) ? HAL_OK : HAL_ERROR;
}
