/*
 * DCMotor.c
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#include "DCMOTOR.h"

void DCMOTOR_SetSpeedPercentage(DCMOTOR_Config_t config, float percentage) {
	DCMOTOR_TimerConfig_t timerConfig = config.timerConfig;

	int minPercent = DC_MOTOR_MIN_PERCENT;
	int maxPulseLength = timerConfig.period;
	uint16_t pulseLength = percentage * timerConfig.period;
	if (pulseLength < minPercent) pulseLength = 0;
	if (pulseLength > maxPulseLength) pulseLength = maxPulseLength;
	PWM_SetValue(timerConfig.handle, timerConfig.channel, timerConfig.period, pulseLength);
}


void DCMOTOR_SetDirection(DCMOTOR_Config_t config, DCMOTOR_Direction_t direction) {
	GPIO_PinState dataToBeSent = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(config.data.GPIOx, config.data.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config.enable.GPIOx, config.enable.GPIO_Pin, GPIO_PIN_RESET);

	uint8_t comparator = 0b10000000;
	for (uint8_t i = 0; i < 8; ++i) {
		dataToBeSent = direction & comparator;
		comparator = comparator >> 1;

		HAL_GPIO_WritePin(config.data.GPIOx, config.data.GPIO_Pin, dataToBeSent);
		HAL_Delay(1);

		HAL_GPIO_WritePin(config.clk.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(config.clk.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(config.data.GPIOx, config.data.GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_RESET);
}
