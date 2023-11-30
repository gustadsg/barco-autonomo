/*
 * Pwm.h
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#include "PWM.h"

void PWM_SetValue(TIM_HandleTypeDef timer, uint32_t channel, uint32_t period,
		uint16_t pulseLength) {
	HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // set the period duration
	HAL_TIM_PWM_Init(&timer); // reinitialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulseLength; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // start PWM generation
}
