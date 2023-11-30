/*
 * Pwm.h
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void PWM_SetValue(TIM_HandleTypeDef timer, uint32_t channel, uint32_t period,
		uint16_t pulseLength);

#endif /* INC_PWM_ */
