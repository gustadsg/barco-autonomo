/*
 * DCMotor.h
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include "Pwm.h"
#include "stm32f4xx_hal.h"
#include "inttypes.h"

#define DC_MOTOR_MIN_PERCENT 0;
#define DC_MOTOR_MAX_PERCENT 100;

typedef struct {
	TIM_HandleTypeDef handle;
	uint32_t channel;
	uint32_t period;
} DCMOTOR_TimerConfig_t;

typedef enum {
	BACKWARD = 0x04,
	FORWARD = 0x08
} DCMOTOR_Direction_t;

typedef struct {
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} DCMOTOR_Pin_t;

typedef struct {
	DCMOTOR_TimerConfig_t timerConfig;
	DCMOTOR_Pin_t latch;
	DCMOTOR_Pin_t enable;
	DCMOTOR_Pin_t data;
	DCMOTOR_Pin_t clk;
} DCMOTOR_Config_t;

void DCMOTOR_SetSpeedPercentage(DCMOTOR_Config_t config, float percentage);
void DCMOTOR_SetDirection(DCMOTOR_Config_t config, DCMOTOR_Direction_t direction);

#endif /* INC_DCMOTOR_H_ */
