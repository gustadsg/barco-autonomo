/*
 * Servo.h
 *
 *  Created on: Oct 7, 2023
 *      Author: gusta
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "PWM.h"

#define SERVO_MIN_ANGLE -90;
#define SERVO_MAX_ANGLE 90;
#define SERVO_MIDDLE_ANGLE 0;
#define SERVO_MIN_PERCENT 0;
#define SERVO_MAX_PERCENT 100;

typedef struct {
	float min;
	float max;
} SERVO_Scale_t;

typedef struct {
	float offset;
	float gain;
} SERVO_Calibration_t;

typedef struct {
	TIM_HandleTypeDef handle;
	uint32_t channel;
	uint32_t period;

	//	porcentagens de duty cycle para ângulos mínimo e máximo
	float minDutyCyclePercentage;
	float maxDutyCyclePercentage;
} SERVO_TimerConfig_t;

typedef struct {
	SERVO_TimerConfig_t timerConfig;
	SERVO_Calibration_t calibration;
} SERVO_Config_t;

void SERVO_SetAngle(SERVO_Config_t servoConfig, float angle);
void SERVO_SetPercentage(SERVO_Config_t servoConfig, float angle);

float __SERVO_ConvertScales(SERVO_Scale_t from, SERVO_Scale_t to, float point);
float __SERVO_GetCalibratedAngle(SERVO_Calibration_t calibration, float desiredAngle);
SERVO_Scale_t __SERVO_GetPWMScale(SERVO_TimerConfig_t timerConfig);

#endif /* INC_SERVO_H_ */
