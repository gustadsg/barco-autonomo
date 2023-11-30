/*
 * Servo.c
 *
 *  Created on: Oct 7, 2023
 *      Author: gusta
 */

#include "SERVO.h"

void SERVO_SetAngle(SERVO_Config_t servoConfig, float angle) {
	SERVO_Scale_t angleScale;
	angleScale.min = SERVO_MIN_ANGLE
	;
	angleScale.max = SERVO_MAX_ANGLE
	;

	SERVO_Scale_t pwmScale = __SERVO_GetPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __SERVO_ConvertScales(angleScale, pwmScale,
			__SERVO_GetCalibratedAngle(servoConfig.calibration, angle));
	PWM_SetValue(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

void SERVO_SetPercentage(SERVO_Config_t servoConfig, float angle) {
	SERVO_Scale_t angleScale;
	angleScale.min = SERVO_MIN_PERCENT;
	angleScale.max = SERVO_MAX_PERCENT;

	SERVO_Scale_t pwmScale = __SERVO_GetPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __SERVO_ConvertScales(angleScale, pwmScale,
			__SERVO_GetCalibratedAngle(servoConfig.calibration, angle));
	PWM_SetValue(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

SERVO_Scale_t __SERVO_GetPWMScale(SERVO_TimerConfig_t timerConfig) {
	SERVO_Scale_t pwmScale;
	pwmScale.min = timerConfig.minDutyCyclePercentage * timerConfig.period;
	pwmScale.max = timerConfig.maxDutyCyclePercentage * timerConfig.period;

	return pwmScale;
}

float __SERVO_ConvertScales(SERVO_Scale_t from, SERVO_Scale_t to, float point) {
	float deltaScaleFrom = from.max - from.min;
	float deltaScaleTo = to.max - to.min;

	float result = ((point - from.min) / deltaScaleFrom) * deltaScaleTo
			+ to.min;

	if(result > to.max) return to.max;
	if(result < to.min) return to.min;

	return result;
}

/*
 * Foi observado que há imprecisões no servo motor, de forma a ser necessário ajustar o valor enviado conforme a reta de calibração obtida
 */
float __SERVO_GetCalibratedAngle(SERVO_Calibration_t calibration, float desiredAngle) {
	return calibration.gain * desiredAngle + calibration.offset; // y = a*x + b
}
