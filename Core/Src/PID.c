/*
 * PID.c
 *
 *  Created on: Nov 13, 2023
 *      Author: gusta
 */

#include <math.h>
#include "PID.h"

void PID_Create(PID_Controller_t *controller, float kp, float ki, float kd,
		int periodMs) {
	controller->Kp = kp;
	controller->Ki = ki;
	controller->Kd = kd;

	controller->setpoint = 0;
	controller->measured = 0;

	controller->errorIntegral = 0;
	controller->errorDerivative = 0;

	controller->errorArray[PID_CURRENT] = 0;
	controller->errorArray[PID_LAST] = 0;

	controller->periodMs = periodMs;

	// no saturation limits by default
	controller->maxOutput = INFINITY;
	controller->minOutput = -INFINITY;
}

void PID_SetSaturationLimits(PID_Controller_t *controller, float min, float max) {
	controller->minOutput = min;
	controller->maxOutput = max;
}

void PID_SetSetpoint(PID_Controller_t *controller, float setpoint) {
	controller->setpoint = setpoint;
}

void PID_ProcessInput(PID_Controller_t *controller, float input) {
	float error = controller->setpoint - input;

	controller->errorIntegral += (error * controller->periodMs)/2;

	controller->errorArray[PID_LAST] = controller->errorArray[PID_CURRENT];
	controller->errorArray[PID_CURRENT] = error;

	controller->errorDerivative = (controller->errorArray[PID_CURRENT]
			- controller->errorArray[PID_LAST]) / controller->periodMs;
}

float PID_CalculateControlAction(PID_Controller_t *controller) {
	float P = controller->Kp * controller->errorArray[PID_CURRENT];
	float I = controller->Ki * controller->errorIntegral;
	float D = controller->Kd * controller->errorDerivative;

	float PID = __PID_SaturateOutput(controller, P + I + D);

	return PID;
}

float __PID_SaturateOutput(PID_Controller_t *controller, float originalOutput) {
	// limit integral error
	if (controller->errorIntegral * controller->Ki > controller->maxOutput)
		controller->errorIntegral = controller->maxOutput/controller->Ki;

	if (controller->errorIntegral * controller->Ki < controller->minOutput)
			controller->errorIntegral = controller->minOutput/controller->Ki;

	if (originalOutput > controller->maxOutput)
		return controller->maxOutput;
	if (originalOutput < controller->minOutput)
		return controller->minOutput;
	return originalOutput;
}
