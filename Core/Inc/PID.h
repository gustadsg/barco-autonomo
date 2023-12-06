/*
 * PID.h
 *
 *  Created on: Nov 13, 2023
 *      Author: gusta
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define PID_CURRENT 0
#define PID_LAST 1

typedef struct {
	// @brief Proportional gain
	float Kp;
	// @brief Integral gain
	float Ki;
	// @brief Derivative gain
	float Kd;
	// @brief The desired value the system should reach
	float setpoint;
	// @brief Measured value: the current value of the system
	float measured;
	// @brief Maximum output value of the controller
	float maxOutput;
	// @brief Minimum output value of the controller
	float minOutput;
	// @brief Array to store the last two errors for derivative calculation
	float errorArray[2];
	// @brief Integral of the error over time
	float errorIntegral;
	// @brief Derivative of the error with respect to time
	float errorDerivative;
	// @brief The period of time in milliseconds at which the controller operates
	int periodMs;
} PID_Controller_t;

/**
 * @brief Initializes a PID controller with the specified gains and control period.
 * @param controller Pointer to the PID controller structure to be initialized.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param periodMs Time period in milliseconds at which the controller operates.
 */
void PID_Create(PID_Controller_t *controller, float kp, float ki, float kd,
		int periodMs);

/**
 * @brief Sets the saturation limits for the PID controller's output.
 * @param controller Pointer to the PID controller structure.
 * @param min Minimum allowable output value.
 * @param max Maximum allowable output value.
 */
void PID_SetSaturationLimits(PID_Controller_t *controller, float min, float max);

/**
 * @brief Sets the desired setpoint for the PID controller.
 * @param controller Pointer to the PID controller structure.
 * @param setpoint The desired setpoint value the system should reach.
 */
void PID_SetSetpoint(PID_Controller_t *controller, float setpoint);

/**
 * @brief Processes a new input value for the PID controller.
 * @param controller Pointer to the PID controller structure.
 * @param input The current input value to the system.
 */
void PID_ProcessInput(PID_Controller_t *controller, float input);

/**
 * @brief Calculates the control action based on the PID algorithm.
 * @param controller Pointer to the PID controller structure.
 * @return The calculated control action.
 */
float PID_CalculateControlAction(PID_Controller_t *controller);

/**
 * @brief Applies saturation limits to the original control output.
 * @param controller Pointer to the PID controller structure.
 * @param originalOutput The control output before saturation.
 * @return The saturated control output within the specified limits.
 */
float __PID_SaturateOutput(PID_Controller_t *controller, float originalOutput);

#endif /* INC_PID_H_ */
