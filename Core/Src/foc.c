/*
 * foc.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include <stm32f1xx_hal.h>
#include "foc_utils.h"
#include "foc.h"
#include "AS5600.h"
#include "timer_utils.h"

/*
 * @scope static
 * @brief Set pwm signal of channels
 * @param[in] Controller control
 * @param[in] Motor* motor
 */
static void SetPWM(Motor* motor)
{
	TIM1->CCR1 = _constrain(motor->phaseVs->Ua / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
	TIM1->CCR2 = _constrain(motor->phaseVs->Ub / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
	TIM1->CCR3 = _constrain(motor->phaseVs->Uc / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
}

/*
 * @brief Inverse Clarke & Park transformations
 * @param[in] Motor* motor
 * @note Calls setpwm()
 */
static void SetPhaseVoltage(Motor* motor) {

    /* Normalize electric angle */
    float el_angle = _normalizeAngle(motor->params->electric_angle);

	/* Inverse park transform */
	float Ualpha = -(motor->dqVals->Uq) * _sin(el_angle);
	float Ubeta = motor->dqVals->Uq * _cos(el_angle);

	/* Inverse Clarke transform */
	motor->phaseVs->Ua = Ualpha + motor->params->supply_voltage / 2;
	motor->phaseVs->Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + motor->params->supply_voltage / 2;
	motor->phaseVs->Uc = (- Ualpha - _SQRT3 * Ubeta) / 2 + motor->params->supply_voltage / 2;

	SetPWM(motor);
}

/*
 * @brief Initializes motor with pole pair count & supply voltage.
 * @params[in] Motor* motor
 * @params[in] uint8_t pole_pairs
 * @params[in] float supply_voltage
 * @note Motor voltage limit defaults to supply voltage / 2.
 */
void MotorInit(Motor* motor, uint8_t pole_pairs, float supply_voltage)
{
	/* Set pole pairs & supply voltage */
	motor->params->pole_pairs = pole_pairs;
	motor->params->supply_voltage = supply_voltage;
	motor->params->motor_voltage_limit = supply_voltage / 2;

	if(motor->sensor != NULL)
	{
		motor->sensor = NULL;
	}
}

/*
 * @brief Links a AS5600 sensor to a motor object
 * @param[in] Motor* motor
 * @param[in] AS5600* sensor
 * @param[in] I2C_HandleTypeDef *i2c_handle
 */
void LinkSensor(Motor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle)
{
	uint8_t init_stat = AS5600_Init(sensor, i2c_handle, 1);

	/* Check if sensor link successful */
	if(init_stat != 0)
	{
		return;
	}

	motor->sensor = sensor;
}

/*
 * @brief Sends sensor readings through USB. For debugging sensors.
 * @param[in] Motor* motor
 */
void DebugSensor(Motor* motor)
{
	return;
}

/*
 * @brief Open-loop velocity control
 * @param[in] Motor* motor
 * @param[in] float target_velocity (rads/sec)
 * @warning Ensure DWT_Init() is called in main() to initialize timer first.
 */
void OLVelocityControl(Motor* motor, float target_velocity)
{
	/* Track current micros */
	uint32_t now_us = micros();

	/* Time difference since last call */
	float time_elapsed_s = (now_us - motor->params->prev_us) / 1000000;
	time_elapsed_s = time_elapsed_s > 0.5 ? 0.001 : time_elapsed_s;

	/* Update virtual shaft angle, and calculate phase voltages */
	motor->params->shaft_angle = _normalizeAngle(motor->params->shaft_angle + target_velocity * time_elapsed_s);
	motor->dqVals->Uq = motor->params->motor_voltage_limit;
	SetPhaseVoltage(motor);

	/* Update timestamp */
	motor->params->prev_us = micros();
}
