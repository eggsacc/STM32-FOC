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
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "foc_utils.h"
#include "foc.h"
#include "AS5600.h"
#include "timer_utils.h"

/*
 * @scope static inline
 * @brief Set pwm duty cycle of timer channels
 * @param[in] Motor* motor
 */
__STATIC_INLINE void SetPWM(Motor* motor)
{
	motor->timer->Instance->CCR1 = _constrain(motor->phaseVs->Ua / motor->supply_voltage, 0.0f, 1.0f) * 256;
	motor->timer->Instance->CCR2 = _constrain(motor->phaseVs->Ub / motor->supply_voltage, 0.0f, 1.0f) * 256;
	motor->timer->Instance->CCR3 = _constrain(motor->phaseVs->Uc / motor->supply_voltage, 0.0f, 1.0f) * 256;
}

/*
 * @brief Inverse Clarke & Park transformations to calculate phase voltages
 * @param[in] Motor* motor
 * @note Calls setpwm()
 */
void SetTorque(Motor* motor) {
	/* Constrain Uq to within voltage range */
	motor->dqVals->Uq = _constrain(motor->dqVals->Uq, -motor->motor_v_limit, motor->motor_v_limit);
    /* Normalize electric angle */
    float el_angle = _normalizeAngle(motor->vars->electric_angle);

	/* Inverse park transform */
	float Ualpha = -(motor->dqVals->Uq) * _sin(el_angle);
	float Ubeta = motor->dqVals->Uq * _cos(el_angle);

	/* Inverse Clarke transform */
	motor->phaseVs->Ua = Ualpha + motor->supply_voltage / 2.0f;
	motor->phaseVs->Ub = (_SQRT3 * Ubeta - Ualpha) / 2.0f + motor->supply_voltage / 2.0f;
	motor->phaseVs->Uc = (- Ualpha - _SQRT3 * Ubeta) / 2.0f + motor->supply_voltage / 2.0f;

	SetPWM(motor);
}

/*
 * @brief Starts PWM channels 1, 2, 3 of specified timer.
 * @param[in] TIM_HandleTypeDef timer
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer)
{
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3);
}

/*
 * @brief Initializes motor object.
 *        Creates all relevant structs and returns main motor struct.
 *
 * @param[in] TIM_HandleTypeDef* timer
 * @param[in] float supply_voltage
 * @param[in] uint8_t pole_pairs
 *
 * @note
 * - Sensor pointer is NULL by default (no sensor)
 * - Motor voltage limit set to supply voltage / 2 by default
 *
 * @retval Motor motor
 */
Motor MotorInit(TIM_HandleTypeDef* timer, float supply_voltage, uint8_t pole_pairs)
{
	/* Create structs */
	static Var_t motor_vars;
	motor_vars.electric_angle = 0;
	motor_vars.prev_us = 0;
	motor_vars.zero_angle = 0;
	motor_vars.shaft_angle = 0;

	static DQval_t motor_dq = {0, 0};
	static PhaseV_t motor_pv = {0, 0, 0};

	Motor motor = {pole_pairs, 0, supply_voltage / 2, supply_voltage, &motor_vars, &motor_dq, &motor_pv, NULL, timer};

	return motor;
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
		motor->sensor = NULL;
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
	/* Check if motor timer initialized properly */
	if(motor->timer == NULL)
	{
		return;
	}

	/* Track current micros */
	uint32_t now_us = micros();

	/* Time difference since last call */
	uint32_t time_elapsed_us = now_us - motor->vars->prev_us;
	float time_elapsed_s = time_elapsed_us  / 1000000.0f;
	time_elapsed_s = time_elapsed_s > 0.5 ? 0.001 : time_elapsed_s;

	/* Update virtual shaft angle, and calculate phase voltages */
	motor->vars->shaft_angle = _normalizeAngle(motor->vars->shaft_angle + target_velocity * time_elapsed_s);
	motor->vars->electric_angle = _electricalAngle(motor->vars->shaft_angle, motor->pole_pairs);
	motor->dqVals->Uq = motor->motor_v_limit;
	SetTorque(motor);

	/* Update timestamp */
	motor->vars->prev_us = micros();
}

/*
 * @brief Closed loop position control
 * @param[in] Motor* motor
 * @param[in] float target_pos (in radians)
 * @note Does nothing if no sensor is attached to the motor.
 */
void CLPositionControl(Motor* motor, float target_pos)
{
	/* Check if sensor is attached to motor */
	if(motor->sensor == NULL)
	{
		return;
	}

	/* Electrical angle calculated based on sensor angle * pole pairs */
	motor->vars->electric_angle = _normalizeAngle((float)(DIR * motor->pole_pairs) * AS5600_ReadNormalizedAngle(motor->sensor));
	/* KP sets max torque when shaft is 45deg (0.7854 rad) off from target pos */
	motor->dqVals->Uq = KP * (target_pos - DIR * AS5600_ReadAngle(motor->sensor));
	SetTorque(motor);
}
