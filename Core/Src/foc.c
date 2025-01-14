/*
 * foc.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include<stm32f1xx_hal.h>
#include "foc_utils.h"
#include "foc.h"
#include "AS5600.h"

/*
 * @scope static
 * @brief Set pwm signal of channels
 * @param[in] Controller control
 * @param[in] float Ua
 * @param[in] float Ub
 * @param[in] float Uc
 */
static void setPWM(Motor* motor)
{
	TIM1->CCR1 = _constrain(motor->phaseVs->Ua / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
	TIM1->CCR2 = _constrain(motor->phaseVs->Ub / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
	TIM1->CCR3 = _constrain(motor->phaseVs->Uc / motor->params->supply_voltage, 0.0f, 1.0f) * TIM1->ARR;
}

/*
 * @brief Inverse Clarke & Park transformations
 * @param[in] float Uq
 * @param[in] float Ud
 * @param[in] float angle_el
 */
//static void setPhaseVoltage(float Uq, float Ud, float angle_el) {
//  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
//
//  // inverse park transformation
//  Ualpha = -Uq * _sin(angle_el);
//  Ubeta = Uq * _cos(angle_el);
//
//  // inverse clarke transformation
//  Ua = Ualpha + voltage_power_supply / 2;
//  Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
//  Uc = (- Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;
//
//  setPWM(Ua, Ub, Uc);
//}

/*
 * @brief Initializes motor with pole pair count & supply voltage.
 * @params[in] Motor* motor
 * @params[in] uint8_t pole_pairs
 * @params[in] float supply_voltage
 */
void MotorInit(Motor* motor, uint8_t pole_pairs, float supply_voltage)
{
	/* Set pole pairs & supply voltage */
	motor->params->pole_pairs = pole_pairs;
	motor->params->supply_voltage = supply_voltage;

	/* Initialize all values to 0 */
	motor->params->shaft_angle = 0;
	motor->params->zero_angle = 0;
	motor->phaseVs->Ua = 0;
	motor->phaseVs->Ub = 0;
	motor->phaseVs->Uc = 0;
	motor->qdVals->Ud = 0;
	motor->qdVals->Uq = 0;

}

/*
 * @brief Links a AS5600 sensor to a motor object
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

void DebugSensor(Motor* motor)
{
	return;
}
