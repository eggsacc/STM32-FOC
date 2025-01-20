/*
 * foc.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

/*
 * Includes
 */
#include "foc_utils.h"
#include "AS5600.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "timer_utils.h"

/*
 * @brief PWM Timer starter macro.
 *        Starts the timer for channels 1-3.
 */
void PWMstart(TIM_HandleTypeDef* timer);

/*
 * Typedef structures
 *
 * The library works by allocating (static) memory to each struct, then passing the struct pointers
 * around to update / use the stored values.
 *
 * The Motor structure contains pointers to all 3 other structures: FOCparams, QDvalues, and PhaseVoltages.
 * Also includes pointers to the sensor struct and timer typedef struct.
 */
typedef struct
{
	float supply_voltage;
	float motor_voltage_limit;
	float zero_angle;
	float shaft_angle;
	float electric_angle;
	uint32_t prev_us;
	uint8_t pole_pairs;
} FOCparams;

typedef struct
{
	float Uq;
	float Ud;
} DQvalues;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
} PhaseVoltages;

typedef struct
{
	FOCparams* params;
	DQvalues* dqVals;
	PhaseVoltages* phaseVs;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
} Motor;

/*
 * Public functions
 */
Motor MotorInit(TIM_HandleTypeDef* timer, float supply_voltage, uint8_t pole_pairs);

void LinkSensor(Motor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
void DebugSensor(Motor* motor);



/*
 * Control functions
 */
void OLVelocityControl(Motor* motor, float target_velocity);


#endif /* INC_FOC_H_ */
