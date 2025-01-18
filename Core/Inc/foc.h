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

/*
 * Macro to setup a motor.
 * Sets up all structures on the stack, only leaving Motor->AS5600* sensor & timer as NULL.
 * Default to 12V for supply voltage, 6V voltage limit & 6 pole pairs
 * All other struct values are initialized to 0
 * Sensor will have to be attached manually using LinkSensor(); the default is open-loop control.
 */
#define MotorSetup(motor)                                                                \
	do {                                                                                 \
		FOCparams motor##_params = {12, 6, 0, 0, 0, 0, 6};                               \
		DQvalues motor##_dq = {0, 0};                                                    \
		PhaseVoltages motor##_pv = {0, 0, 0};                                            \
		Motor motor##_struct = {&motor##_params, &motor##_dq, &motor##_pv, NULL, NULL};  \
	} while(0)                                                                           \

#define AttachSensor(motor)                                   \
	AS5600

/*
 * Typedefs structures
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
	TIM_HandleTypeDef timer;
} Motor;

/*
 * Public functions
 */
void MotorInit(Motor* motor, uint8_t pole_pairs, float supply_voltage);

void LinkSensor(Motor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
void DebugSensor(Motor* motor);

/*
 * Control functions
 */
void OLVelocityControl(Motor* motor, float target_velocity);


#endif /* INC_FOC_H_ */
