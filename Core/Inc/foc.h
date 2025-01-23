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
 * Typedef structures
 *
 * The library works by allocating (static) memory to each struct, then passing the struct pointers
 * around to update / use the stored values.
 *
 * The Motor structure contains motor parameters and pointers to all 3 other structures: Var_t, QDval_t, and PhaseV_t.
 * Also includes pointers to the sensor struct and timer handle struct.
 */
typedef struct
{
	float supply_voltage;
	float zero_angle;
	float shaft_angle;
	float electric_angle;
	uint32_t prev_us;
} Var_t;

typedef struct
{
	float Uq;
	float Ud;
} DQval_t;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
} PhaseV_t;

typedef struct
{
	uint8_t pole_pairs;
	uint16_t kv;
	float motor_v_limit;

	Var_t* vars;
	DQval_t* dqVals;
	PhaseV_t* phaseVs;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
} Motor_t;

/*
 * Public functions
 */
// void DebugSensor(Motor_t* motor);
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer);

Motor_t MotorInit(TIM_HandleTypeDef* timer, float supply_voltage, uint8_t pole_pairs);
void LinkSensor(Motor_t* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);

/*
 * Control functions
 */
void OLVelocityControl(Motor_t* motor, float target_velocity);


#endif /* INC_FOC_H_ */
