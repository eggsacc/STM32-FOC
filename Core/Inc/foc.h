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
 * @brief Macro to setup a motor.
 *        Sets up all structures on the stack, only leaving Motor->AS5600* sensor & timer as NULL.
 * @note Default to 12V for supply voltage, 6V voltage limit & 6 pole pairs
 *       All other struct values are initialized to 0
 *       Sensor will have to be attached manually using LinkSensor(); the default is open-loop control.
 */
#define MotorSetup(motor)                                                                \
	do {                                                                                 \
		static FOCparams motor##_params = {12, 6, 0, 0, 0, 0, 6};                        \
		static DQvalues motor##_dq = {0, 0};                                             \
		static PhaseVoltages motor##_pv = {0, 0, 0};                                     \
		static Motor motor = {&motor##_params, &motor##_dq, &motor##_pv, NULL, NULL};    \
		DWT_Init();                                                                      \
	} while(0)                                                                           \

/*
 * @brief Create & attach sensor object to motor object.
 */
#define AttachSensor(motor, i2c_handle)                                                                    \
	do{                                                                                                    \
		static AS5600 motor##_sensor;                                                                      \
		motor->sensor = (AS5600_Init(&(motor##_sensor), i2c_handle, 1)  == 0) ? &(motor##_sensor) : NULL;  \
	} while(0)                                                                                             \

/*
 * @brief PWM Timer starter macro.
 *        Starts the timer for channels 1-3.
 */
#define PWMstart(htimx)                              \
	do{                                              \
		HAL_TIM_PWM_Start(htimx, TIM_CHANNEL_1);  \
		HAL_TIM_PWM_Start(htimx, TIM_CHANNEL_2);  \
		HAL_TIM_PWM_Start(htimx, TIM_CHANNEL_3);  \
	} while(0)                                       \

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
