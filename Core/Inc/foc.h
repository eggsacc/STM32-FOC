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

/*
 * Typedefs structures
 *
 * The library works by allocating (static) memory to each struct, then passing the struct pointers
 * around to update / use the stored values.
 *
 * The Motor structure contains pointers to all 3 other structures: FOCparams, QDvalues, and PhaseVoltages.
 */
typedef struct
{
	float supply_voltage;
	float zero_angle;
	float shaft_angle;
	float electric_angle;
	uint8_t pole_pairs;
}FOCparams;

typedef struct
{
	float Uq;
	float Ud;
}QDvalues;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
}PhaseVoltages;

typedef struct
{
	FOCparams* params;
	QDvalues* qdVals;
	PhaseVoltages* phaseVs;
}Motor;

/*
 * Public functions
 */
void MotorInit(Motor* motor, FOCparams* params, QDvalues* qdVals, PhaseVoltages* phaseVs);


#endif /* INC_FOC_H_ */
