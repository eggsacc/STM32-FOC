/*
 * foc_utils.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_UTILS_H_
#define INC_FOC_UTILS_H_

/*
 * Includes
 */
#include "stm32f1xx_hal.h"

/*
 * Constants definition
 */
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

/*
 * Constrain macro
 */
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
 * Trig approximation functions using look-up table
 */
float _sin(float angle);
float _cos(float angle);
float _normalizeAngle(float angle);
float _electricalAngle(float angle, uint8_t pole_pairs);

/*
 * Sine loop-up table: 16-bit depth, 8-bit resolution
 */
extern uint16_t sineLUT[];

#endif /* INC_FOC_UTILS_H_ */
