/*
 * foc_utils.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_UTILS_H_
#define INC_FOC_UTILS_H_

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
uint16_t sineLUT[65] = {0, 804, 1607, 2410, 3211, 4011, 4808, 5602, 6392, 7179, 7961, 8739, 9512, 10278, 11039, 11793,
12539, 13278, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594, 23170,
23732, 24279, 24812, 25330, 25832, 26319, 26790, 27245, 27684, 28106, 28511, 28898, 29269, 29621, 29956, 30273, 30572,
30852, 31114, 31357, 31581, 31785, 31971, 32138, 32285, 32413, 32521, 32610, 32679, 32728, 32758, 32768};

#endif /* INC_FOC_UTILS_H_ */
