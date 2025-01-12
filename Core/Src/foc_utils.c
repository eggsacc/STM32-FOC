/*
 * foc_utils.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include "foc_utils.h"
#include <math.h>

/*
 * @brief Sine approximation using look-up table
 * @param[in] angle(radians)
 * @return sin(angle)
 */
float _sin(float angle){

  int32_t first, second;
  uint16_t index = (uint16_t)angle / _2PI * 65536.0f;
  int frac = index & 0xff;
  index = (index >> 8) & 0xff;

  if (index < 64){
    first = (int32_t)sineLUP[index];
    second = (int32_t)sineLUP[index + 1];
  }
  else if (index < 128){
    first = (int32_t)sineLUP[128 - index];
    second = (int32_t)sineLUP[127 - index];
  }
  else if (index < 192){
    first = -(int32_t)sineLUP[index - 128];
    second = -(int32_t)sineLUP[index - 127];
  }
  else {
    first = -(int32_t)sineLUP[256 - index];
    second = -(int32_t)sineLUP[255 - index];
  }

  return (first + (((second - first) * frac) >> 8)) / 32768.0f;
}

/*
 * @brief Cosine approximation
 * @param[in] angle(radians)
 * @return cos(angle)
 */
float _cos(float angle) {
  float _angle = angle + _PI_2;
  _angle = _angle > _2PI ? _angle - _2PI : _angle;
  return _sin(_angle);
}

/*
 * @brief Calculates electrical angle from rotor angle
 * @param[in] shaft_angle(radians)
 * @param[in] pole_pairs
 * @return electrical angle
 */
float _electricalAngle(float shaft_angle, int pole_pairs){
  return(shaft_angle * pole_pairs);
}

/*
 * @brief Normalize angle to [0, 2pi]
 * @param[in] angle(radians)
 * @return normalized_angle
 */
float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);       // fmod(x,y) returns remainder of x/y
  return a >= 0 ? a : (a + _2PI);    // add 2pi to negative angles to make positive
}
