/*
 * foc.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

void setPWM(float Ua, float Ub, float Uc)
{

}
void setPhaseVoltage(float Uq, float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);

  // inverse park transformation
  Ualpha = -Uq * _sin(angle_el);
  Ubeta = Uq * _cos(angle_el);

  // inverse clarke transformation
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (- Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

  setPWM(Ua, Ub, Uc);
}
