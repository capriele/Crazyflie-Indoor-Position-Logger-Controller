/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "motors.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

int16_t t,r,p,y; 
void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    r = control->roll / 2.0f;
    p = control->pitch / 2.0f;
    y = control->yaw;
    t = control->thrust;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 = limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll  - control->yaw);
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + control->roll  - control->yaw);
  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}
void powerDistributionOffline(const control_float_t *control)
{
  motorPower.m1 = limitThrust(control->m1);
  motorPower.m2 = limitThrust(control->m2);
  motorPower.m3 = limitThrust(control->m3);
  motorPower.m4 = limitThrust(control->m4);
  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
}
void powerDistributionSetPoint(const setpoint_t *setpoint)
{
  motorPower.m1 = limitThrust(setpoint->attitude.roll);
  motorPower.m2 = limitThrust(setpoint->attitude.pitch);
  motorPower.m3 = limitThrust(setpoint->attitudeRate.yaw);
  motorPower.m4 = limitThrust(setpoint->thrust);
  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(motorForces)
LOG_ADD(LOG_INT32, t, &t)
LOG_ADD(LOG_INT32, r, &r)
LOG_ADD(LOG_INT32, p, &p)
LOG_ADD(LOG_INT32, y, &y)
LOG_GROUP_STOP(motor)
