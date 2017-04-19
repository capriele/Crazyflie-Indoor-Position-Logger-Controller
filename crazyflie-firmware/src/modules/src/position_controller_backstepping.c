/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"
#include "arm_math.h"
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

// Maximum roll/pitch angle permited
static float rpLimit  = 20;
static float errorLimit  = 1;
//static float motorInputFilter  = 1;

#define fmotmax 0.5886f/4.0f  // max forza generata dai motori
static float CRAZYFLIE_ARM_LENGTH = 0.046f; // m
//static float CRAZYFLIE_MASS = 29.3e-3f; // kg

//Drone Parameters
static const float m = 29.3e-3f;
static float d;
static float c = 0.005964552f;
static const float g = 9.81f;

//Current attitude quaternion
static float q[4] = {1,0,0,0};

//Attitude Quaternion Desired
static float qd[4] = {1,0,0,0};

//Attitude Quaternion Error
static float qe[4] = {1,0,0,0};

//Angular speed Error
static float we[4] = {0,0,0,0};

//Controller Gains
//Z
float c10 = 3.0f;
float c13 = 3.0f;
    
// X
float c8 = 3.0f;
float c11 = 3.0f;
    
// Y
float c9 = 3.0f;
float c12 = 3.0f;

//Roll
float cr1 = 40.0f;
float cr2 = 40.0f;

//Pitch
float cp1 = 40.0f;
float cp2 = 40.0f;

//Yaw
float cy1 = 20.0f;
float cy2 = 20.0f;

float T, R, P, Y, u1, u2, u3, u4, Ux, Uy;
float e8, e9, e10, desX, desY, desZ;

float limitError(float e){
  if( e > errorLimit )
    e = errorLimit;
  else if(e < - errorLimit)
    e = -errorLimit;
  return e;
}
void positionControllerInit(){}
void positionControllerResetAllPID(){}
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, const sensorData_t *sensors)
{
    bool enable = (setpoint->position.z != 0.0f);
    if (!enable)
    {
      *thrust = 0;
      attitude->roll  = 0;
      attitude->pitch = 0;
      return;
    }

    d = CRAZYFLIE_ARM_LENGTH*arm_sin_f32(PI/4.0f);

    // Euler Angles
    float x1 = state->attitudeQuaternion.w;  // q0
    float x2 = state->attitudeQuaternion.x;  // q1
    float x3 = state->attitudeQuaternion.y;  // q2
    float x4 = state->attitudeQuaternion.z;  // q3

    // Angular Speeds
    float x5 = sensors->gyro.x*PI/180.0f; // wx
    float x6 = sensors->gyro.y*PI/180.0f; // wy
    float x7 = sensors->gyro.z*PI/180.0f; // wz

    // Positions
    float x8 = state->position.x;  // x
    float x9 = state->position.y;  // y
    float x10 = state->position.z; // z

    // Speeds
    float x11 = state->velocity.x; // vx
    float x12 = state->velocity.y; // vy
    float x13 = state->velocity.z; // vz

    desX = setpoint->position.x;
    desY = setpoint->position.y;
    desZ = setpoint->position.z;

    //Z-Controller
    e10 = limitError(setpoint->position.z - x10);
    float e13 = limitError(x13 - setpoint->velocity.z - c10 * e10);
    u1 = m * (g + e10 + setpoint->acceleration.z - c13 * e13 + c10 * (setpoint->velocity.z - x13)) / (x1*x1 - x2*x2 - x3*x3 + x4*x4);

    //X-Y Controllers
    Ux = 0;
    Uy = 0;
    if(u1 != 0.0f){
      // X
      e8 = limitError(setpoint->position.x - x8);
      float e11 = limitError(x11 - setpoint->velocity.x - c8 * e8);
      Ux = m * (e8 + setpoint->acceleration.x - c11 * e11 + c8 * (setpoint->velocity.x - x11)) / (2.0f*u1);

      // Y
      e9 = limitError(setpoint->position.y - x9);
      float e12 = limitError(x12 - setpoint->velocity.y - c9 * e9);
      Uy = m * (e9 + setpoint->acceleration.y - c12 * e12 + c9 * (setpoint->velocity.y - x12)) / (2.0f*u1);
    }

    //Desired attitude quaternion
    qd[0] = 1.0f;
    qd[1] = -(Uy-x3*x4);///x1;
    qd[2] = (Ux-x2*x4);///x1;
    qd[3] = 0.0f;//desired yaw
    float norm = arm_sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
    qd[0] = qd[0]/norm; 
    qd[1] = qd[1]/norm; 
    qd[2] = qd[2]/norm; 
    qd[3] = qd[3]/norm;

    //Current attitude quaternion coniugate
    q[0] = x1;
    q[1] = -x2;
    q[2] = -x3;
    q[3] = -x4;

    //Compute quaternion error
    qe[0] = qd[0]*q[0] - qd[1]*q[1] - qd[2]*q[2] - qd[3]*q[3];
    qe[1] = qd[1]*q[0] + qd[0]*q[1] - qd[3]*q[2] + qd[2]*q[3];
    qe[2] = qd[2]*q[0] + qd[3]*q[1] + qd[0]*q[2] - qd[1]*q[3];
    qe[3] = qd[3]*q[0] - qd[2]*q[1] + qd[1]*q[2] + qd[0]*q[3];

    //Desired angular velocity
    qd[0] = 0.0f;
    qd[1] = -x5;
    qd[2] = -x6;
    qd[3] = -x7;
    norm = arm_sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
    if(norm != 0.0f){
      qd[0] = qd[0]/norm; 
      qd[1] = qd[1]/norm; 
      qd[2] = qd[2]/norm; 
      qd[3] = qd[3]/norm;
    }

    //Compute angular velocity quaternion error
    we[0] = qd[0]*qe[0] - qd[1]*qe[1] - qd[2]*qe[2] - qd[3]*qe[3];
    we[1] = qd[1]*qe[0] + qd[0]*qe[1] - qd[3]*qe[2] + qd[2]*qe[3];
    we[2] = qd[2]*qe[0] + qd[3]*qe[1] + qd[0]*qe[2] - qd[1]*qe[3];
    we[3] = qd[3]*qe[0] - qd[2]*qe[1] + qd[1]*qe[2] + qd[0]*qe[3];

    /*
    //Control 1
    //Roll
    float er1 = limitError(qe[1]);
    float er2 = limitError(x5 - we[1] - cr1 * er1);
    u2 = m * d * d * (er1 + x6 * x7 - cr2 * er2 + cr1 * (we[1] - x5));

    //Pitch
    float ep1 = limitError(qe[2]);
    float ep2 = limitError(x6 - we[2] - cp1 * ep1);
    u3 = m * d * d * (ep1 - x5 * x7 - cp2 * ep2 + cp1 * (we[2] - x6));

    //Yaw
    float ey1 = limitError(qe[3]);
    float ey2 = limitError(x7 - we[3] - cy1 * ey1);
    u4 = 2.0f * m * d * d * (ey1 - cy2 * ey2 + cy1 * (we[3] - x7));
    */

    /*
    Control 2 (Full Quaternion BackStepping)
    */
    float c4 = cy2;
    float c44 = cy1;
    float e4 = qe[3];
    float e44 = 0.5f * (-x3 * x5 + x2 * x6 + x1 * x7) - c4 * e4;
    float xd4d = we[3];

    float c3 = cp1;
    float c33 = cp2;
    float e3 = qe[2];
    float e33 = 0.5f * (x4 * x5 + x1 * x6 - x2 * x7) - c3 * e3;
    float xd3d = we[2];

    float c2 = cr1;
    float c22 = cr2;
    float e2 = qe[1];
    float e22 = 0.5f * (x1 * x5 - x4 * x6 + x3 * x7) - c2 * e2;
    float xd2d = we[1];

    float x1_2 = x1*x1;
    float x2_2 = x2*x2;
    float x3_2 = x3*x3;
    float x4_2 = x4*x4;
    float x5_2 = x5*x5;
    float x6_2 = x6*x6;
    float x7_2 = x7*x7;
    float x1_3 = x1_2*x1;
    float x2_3 = x2_2*x2;
    float x3_3 = x3_2*x3;
    float x4_3 = x4_2*x4;
    float x5_3 = x5_2*x5;
    float div = x1*(x1_2 + x2_2 + x3_2 + x4_2);
    float mult = d*d*m;

    u2 = 0;
    u3 = 0;
    u4 = 0;
    if(div != 0){
        u4 = (mult*(x4_3 * x6_2 - x4_3 * x5_2 + x4_3 * x7_2 + 4 * e4 * x1_2 + 4 * e4 * x4_2 - 2 * c4 * x1_3 * x7 + 4 * c4 * x1_2 * xd4d + 4 * c4 * x4_2 * xd4d - 2 * x1_3 * x5 * x6 - x1_3 * x4 * x5_3 + x1_2 * x4 * x6_2 + x2_2 * x4 * x5_2 + x1_2 * x4 * x7_2 + x2_2 * x4 * x6_2 + x3_2 * x4 * x5_2 + x2_2 * x4 * x7_2 + x3_2 * x4 * x6_2 + x3_2 * x4 * x7_2 + 4 * e2 * x1 * x3 - 4 * e3 * x1 * x2 + 4 * e2 * x2 * x4 + 4 * e3 * x3 * x4 - 4 * c44 * e44 * x1_2 - 4 * c44 * e44 * x4_2 - 4 * c22 * e22 * x1 * x3 - 4 * c22 * e22 * x2 * x4 + 4 * c33 * e33 * x1 * x2 - 4 * c33 * e33 * x3 * x4 + 4 * c2 * x1 * x3 * xd2d - 4 * c3 * x1 * x2 * xd3d + 4 * c2 * x2 * x4 * xd2d + 4 * c3 * x3 * x4 * xd3d - 2 * c2 * x1_2 * x3 * x5 + 2 * c3 * x1_2 * x2 * x6 - 2 * c2 * x1 * x3_2 * x7 - 2 * c3 * x1 * x2_2 * x7 - 2 * c4 * x1_2 * x2 * x6 + 2 * c4 * x1_2 * x3 * x5 + 2 * c2 * x2 * x4_2 * x6 - 2 * c3 * x3 * x4_2 * x5 - 2 * c4 * x1 * x4_2 * x7 - 2 * c4 * x2 * x4_2 * x6 + 2 * c4 * x3 * x4_2 * x5 + 2 * x1_2 * x2 * x5 * x7 - 2 * x1 * x4_2 * x5 * x6 + 2 * x2 * x4_2 * x5 * x7 - 2 * c2 * x1 * x2 * x4 * x5 + 2 * c3 * x1 * x2 * x4 * x5 + 2 * c2 * x1 * x3 * x4 * x6 - 2 * c3 * x1 * x3 * x4 * x6 - 2 * c2 * x2 * x3 * x4 * x7 + 2 * c3 * x2 * x3 * x4 * x7)) / div;
        u3 = (mult*(x3_3 * x5_2 + x3_3 * x6_2 + x3_3 * x7_2 + 4 * e3 * x1_2 + 4 * e3 * x3_2 - 2 * c3 * x1_3 * x6 + 4 * c3 * x1_2 * xd3d + 4 * c3 * x3_2 * xd3d - 2 * x1_3 * x5 * x7 + x1_3 * x3 * x5_3 + x1_2 * x3 * x6_2 + x2_2 * x3 * x5_2 + x1_2 * x3 * x7_2 + x2_2 * x3 * x6_2 - x3 * x4_2 * x5_2 + x2_2 * x3 * x7_2 + x3 * x4_2 * x6_2 + x3 * x4_2 * x7_2 - 4 * e2 * x1 * x4 + 4 * e2 * x2 * x3 + 4 * e4 * x1 * x2 + 4 * e4 * x3 * x4 - 4 * c33 * e33 * x1_2 - 4 * c33 * e33 * x3_2 + 4 * c22 * e22 * x1 * x4 - 4 * c22 * e22 * x2 * x3 - 4 * c44 * e44 * x1 * x2 - 4 * c44 * e44 * x3 * x4 - 4 * c2 * x1 * x4 * xd2d + 4 * c2 * x2 * x3 * xd2d + 4 * c4 * x1 * x2 * xd4d + 4 * c4 * x3 * x4 * xd4d + 2 * c2 * x1_2 * x4 * x5 - 2 * c2 * x1 * x4_2 * x6 - 2 * c3 * x1 * x3_2 * x6 + 2 * c3 * x1_2 * x2 * x7 - 2 * c3 * x1_2 * x4 * x5 - 2 * c4 * x1 * x2_2 * x6 - 2 * c2 * x2 * x3_2 * x7 - 2 * c4 * x1_2 * x2 * x7 + 2 * c3 * x2 * x3_2 * x7 - 2 * c3 * x3_2 * x4 * x5 + 2 * c4 * x3_2 * x4 * x5 - 2 * x1 * x2 * x4 * x5_2 - 2 * x1_2 * x2 * x5 * x6 - 2 * x1 * x3_2 * x5 * x7 - 2 * x1 * x4_2 * x5 * x7 - 2 * c2 * x1 * x2 * x3 * x5 + 2 * c4 * x1 * x2 * x3 * x5 + 2 * c2 * x1 * x3 * x4 * x7 + 2 * c2 * x2 * x3 * x4 * x6 - 2 * c4 * x1 * x3 * x4 * x7 - 2 * c4 * x2 * x3 * x4 * x6 - 2 * x1 * x3 * x4 * x5 * x6 + 2 * x2 * x3 * x4 * x5 * x7)) / (2 * div);
        u2 = (mult*(x2_3 * x5_2 + x2_3 * x6_2 + x2_3 * x7_2 + 4 * e2 * x1_2 + 4 * e2 * x2_2 - 2 * c2 * x1_3 * x5 + 4 * c2 * x1_2 * xd2d + 4 * c2 * x2_2 * xd2d + 2 * x1_3 * x6 * x7 + x1_3 * x2 * x5_3 + x1_2 * x2 * x6_2 + x2 * x3_2 * x5_2 + x1_2 * x2 * x7_2 + x2 * x3_2 * x6_2 - x2 * x4_2 * x5_2 + x2 * x3_2 * x7_2 + x2 * x4_2 * x6_2 + x2 * x4_2 * x7_2 + 4 * e3 * x1 * x4 + 4 * e3 * x2 * x3 - 4 * e4 * x1 * x3 + 4 * e4 * x2 * x4 - 4 * c22 * e22 * x1_2 - 4 * c22 * e22 * x2_2 - 4 * c33 * e33 * x1 * x4 - 4 * c33 * e33 * x2 * x3 + 4 * c44 * e44 * x1 * x3 - 4 * c44 * e44 * x2 * x4 + 4 * c3 * x1 * x4 * xd3d + 4 * c3 * x2 * x3 * xd3d - 4 * c4 * x1 * x3 * xd4d + 4 * c4 * x2 * x4 * xd4d - 2 * c2 * x1 * x2_2 * x5 - 2 * c2 * x1_2 * x3 * x7 + 2 * c2 * x1_2 * x4 * x6 - 2 * c3 * x1 * x4_2 * x5 - 2 * c4 * x1 * x3_2 * x5 - 2 * c2 * x2_2 * x3 * x7 + 2 * c2 * x2_2 * x4 * x6 - 2 * c3 * x1_2 * x4 * x6 + 2 * c3 * x2_2 * x3 * x7 + 2 * c4 * x1_2 * x3 * x7 - 2 * c4 * x2_2 * x4 * x6 + 2 * x1 * x3 * x4 * x5_2 + 2 * x1_2 * x3 * x5 * x6 + 2 * x1 * x2_2 * x6 * x7 + 2 * x1 * x3_2 * x6 * x7 + 2 * x1 * x4_2 * x6 * x7 + 2 * x2_2 * x4 * x5 * x7 - 2 * c3 * x1 * x2 * x3 * x6 + 2 * c4 * x1 * x2 * x3 * x6 + 2 * c3 * x1 * x2 * x4 * x7 - 2 * c3 * x2 * x3 * x4 * x5 - 2 * c4 * x1 * x2 * x4 * x7 + 2 * c4 * x2 * x3 * x4 * x5 - 2 * x1 * x2 * x3 * x5 * x7 - 2 * x1 * x2 * x4 * x5 * x6)) / (2 * div);
    }

    //Compute the forces
    /*
    float f1, f2, f3, f4;
    f1 = (u1 - u2/d + u3/d + u4/c)/4.0f;
    f2 = (u1 - u2/d - u3/d - u4/c)/4.0f;
    f3 = (u1 + u2/d - u3/d + u4/c)/4.0f;
    f4 = (u1 + u2/d + u3/d - u4/c)/4.0f;
    if(f1 < 0) f1 = 0;
    if(f2 < 0) f2 = 0;
    if(f3 < 0) f3 = 0;
    if(f4 < 0) f4 = 0;

    //Compute saturated inputs
    u1 = f1 + f2 + f3 + f4;
    u2 = -d*f1 - d*f2 + d*f3 + d*f4;
    u3 = d*f1 - d*f2 - d*f3 + d*f4;
    u4 = c*f1 - c*f2 + c*f3 - c*f4;
    */

    //Get control r-p-y-t
    float scaleFactor = 65535.0f / (fmotmax * 4.0f);
    u1 = abs(u1)*scaleFactor;
    u2 = u2/d;
    u3 = u3/d;
    u4 = u4/c;

    if(u2 < -65535.0f)
      u2 = -65535.0f;
    else if(u2 > 65535.0f)
      u2 = 65535.0f;

    if(u3 < -65535.0f)
      u3 = -65535.0f;
    else if(u3 > 65535.0f)
      u3 = 65535.0f;

    if(u4 < -65535.0f)
      u4 = -65535.0f;
    else if(u4 > 65535.0f)
      u4 = 65535.0f;

    //Motor input
    float m1 = u1 - u2 + u3 + u4;
    float m2 = u1 - u2 - u3 - u4;
    float m3 = u1 + u2 - u3 + u4;
    float m4 = u1 + u2 + u3 - u4;

    T = (m1 + m2 + m3 + m4)/4.0f;
    R = (m4 + m3 - m2 - m1);
    P = -(m1 + m4 - m2 - m3);
    Y = (m1 - m2 + m3 - m4)/4.0f;
    if(T > 65535.0f)
      T = 65535.0f;

    if(R > rpLimit)
      R = rpLimit;
    else if(R < -rpLimit)
      R = -rpLimit;

    if(P > rpLimit)
      P = rpLimit;
    else if(P < -rpLimit)
      P = -rpLimit;

    //Send Control commands 
    *thrust = T;
    attitude->roll  = R;
    attitude->pitch = P;
    attitude->yaw = 0;
}


LOG_GROUP_START(Bks)
LOG_ADD(LOG_FLOAT, u1, &u1)
LOG_ADD(LOG_FLOAT, u2, &u2)
LOG_ADD(LOG_FLOAT, u3, &u3)
LOG_ADD(LOG_FLOAT, u4, &u4)
LOG_ADD(LOG_FLOAT, Ux, &Ux)
LOG_ADD(LOG_FLOAT, Uy, &Uy)
LOG_ADD(LOG_FLOAT, e8, &e8)
LOG_ADD(LOG_FLOAT, e9, &e9)
LOG_ADD(LOG_FLOAT, e10, &e10)
LOG_ADD(LOG_FLOAT, desX, &desX)
LOG_ADD(LOG_FLOAT, desY, &desY)
LOG_ADD(LOG_FLOAT, desZ, &desZ)
LOG_GROUP_STOP(state)

PARAM_GROUP_START(posCtlBks)

PARAM_ADD(PARAM_FLOAT, kx1, &c8)
PARAM_ADD(PARAM_FLOAT, kx2, &c11)

PARAM_ADD(PARAM_FLOAT, ky1, &c9)
PARAM_ADD(PARAM_FLOAT, ky2, &c12)

PARAM_ADD(PARAM_FLOAT, kz1, &c10)
PARAM_ADD(PARAM_FLOAT, kz2, &c13)

PARAM_ADD(PARAM_FLOAT, kr1, &cr1)
PARAM_ADD(PARAM_FLOAT, kr2, &cr2)

PARAM_ADD(PARAM_FLOAT, kp1, &cp1)
PARAM_ADD(PARAM_FLOAT, kp2, &cp2)

PARAM_ADD(PARAM_FLOAT, kw1, &cy1)
PARAM_ADD(PARAM_FLOAT, kw2, &cy1)

PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, errorLimit,  &errorLimit)

PARAM_GROUP_STOP(posCtlBks)
