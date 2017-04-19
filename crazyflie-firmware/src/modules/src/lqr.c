/*
 * lqr.c
 *
 *  Created on: Feb 15, 2017
 *      Author: bitcraze
 */

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "sensors.h"
#include "math.h"
#include "num.h"
#include "lqr.h"
#include "motors.h"

// model constants
#define m 0.027f
// gain 0.002 ref 0.16 d 1 alpha 0.5
// gain 0.0014 ref 0.16 d 1 alpha 0.5
// how much thrust to hover? around 0.15 depending on battery
// change K matrix
#define g 9.81f

// global variables for the control
const int NREF = 4;
float referenceSignal[4] = {0, 0, 0, 0};
float controlSignal[] = {0, 0, 0, 0};
float controlDebugThrust[] = {0, 0, 0, 0};
float controlDebugPWM[] = {0, 0, 0, 0};
float stateEst[] = {0, 0, 0, 0, 0, 0};

// state variables
float rollDot = 0;
float pitchDot = 0;
float yawDot = 0;

float prevRoll = 0;
float prevPitch = 0;
float prevYaw = 0;

float prevRollDot = 0;
float prevPitchDot = 0;
float prevYawDot = 0;

// tuning parameters
float paramGain = 0;
float paramD = 1;
float paramAlpha = 0.9;
float paramRef = 0;
float paramStep = 0.005;

// k parameters
float Ka = 0.0255;
float Kb = 0.0085;
float Kc = 0;
float Kd=  0;
float paramK[] = {0.0255, 0.0085, 0, 0};

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static uint16_t limitThrust(int32_t value)
{
    return limitUint16(value);
}

float toRad(float degrees){
    return degrees * ((float) M_PI) / 180;
}

float thrustToPWM(float controlSignal){
    //old a -1.2205e6;
    //    b 5.9683e5;
    //    c 1.1357e3
    // new   8.1372e+05
    //       3.0676e+05
    //       659.2136
    float a = -1.2205e6;   // 8.1372e5;
    float b = 5.9683e5;    // 3.0676e5;
    float c = 1.1357e3;    // 659.2136;
    if (controlSignal > 0.15f) controlSignal = 0.15f;
    float pwm = (a * powf(controlSignal,2) + b * controlSignal + c);
    return pwm;
}

/* LQR controller
 * u = -K * x + Kr * r */
void LQRController(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick){
//void LQRController(float stateEst[], float refSignal[], float paramK[]){
        // get states
    float refSignal[] = {m*g/4.0f, m*g/4.0f, m*g/4.0f, m*g/4.0f};

    rollDot = sensors->gyro.x;
    pitchDot = sensors->gyro.y;
    yawDot = sensors->gyro.z;
    stateEst[0] = toRad(state->attitude.roll);
    stateEst[1] = paramD * rollDot;
    stateEst[2] = toRad(state->attitude.pitch);
    stateEst[3] = paramD * pitchDot;
    stateEst[4] = toRad(state->attitude.yaw);
    stateEst[5] = paramD * yawDot;

    int nCtrl = 4;
    int nRefs = 6;
    float a = paramK[0];
    float b = paramK[1];
    float c = paramK[2];
    float d = paramK[3];
    float K[4][6] = {{-a, -b,  a,  b,  c,  d},
            {-a, -b, -a, -b, -c, -d},
            { a,  b, -a, -b,  c,  d},
            { a,  b,  a,  b, -c, -d}};

    int i;
    int j;
    for (i = 0; i < nCtrl; i++){
        controlSignal[i] =  refSignal[i] * paramRef;
        for (j = 0; j < nRefs; j++){
            controlSignal[i]+= -K[i][j] * stateEst[j] *paramGain;
            controlDebugThrust[i] = controlSignal[i];
        }
        controlSignal[i] = thrustToPWM(controlSignal[i]);
        if (controlSignal[i] < 0){
            controlSignal[i] = 0;
        }
    }

    prevRoll = stateEst[0];
    prevPitch = stateEst[2];
    prevYaw = stateEst[4];

    prevRollDot = stateEst[1];
    prevPitchDot = stateEst[3];
    prevYawDot = stateEst[5];


    motorPowerM1 = limitThrust(fabs(controlSignal[0]));
    motorPowerM2 = limitThrust(fabs(controlSignal[1]));
    motorPowerM3 = limitThrust(fabs(controlSignal[2]));
    motorPowerM4 = limitThrust(fabs(controlSignal[3]));

    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M2, motorPowerM2);
    motorsSetRatio(MOTOR_M3, motorPowerM3);
    motorsSetRatio(MOTOR_M4, motorPowerM4);
}

