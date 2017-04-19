
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "power_distribution.h"
#include "num.h"
#include "motors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "arm_math.h"

#define LQR_CONTROLLER_RATE RATE_500_HZ
#define limitThrust(VAL) limitUint16(VAL)
/**
 * Supporting and utility functions
 */
static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ 
  configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); 
}
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ 
  configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); 
}
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ 
  configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); 
}
static inline float arm_sqrt(float32_t in)
{ 
  float pOut = 0; 
  arm_status result = arm_sqrt_f32(in, &pOut); 
  configASSERT(ARM_MATH_SUCCESS == result); 
  return pOut; 
}

#define g 9.81f
#define m 27.0f/1000.0f //massa in kq
#define d (65.0538f/1000.0f)*0.707106781186547f  // distanza dal centro ai motori
#define c 0.1f          // inerzia delle eliche
#define fmotmax 0.5886f/4.0f  // max forza generata dai motori

static float Ug[4] = {m*g, 0, 0, 0};
static float UTMP[4][1] = {{0}};
static arm_matrix_instance_f32 UTMPm = {4, 1, (float *)UTMP};

static float matT[4][4] = {
  { 1, 1,  1, 1},
  {-d, -d, d, d},
  {d, -d, -d, d},
  {c, -c, c, -c}
};
static arm_matrix_instance_f32 matTm = {4, 4, (float *)matT};
static float matTinv[4][4] = {
  {1/4, -1/(4*d),  1/(4*d),  1/(4*c)},
  {1/4, -1/(4*d), -1/(4*d), -1/(4*c)},
  {1/4,  1/(4*d), -1/(4*d),  1/(4*c)},
  {1/4,  1/(4*d),  1/(4*d), -1/(4*c)}
};
static arm_matrix_instance_f32 matTinvm = {4, 4, (float *)matTinv};

static float K[4][13] = {
  {0.0, -3.24282575393e-27, -2.61836296913e-15, 3.71173266076e-17, 7.12271393172e-29, -1.55635080427e-17, 4.95218409287e-18, -5.15640553154e-15, -1.17027318486e-26, 0.158113883008, -1.22726317541e-15, 3.53485057167e-28, 0.0926723163658},
  {0.0, 0.269706789523, 1.87783277123e-14, 3.39277417633e-14, 0.0102888955743, 5.50471119082e-16, -3.57754512006e-15, 3.57470363361e-14, -0.316227766018, 5.46542676774e-26, 1.03809097574e-14, -0.0935097042236, 1.680685095e-26},
  {0.0, 8.91387505733e-14, 0.269706789523, 9.79453782528e-15, 5.50471119082e-16, 0.0102888955742, 1.03286143214e-15, 0.316227766017, -2.11176551901e-13, -1.36060293733e-14, 0.0935097042233, -5.25749055601e-14, -3.67238615e-15},
  {0.0, 1.70054013724e-14, 7.48421934889e-14, 0.00866025403785, -1.78875209188e-15, 5.16424806779e-16, 0.00877393485892, 1.02685955114e-12, 4.28474635822e-13, 1.98243623979e-15, 1.28804739616e-13, 3.6673186998e-14, 5.84255301785e-16},
};
static arm_matrix_instance_f32 Km = {4, 13, (float *)K};

static float State[13][1];
static float StateD[13][1];
static float StateTMP[13][1];
static arm_matrix_instance_f32 StateTMPm = {13, 1, (float *)StateTMP};
static float forces[4][1];
static arm_matrix_instance_f32 forcesm = {4, 1, (float *)forces};

float setpointThrust = 0;
float thrustM = 1.34f;
float rollM = 1.0f;
float pitchM = 1.0f;
float curPosX = 0;
float curPosY = 0;
float curPosZ = 0;
float setPosX = 0;
float setPosY = 0;
float setPosZ = 0;
float ctrRoll = 0;
float ctrPitch = 0;
float ctrThrust = 0;

void stateControllerLQR(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(LQR_CONTROLLER_RATE, tick)) {
    //Current State
    State[0][0] = state->attitudeQuaternion.w;
    State[1][0] = state->attitudeQuaternion.x;
    State[2][0] = state->attitudeQuaternion.y;
    State[3][0] = state->attitudeQuaternion.z;
    State[4][0] = sensors->gyro.x;
    State[5][0] = sensors->gyro.y;
    State[6][0] = sensors->gyro.z;
    State[7][0] = state->position.x;
    State[8][0] = state->position.y;
    State[9][0] = state->position.z;
    State[10][0] = state->velocity.x;
    State[11][0] = state->velocity.y;
    State[12][0] = state->velocity.z;
    curPosX = State[7][0];
    curPosY = State[8][0];
    curPosZ = State[9][0];

    //Desired State
    StateD[0][0] = 1.0f;
    StateD[1][0] = 0.0f;
    StateD[2][0] = 0.0f;
    StateD[3][0] = 0.0f;
    StateD[4][0] = 0.0f;
    StateD[5][0] = 0.0f;
    StateD[6][0] = 0.0f;
    StateD[7][0] = 0.64f;//setpoint->attitude.roll;
    StateD[8][0] = 1.11f;//setpoint->attitude.pitch;
    StateD[9][0] = 1.70f;//setpoint->attitudeRate.yaw;
    StateD[10][0] = 0.0f;
    StateD[11][0] = 0.0f;
    StateD[12][0] = 0.0f;
    setPosX = StateD[7][0];
    setPosY = StateD[8][0];
    setPosZ = StateD[9][0];

    for(int i = 0; i < 13; ++i){
      StateTMP[i][0] = (StateD[i][0] - State[i][0]);
    }
    mat_mult(&Km, &StateTMPm, &UTMPm);
    for(int i = 0; i < 4; ++i){
      UTMP[i][0] += Ug[i];
    }

    mat_mult(&matTinvm, &UTMPm, &forcesm);

    //limito le forze
    for(int i=0; i < 4; ++i){
      if(forces[i][0] < 0) 
        forces[i][0] = 0;
      if(forces[i][0] > fmotmax) 
        forces[i][0] = fmotmax;
    }

    //Calcolo l'ingresso saturato
    mat_mult(&matTm, &forcesm, &UTMPm);

    //Calcolo l'ingresso per i motori
    float scaleFactor = thrustM * 65536.0f / (fmotmax * 4.0f);
    UTMP[0][0] *= scaleFactor;
    UTMP[1][0] = (UTMP[1][0]/2.0f)/d;
    UTMP[2][0] = (UTMP[2][0]/2.0f)/d;
    UTMP[3][0] = 0/c;

    for(int i = 1; i < 4; ++i){
      if(UTMP[i][0] < -65535.0f){
          UTMP[i][0] = -65535.0f;
      }
      else if(UTMP[i][0] > 65535){
          UTMP[i][0] = 65535;
      }
    }

    float m1 = UTMP[0][0] - UTMP[1][0] + UTMP[2][0] + UTMP[3][0];
    float m2 = UTMP[0][0] - UTMP[1][0] - UTMP[2][0] - UTMP[3][0];
    float m3 = UTMP[0][0] + UTMP[1][0] - UTMP[2][0] + UTMP[3][0];
    float m4 = UTMP[0][0] + UTMP[1][0] + UTMP[2][0] - UTMP[3][0];

    float T =          (m1 + m2 + m3 + m4)/4.0f;
    float R =  rollM * (m4 + m3 - m2 - m1)*10.5f;
    float P = pitchM * (m1 + m4 - m2 - m3)*10.5f;

    ctrRoll = R;
    ctrPitch = P;
    ctrThrust = T;

    if(setpoint->thrust == 0.0f){
      T = 0;
      R = 0;
      P = 0;
    }
    setpointThrust = T;
    setpoint->thrust = T;
    setpoint->attitude.roll = R;
    setpoint->attitude.pitch = P;
    setpoint->attitudeRate.yaw = 0;

  }
}

LOG_GROUP_START(lqrCtrl)
LOG_ADD(LOG_FLOAT, setpointThrust, &setpointThrust)
LOG_ADD(LOG_FLOAT, curPosX, &curPosX)
LOG_ADD(LOG_FLOAT, curPosY, &curPosY)
LOG_ADD(LOG_FLOAT, curPosZ, &curPosZ)
LOG_ADD(LOG_FLOAT, setPosX, &setPosX)
LOG_ADD(LOG_FLOAT, setPosY, &setPosY)
LOG_ADD(LOG_FLOAT, setPosZ, &setPosZ)
LOG_ADD(LOG_FLOAT, ctrRoll, &ctrRoll)
LOG_ADD(LOG_FLOAT, ctrPitch, &ctrPitch)
LOG_ADD(LOG_FLOAT, ctrThrust, &ctrThrust)
LOG_GROUP_STOP(lqrCtrl)

PARAM_GROUP_START(lqrCtrl)
PARAM_ADD(PARAM_FLOAT, tM, &thrustM)  //thrust multiplicator
PARAM_ADD(PARAM_FLOAT, rM, &rollM)    //roll multiplicator
PARAM_ADD(PARAM_FLOAT, pM, &pitchM)   //pitch multiplicator
PARAM_GROUP_STOP(lqrCtrl)
