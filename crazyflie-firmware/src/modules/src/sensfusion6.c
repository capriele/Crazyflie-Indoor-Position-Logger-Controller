/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"

#include "sensfusion6.h"
#include "log.h"
#include "param.h"
 
#include "math.h"
#include "arm_math.h"

#define M_PI_F ((float) M_PI)

#define MADWICK_QUATERNION_IMU

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


#ifndef MADWICK_QUATERNION_IMU
    #define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
    #define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
#endif

#ifndef MADWICK_QUATERNION_IMU
  float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  // integral error terms scaled by Ki
#else
  #define STATE_SIZE 4
  #define OUTPUT_SIZE 6
  static float Q_VARIANCE = 0.01f;
  static float R_VARIANCE_ACC = 100.0f;
  static float R_VARIANCE_MAG = 500.0f;
  // The covariance matrix
  static float P[STATE_SIZE][STATE_SIZE] = {{0}};
  static float R[OUTPUT_SIZE] = {0};
  static arm_matrix_instance_f32 Pm = {STATE_SIZE, STATE_SIZE, (float *)P};

  // The state update matrix
  static float A[STATE_SIZE][STATE_SIZE];
  static arm_matrix_instance_f32 Am = {STATE_SIZE, STATE_SIZE, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_SIZE][STATE_SIZE];
  static arm_matrix_instance_f32 tmpNN1m = { STATE_SIZE, STATE_SIZE, (float *)tmpNN1d};

  static float tmpNN2d[STATE_SIZE][STATE_SIZE];
  static arm_matrix_instance_f32 tmpNN2m = { STATE_SIZE, STATE_SIZE, (float *)tmpNN2d};
#endif

// quaternion of sensor frame relative to auxiliary frame
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f; 

// Unit vector in the estimated gravity direction
static float gravX, gravY, gravZ; 

// The acc in Z for static position (g)
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
static float baseZacc = 1.0;

static bool isInit;

static bool isCalibrated = false;

static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
static float sensfusion6GetAccZ(const float ax, const float ay, const float az);
static void estimatedGravityDirection(float* gx, float* gy, float* gz);

// TODO: Make math util file
static float invSqrt(float x);

void sensfusion6Init()
{
  if(isInit)
    return;
#ifdef MADWICK_QUATERNION_IMU
  for(int i = 0; i < STATE_SIZE; ++i)
    P[i][i] = Q_VARIANCE;
  R[0] = R_VARIANCE_ACC;
  R[1] = R_VARIANCE_ACC;
  R[2] = R_VARIANCE_ACC;
  R[3] = R_VARIANCE_MAG;
  R[4] = R_VARIANCE_MAG;
  R[5] = R_VARIANCE_MAG;
#endif
  isInit = true;
}

bool sensfusion6Test(void)
{
  return isInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  //Cambio gli assi del magnetometro perchè rispetto a quelli del gyro/acc sono diversi:
  //In particolare:
  //Mx = (Gy = Ay)
  //My = (Gx = Ax)
  //-Mz = (Gz = Az)
  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);
  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}

#ifdef MADWICK_QUATERNION_IMU
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
/*
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  float recipNorm;

  gx = gx * M_PI_F / 180.0f;
  gy = gy * M_PI_F / 180.0f;
  gz = gz * M_PI_F / 180.0f;

  // Normalise accelerometer measurement
  recipNorm = invSqrt(ax*ax + ay*ay + az*az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Normalise magnetometer measurement
  recipNorm = invSqrt(mx*mx + my*my + mz*mz);
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  A[0][0] = 1.0f;
  A[0][1] = gz*dt;
  A[0][2] = -gy*dt;
  A[1][0] = -gz*dt;
  A[1][1] = 1.0f;
  A[1][2] = gx*dt;
  A[2][0] = gy*dt;
  A[2][1] = -gx*dt;
  A[2][2] = 1.0f;

  float q[4][1];
  arm_matrix_instance_f32 qm = {4, 1, (float *)q};
  q[0][0] = q0;
  q[1][0] = q1;
  q[2][0] = q2;
  q[3][0] = q3;

  float W[4][4];
  arm_matrix_instance_f32 Wm = {4, 4, (float *)W};
  W[0][0] = 0.0f;
  W[0][1] =  -gx;
  W[0][2] =  -gy;
  W[0][3] =  -gz;
  W[1][0] =   gx;
  W[1][1] = 0.0f;
  W[1][2] =   gz;
  W[1][3] =  -gy;
  W[2][0] =   gy;
  W[2][1] =  -gz;
  W[2][2] = 0.0f;
  W[2][3] =   gx;
  W[3][0] =   gz;
  W[3][1] =   gy;
  W[3][2] =  -gx;
  W[3][3] = 0.0f;

  //Aggiorno lo stato con runge kutta 4
  float m1[4][1];
  float m2[4][1];
  float m3[4][1];
  float x1[4][1];
  float x2[4][1];
  arm_matrix_instance_f32 m1m = {4, 1, (float *)m1};
  arm_matrix_instance_f32 m2m = {4, 1, (float *)m2};
  arm_matrix_instance_f32 m3m = {4, 1, (float *)m3};
  arm_matrix_instance_f32 x1m = {4, 1, (float *)x1};
  arm_matrix_instance_f32 x2m = {4, 1, (float *)x2};

  //Primo step
  mat_mult(&Wm, &qm, &m1m);
  for(int i = 0; i < 4; ++i){
    x1[i][0] = q[i][0] + 0.5f*m1[i][0]*dt;
  }

  //Secondo step
  mat_mult(&Wm, &x1m, &m2m);
  for(int i = 0; i < 4; ++i){
    x2[i][0] = q[i][0] + 0.5f*(m1[i][0] + m2[i][0])*dt/4.0f;
  }

  //Terzo step
  mat_mult(&Wm, &x2m, &m3m);
  for(int i = 0; i < 4; ++i){
    q[i][0] = q[i][0] + 0.5f*(m1[i][0] + m2[i][0] + 4.0f*m3[i][0])*(dt/6.0f);
  }

  //Normalizzo la stima dei quaternioni
  recipNorm = invSqrt(q[0][0]*q[0][0] + q[1][0]*q[1][0] + q[2][0]*q[2][0] + q[3][0]*q[3][0]);
  for(int i = 0; i < 4; ++i){
    q[i][0] *= recipNorm;
  }

  q0 = q[0][0];
  q1 = q[1][0];
  q2 = q[2][0];
  q3 = q[3][0];

  //Matrice di rotazione
  float QuaternionDCM[3][3];
  arm_matrix_instance_f32 QuaternionDCMm = {3, 3, (float *)QuaternionDCM};
  QuaternionDCM[0][0] = 2*q0*q0-1+2*q1*q1;
  QuaternionDCM[0][1] = 2*(q1*q2+q0*q3);
  QuaternionDCM[0][2] = 2*(q1*q3-q0*q2);
  QuaternionDCM[1][0] = 2*(q1*q2-q0*q3);
  QuaternionDCM[1][1] = 2*q0*q0-1+2*q2*q2;
  QuaternionDCM[1][2] = 2*(q2*q3+q0*q1);
  QuaternionDCM[2][0] = 2*(q1*q3+q0*q2);
  QuaternionDCM[2][1] = 2*(q2*q3-q0*q1);
  QuaternionDCM[2][2] = 2*q0*q0-1+2*q3*q3;

  //Gravity Reference
  float G[3][1];
  float Gr[3][1];
  arm_matrix_instance_f32 Gm = {3, 1, (float *)G};
  arm_matrix_instance_f32 Grm = {3, 1, (float *)Gr};
  G[0][0] = 0;
  G[1][0] = 0;
  G[2][0] = 1;
  mat_mult(&QuaternionDCMm, &Gm, &Grm);

  //Magnetic field Reference
  float M[3][1];
  float Mr[3][1];
  arm_matrix_instance_f32 Mm = {3, 1, (float *)M};
  arm_matrix_instance_f32 Mrm = {3, 1, (float *)Mr};
  float hx = mx*q0*q0 + 2.0f*mz*q0*q2 - 2.0f*my*q0*q3 + mx*q1*q1 + 2.0f*my*q1*q2 + 2.0f*mz*q1*q3 - mx*q2*q2 - mx*q3*q3;
  float hy = my*q0*q0 - 2.0f*mz*q0*q1 + 2.0f*mx*q0*q3 - my*q1*q1 + 2.0f*mx*q1*q2 + my*q2*q2 + 2.0f*mz*q2*q3 - my*q3*q3;
  float hz = mz*q0*q0 + 2.0f*my*q0*q1 - 2.0f*mx*q0*q2 - mz*q1*q1 + 2.0f*mx*q1*q3 - mz*q2*q1 + 2.0f*my*q2*q3 + mz*q3*q3;
  M[0][0] = arm_sqrt(hx*hx + hy*hy);
  M[1][0] = 0;
  M[2][0] = hz;
  mat_mult(&QuaternionDCMm, &Mm, &Mrm);

  // Aggiorno la Covarianza
  float P0[STATE_SIZE][STATE_SIZE];
  arm_matrix_instance_f32 P0m = {STATE_SIZE, STATE_SIZE, (float *)P0};

  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &P0m); // A P A'
  for(int i = 0; i < STATE_SIZE; ++i){
    P0[i][i] += Q_VARIANCE;
  }

  // Calcolo il guadagno
  float K[STATE_SIZE][OUTPUT_SIZE];
  arm_matrix_instance_f32 Km = {STATE_SIZE, OUTPUT_SIZE, (float *)K};

  float h[OUTPUT_SIZE][STATE_SIZE];
  arm_matrix_instance_f32 Hm = {OUTPUT_SIZE, STATE_SIZE, (float *)h};

  float hT[STATE_SIZE][OUTPUT_SIZE];
  arm_matrix_instance_f32 HTm = {STATE_SIZE, OUTPUT_SIZE, (float *)hT};

  float P0hT[STATE_SIZE][OUTPUT_SIZE];
  arm_matrix_instance_f32 P0HTm = {STATE_SIZE, OUTPUT_SIZE, (float *)P0hT};

  float hP0hT[OUTPUT_SIZE][OUTPUT_SIZE];
  arm_matrix_instance_f32 HP0HTm = {OUTPUT_SIZE, OUTPUT_SIZE, (float *)hP0hT};

  float hP0hT_inv[OUTPUT_SIZE][OUTPUT_SIZE];
  arm_matrix_instance_f32 HP0HT_INVm = {OUTPUT_SIZE, OUTPUT_SIZE, (float *)hP0hT_inv};

  //Accelerometer
  h[0][0] = 0;
  h[0][1] = -Gr[2][0];
  h[0][2] = Gr[1][0];
  h[1][0] = Gr[2][0];
  h[1][1] = 0;
  h[1][2] = -Gr[0][0];
  h[2][0] = -Gr[1][0];
  h[2][1] = Gr[0][0];
  h[2][2] = 0;

  //Magnetometer
  h[3][0] = 0;
  h[3][1] = -Mr[2][0];
  h[3][2] = Mr[1][0];
  h[4][0] = Mr[2][0];
  h[4][1] = 0;
  h[4][2] = -Mr[0][0];
  h[5][0] = -Mr[1][0];
  h[5][1] = Mr[0][0];
  h[5][2] = 0;


  // ====== INNOVATION COVARIANCE ======
  mat_trans(&Hm, &HTm);
  mat_mult(&P0m, &HTm, &P0HTm); // P0*H'
  mat_mult(&Hm, &P0HTm, &HP0HTm); // H*P0*H'
  for (int i = 0; i < OUTPUT_SIZE; ++i) { // Add the element of HPH' to the above
    hP0hT[i][i] += R[i];
  }
  mat_inv(&HP0HTm, &HP0HT_INVm); // (H*P0*H' + R)^(-1)
  mat_mult(&P0HTm, &HP0HT_INVm, &Km); // K = P0*H'*(H*P0*H' + R)^(-1)

  //Aggiorno la P
  mat_mult(&Km, &Hm, &tmpNN1m); // KH
  for (int i = 0; i < STATE_SIZE; ++i) { 
    tmpNN1d[i][i] -= 1.0f; 
    for (int j = 0; j < STATE_SIZE; ++j) {
      tmpNN1d[i][j] *= -1.0f; 
    }
  } // -(KH - I)
  mat_mult(&tmpNN1m, &P0m, &Pm); // -(KH - I)*P0

  float Error[OUTPUT_SIZE][1];
  arm_matrix_instance_f32 Errorm = {STATE_SIZE, 1, (float *)Error};
  float ae[STATE_SIZE][1];
  arm_matrix_instance_f32 aem = {STATE_SIZE, 1, (float *)ae};
  Error[0][0] = ax - Gr[0][0];
  Error[1][0] = ay - Gr[1][0];
  Error[2][0] = az - Gr[2][0];
  Error[3][0] = mx - Mr[0][0];
  Error[4][0] = my - Mr[1][0];
  Error[5][0] = mz - Mr[2][0];
  mat_mult(&Km, &Errorm, &aem); // K*error(k)

  float qe0 = 1; 
  float qe1 = ae[0][0]/2;
  float qe2 = ae[1][0]/2;
  float qe3 = ae[2][0]/2;

  float q0tmp = q0;
  float q1tmp = q1;
  float q2tmp = q2;
  float q3tmp = q3;
  q0 = q0tmp*qe0 - q1tmp*qe1 - q2tmp*qe2 - q3tmp*qe3;
  q1 = q0tmp*qe1 + q1tmp*qe0 + q2tmp*qe3 - q3tmp*qe2;
  q2 = q0tmp*qe2 + q2tmp*qe0 - q1tmp*qe3 + q3tmp*qe1;
  q3 = q0tmp*qe3 + q1tmp*qe2 - q2tmp*qe1 + q3tmp*qe0;

  // Normalise quaternion
  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
*/
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float recipNorm;

    gx = gx * M_PI_F / 180.0f;
    gy = gy * M_PI_F / 180.0f;
    gz = gz * M_PI_F / 180.0f;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Converto il vettore intensità campo magnetico
    //float h1 =  mx*q1 + my*q2 + mz*q3; 
    //float h2 =  mx*q0 - my*q3 + mz*q2; 
    //float h3 =  mx*q3 + my*q0 - mz*q1; 
    //float h4 = -mx*q2 + my*q1 + mz*q0; 
    //float n2 = q0*h2 + q1*h1 + q2*h4 - q3*h3;
    //float n3 = q0*h3 - q1*h4 + q2*h1 + q3*h2;
    //float n4 = q0*h4 + q1*h3 - q2*h2 + q3*h1;
    //float b2 = arm_sqrt(n2*n2 + n3*n3);
    //float b4 = n4;
    //float b2 = mx;
    //float b4 = mz;
    float hx = mx*q0*q0 + 2.0f*mz*q0*q2 - 2.0f*my*q0*q3 + mx*q1*q1 + 2.0f*my*q1*q2 + 2.0f*mz*q1*q3 - mx*q2*q2 - mx*q3*q3;
    float hy = my*q0*q0 - 2.0f*mz*q0*q1 + 2.0f*mx*q0*q3 - my*q1*q1 + 2.0f*mx*q1*q2 + my*q2*q2 + 2.0f*mz*q2*q3 - my*q3*q3;
    float hz = mz*q0*q0 + 2.0f*my*q0*q1 - 2.0f*mx*q0*q2 - mz*q1*q1 + 2.0f*mx*q1*q3 - mz*q2*q1 + 2.0f*my*q2*q3 + mz*q3*q3;
    float b2 = arm_sqrt(hx*hx + hy*hy);
    float b4 = hz;

    A[0][0] = 1.0f;
    A[0][1] = -gx*dt*0.5f;
    A[0][2] = -gy*dt*0.5f;
    A[0][3] = -gz*dt*0.5f;
    A[1][0] = gx*dt*0.5f;
    A[1][1] = 1.0f;
    A[1][2] = gz*dt*0.5f;
    A[1][3] = -gy*dt*0.5f;
    A[2][0] = gy*dt*0.5f;
    A[2][1] = -gz*dt*0.5f;
    A[2][2] = 1.0f;
    A[2][3] = gx*dt*0.5f;
    A[3][0] = gz*dt*0.5f;
    A[3][1] = gy*dt*0.5f;
    A[3][2] = -gx*dt*0.5f;
    A[3][3] = 1.0f;

    // Aggiorno la Covarianza
    float P0[STATE_SIZE][STATE_SIZE];
    arm_matrix_instance_f32 P0m = {STATE_SIZE, STATE_SIZE, (float *)P0};

    mat_mult(&Am, &Pm, &tmpNN1m); // A P
    mat_trans(&Am, &tmpNN2m); // A'
    mat_mult(&tmpNN1m, &tmpNN2m, &P0m); // A P A'
    for(int i = 0; i < STATE_SIZE; ++i){
      P0[i][i] += Q_VARIANCE;
    }

    // Calcolo il guadagno
    float K[STATE_SIZE][OUTPUT_SIZE];
    arm_matrix_instance_f32 Km = {STATE_SIZE, OUTPUT_SIZE, (float *)K};

    float h[OUTPUT_SIZE][STATE_SIZE];
    arm_matrix_instance_f32 Hm = {OUTPUT_SIZE, STATE_SIZE, (float *)h};

    float hT[STATE_SIZE][OUTPUT_SIZE];
    arm_matrix_instance_f32 HTm = {STATE_SIZE, OUTPUT_SIZE, (float *)hT};

    float P0hT[STATE_SIZE][OUTPUT_SIZE];
    arm_matrix_instance_f32 P0HTm = {STATE_SIZE, OUTPUT_SIZE, (float *)P0hT};

    float hP0hT[OUTPUT_SIZE][OUTPUT_SIZE];
    arm_matrix_instance_f32 HP0HTm = {OUTPUT_SIZE, OUTPUT_SIZE, (float *)hP0hT};

    float hP0hT_inv[OUTPUT_SIZE][OUTPUT_SIZE];
    arm_matrix_instance_f32 HP0HT_INVm = {OUTPUT_SIZE, OUTPUT_SIZE, (float *)hP0hT_inv};

    //Accelerometer
    h[0][0] = -2.0f*q2;
    h[0][1] =  2.0f*q3;
    h[0][2] = -2.0f*q0;
    h[0][3] =  2.0f*q1;
    h[1][0] =  2.0f*q1;
    h[1][1] =  2.0f*q0;
    h[1][2] =  2.0f*q3;
    h[1][3] =  2.0f*q2;
    h[2][0] =  4.0f*q0;
    h[2][1] =  0.0f;
    h[2][2] =  0.0f;
    h[2][3] =  4.0f*q3;
    //h[2][0] =  2.0f*q0;
    //h[2][1] = -2.0f*q1;
    //h[2][2] = -2.0f*q2;
    //h[2][3] =  2.0f*q3;

    //Magnetometer
    h[3][0] =  4.0f*b2*q0 - 2.0f*b4*q2;
    h[3][1] =  4.0f*b2*q1 + 2.0f*b4*q3;
    h[3][2] = -2.0f*b4*q0;
    h[3][3] =  2.0f*b4*q1;
    h[4][0] =  2.0f*b4*q1 - 2.0f*b2*q3;
    h[4][1] =  2.0f*b2*q2 + 2.0f*b4*q0;
    h[4][2] =  2.0f*b2*q1 + 2.0f*b4*q3;
    h[4][3] =  2.0f*b4*q2 - 2.0f*b2*q0;
    h[5][0] =  2.0f*b2*q2 + 4.0f*b4*q0;
    h[5][1] =  2.0f*b2*q3;
    h[5][2] =  2.0f*b2*q0;
    h[5][3] =  2.0f*b2*q1 + 4.0f*b4*q3;

    //h[3][0] = -2.0f*b4*q2;
    //h[3][1] =  2.0f*b4*q3;
    //h[3][2] = -4.0f*b2*q2 - 2.0f*b4*q0;
    //h[3][3] = -4.0f*b2*q3 + 2.0f*b4*q1;
    //h[4][0] = -2.0f*b2*q3 + 2.0f*b4*q1;
    //h[4][1] =  2.0f*b2*q2 + 2.0f*b4*q0;
    //h[4][2] =  2.0f*b2*q1 + 2.0f*b4*q3;
    //h[4][3] = -2.0f*b2*q0 + 2.0f*b4*q2;
    //h[5][0] =  2.0f*b2*q2;
    //h[5][1] =  2.0f*b2*q3 - 4.0f*b4*q1;
    //h[5][2] =  2.0f*b2*q0 - 4.0f*b4*q2;
    //h[5][3] =  2.0f*b2*q1;


    // ====== INNOVATION COVARIANCE ======
    mat_trans(&Hm, &HTm);
    mat_mult(&P0m, &HTm, &P0HTm); // P0*H'
    mat_mult(&Hm, &P0HTm, &HP0HTm); // H*P0*H'
    for (int i = 0; i < OUTPUT_SIZE; ++i) { // Add the element of HPH' to the above
      hP0hT[i][i] += R[i];
    }
    mat_inv(&HP0HTm, &HP0HT_INVm); // (H*P0*H' + R)^(-1)
    mat_mult(&P0HTm, &HP0HT_INVm, &Km); // K = P0*H'*(H*P0*H' + R)^(-1)

    //Aggiorno Stato
    float State[STATE_SIZE][1];
    arm_matrix_instance_f32 Statem = {STATE_SIZE, 1, (float *)State};
    float StateTMP[STATE_SIZE][1];
    arm_matrix_instance_f32 StateTMPm = {STATE_SIZE, 1, (float *)StateTMP};
    float Error[OUTPUT_SIZE][1];
    arm_matrix_instance_f32 Errorm = {STATE_SIZE, 1, (float *)Error};
    float ErrorTMP[OUTPUT_SIZE][1];
    arm_matrix_instance_f32 ErrorTMPm = {STATE_SIZE, 1, (float *)ErrorTMP};
    State[0][0] = q0;
    State[1][0] = q1;
    State[2][0] = q2;
    State[3][0] = q3;
    mat_mult(&Am, &Statem, &StateTMPm); // A*x(k)

    Error[0][0] = ax - 2.0f*(q1*q3 - q0*q2);
    Error[1][0] = ay - 2.0f*(q0*q1 + q2*q3);
    Error[2][0] = az - 2.0f*(q0*q0 + q3*q3 - 0.5f);
    Error[3][0] = mx - 2.0f*(b2*(q0*q0 + q1*q1 - 0.5f) - b4*(q0*q2 - q1*q3));
    Error[4][0] = my - 2.0f*(b4*(q0*q1 + q2*q3) - b2*(q0*q3 - q1*q2));
    Error[5][0] = mz - 2.0f*(b4*(q0*q0 + q3*q3 - 0.5f) + b2*(q0*q2 + q1*q3));
    mat_mult(&Km, &Errorm, &ErrorTMPm); // K*error(k)

    q0 = StateTMP[0][0] + ErrorTMP[0][0];
    q1 = StateTMP[1][0] + ErrorTMP[1][0];
    q2 = StateTMP[2][0] + ErrorTMP[2][0];
    q3 = StateTMP[3][0] + ErrorTMP[3][0];

    //Aggiorno la P
    mat_mult(&Km, &Hm, &tmpNN1m); // KH
    for (int i = 0; i < STATE_SIZE; ++i) { 
      tmpNN1d[i][i] -= 1.0f; 
      for (int j = 0; j < STATE_SIZE; ++j) {
        tmpNN1d[i][j] *= -1.0f; 
      }
    } // -(KH - I)
    mat_mult(&tmpNN1m, &P0m, &Pm); // -(KH - I)*P0

    // Normalise quaternion
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

#else // MAHONY_QUATERNION_IMU
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * M_PI_F / 180.0f;
  gy = gy * M_PI_F / 180.0f;
  gz = gz * M_PI_F / 180.0f;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
#endif

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;

  if (gx >  1) gx =  1;
  if (gx < -1) gx = -1;

  *yaw = atan2f(2.0f*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180.0f / M_PI_F;
  *pitch = asinf(gx) * 180.0f / M_PI_F; //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * 180.0f / M_PI_F;
}
void sensfusion6GetQuaternion(float* Q0, float* Q1, float* Q2, float* Q3)
{
  *Q0 = q0;
  *Q1 = q1;
  *Q2 = q2;
  *Q3 = q3;
}

float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

float sensfusion6GetInvThrustCompensationForTilt()
{
  // Return the z component of the estimated gravity direction
  // (0, 0, 1) dot G
  return gravZ;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float sensfusion6GetAccZ(const float ax, const float ay, const float az)
{
  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  return (ax * gravX + ay * gravY + az * gravZ);
}

static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (q1 * q3 - q0 * q2);
  *gy = 2 * (q0 * q1 + q2 * q3);
  *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

LOG_GROUP_START(sensorfusion6)
  LOG_ADD(LOG_FLOAT, qw, &q0)
  LOG_ADD(LOG_FLOAT, qx, &q1)
  LOG_ADD(LOG_FLOAT, qy, &q2)
  LOG_ADD(LOG_FLOAT, qz, &q3)
  LOG_ADD(LOG_FLOAT, gravityX, &gravX)
  LOG_ADD(LOG_FLOAT, gravityY, &gravY)
  LOG_ADD(LOG_FLOAT, gravityZ, &gravZ)
  LOG_ADD(LOG_FLOAT, accZbase, &baseZacc)
  LOG_ADD(LOG_UINT8, isInit, &isInit)
  LOG_ADD(LOG_UINT8, isCalibrated, &isCalibrated)
LOG_GROUP_STOP(sensorfusion6)

PARAM_GROUP_START(sensorfusion6)
#ifndef MADWICK_QUATERNION_IMU
PARAM_ADD(PARAM_FLOAT, kp, &twoKp)
PARAM_ADD(PARAM_FLOAT, ki, &twoKi)
#else
PARAM_ADD(PARAM_FLOAT, Q_VARIANCE, &Q_VARIANCE)
#endif
PARAM_ADD(PARAM_FLOAT, baseZacc, &baseZacc)
PARAM_GROUP_STOP(sensorfusion6)
