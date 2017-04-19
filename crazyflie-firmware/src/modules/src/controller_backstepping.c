/*
Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
__author__ = "Alberto Petrucci"
__copyright__ = "Copyright 2017, Alberto Petrucci"
__credits__ = ["Alberto Petrucci"]
__license__ = "Apache"
__version__ = "1.0.0"
__maintainer__ = "Alberto Petrucci"
__email__ = "petrucci.alberto@gmail.com"
__status__ = "Production"
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
#include "physical_constants.h"
#include "position_controller.h"

// Maximum roll/pitch angle permited
//static float rpLimit  = 20;
static float errorLimit  = 1;
//static float motorInputFilter  = 1;

//Drone Parameters
static float d;
//static float c = 0.005964552f;
static const float g = 9.81f;

//Attitude Quaternion Desired
static float qd[4] = {1,0,0,0};

//Current attitude quaternion
static float q[4] = {1,0,0,0};

//Attitude Quaternion Error
static float qe[4] = {1,0,0,0};
/*
//Angular speed Error
static float we[4] = {0,0,0,0};
*/
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

float u1, u2, u3, u4, Ux, Uy;
float e8, e9, e10;
float u1last = 0.0f, u2last = 0.0f, u3last = 0.0f, u4last = 0.0f;

//Disturbances compensation variables
float alpha = 0.999f;
float alphaw = 0.999f;
float L1 = 10.0f;
float L2 = 20.0f;
float z1x = 0, z2x = 0, z1y = 0, z2y = 0, z1z = 0, z2z = 0, z1wx = 0, z2wx = 0, z1wy = 0, z2wy = 0, z1wz = 0, z2wz = 0;
float dx = 0; 
float dy = 0;
float dz = 0; 
float dwx = 0; 
float dwy = 0;
float dwz = 0; 
float dt = 1.0f/POSITION_RATE; 

float limitError(float e){
  if( e > errorLimit )
    e = errorLimit;
  else if(e < - errorLimit)
    e = -errorLimit;
  return e;
}

void stateControllerInit(void){}
bool stateControllerTest(void){return true;}
void stateControllerBackStepping(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
  control->enable = setpoint->enable;
  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    if (!control->enable)
    {
      control->thrust = 0;
      control->torque[0] = 0;
      control->torque[1] = 0;
      control->torque[2] = 0;

      setpoint->velocity.x = 0;
      setpoint->velocity.y = 0;
      setpoint->velocity.z = 0;
      setpoint->acceleration.x = 0;
      setpoint->acceleration.y = 0;
      setpoint->acceleration.z = 0;
      return;
    }

    d = CRAZYFLIE_ARM_LENGTH*arm_sin_f32(PI/4.0f);

    // Euler Angles
    float x1 = state->attitudeQuaternion.w;  // q0
    float x2 = state->attitudeQuaternion.x;  // q1
    float x3 = state->attitudeQuaternion.y;  // q2
    float x4 = state->attitudeQuaternion.z;  // q3

    // Angular Speeds
    float x5 = radians(sensors->gyro.x); // wx
    float x6 = radians(sensors->gyro.y); // wy
    float x7 = radians(sensors->gyro.z); // wz

    // Positions
    float x8 = state->position.x;  // x
    float x9 = state->position.y;  // y
    float x10 = state->position.z; // z

    // Speeds
    float x11 = state->velocity.x; // vx
    float x12 = state->velocity.y; // vy
    float x13 = state->velocity.z; // vz

    //Disturbances estimations
    //Disturbo su vx
    float error = z1x - x11;
    z1x = z1x - L1*dt*error + dt*z2x;
    z2x = z2x - L2*dt*error;
    dx = alpha*dx + (1.0f-alpha)*(z2x - 2.0f*(x2*x4+x1*x3)*u1/CRAZYFLIE_MASS)*CRAZYFLIE_MASS;
    
    //Disturbo su vy
    error = z1y - x12;
    z1y = z1y - L1*dt*error + dt*z2y;
    z2y = z2y - L2*dt*error;
    dy = alpha*dy + (1.0f-alpha)*(z2y - 2.0f*(x3*x4-x1*x2)*u1/CRAZYFLIE_MASS)*CRAZYFLIE_MASS;
    
    //Disturbo su vz
    error = z1z - x13;
    z1z = z1z - L1*dt*error + dt*z2z;
    z2z = z2z - L2*dt*error;
    dz = alpha*dz + (1.0f-alpha)*(z2z + g - (x1*x1-x2*x2-x3*x3+x4*x4)*u1/CRAZYFLIE_MASS)*CRAZYFLIE_MASS;
    
    //Disturbo su wx
    error = z1wx - x5;
    z1wx = z1wx - L1*dt*error + dt*z2wx;
    z2wx = z2wx - L2*dt*error;
    dwx = alphaw*dwx + (1.0f-alphaw)*(z2wx + x6*x7 - u2/(CRAZYFLIE_MASS*d*d))*CRAZYFLIE_MASS*d*d;
    
    //Disturbo su wy
    error = z1wy - x6;
    z1wy = z1wy - L1*dt*error + dt*z2wy;
    z2wy = z2wy - L2*dt*error;
    dwy = alphaw*dwy + (1.0f-alphaw)*(z2wy - x5*x7 - u3/(CRAZYFLIE_MASS*d*d))*CRAZYFLIE_MASS*d*d;
    
    //Disturbo su wz
    error = z1wz - x7;
    z1wz = z1wz - L1*dt*error + dt*z2wz;
    z2wz = z2wz - L2*dt*error;
    dwz = alphaw*dwz + (1.0f-alphaw)*(z2wz - u4/(2.0f*CRAZYFLIE_MASS*d*d))*2.0f*CRAZYFLIE_MASS*d*d;

    //Z-Controller
    e10 = limitError(setpoint->position.z - x10);
    float e13 = limitError(x13 - setpoint->velocity.z - c10 * e10);
    u1 = CRAZYFLIE_MASS * (g + e10 + setpoint->acceleration.z - dz / CRAZYFLIE_MASS - c13 * e13 + c10 * (setpoint->velocity.z - x13)) / (x1*x1 - x2*x2 - x3*x3 + x4*x4);

    //X-Y Controllers
    Ux = 0;
    Uy = 0;
    if(u1 != 0.0f){
      // X
      e8 = limitError(setpoint->position.x - x8);
      float e11 = limitError(x11 - setpoint->velocity.x - c8 * e8);
      Ux = CRAZYFLIE_MASS * (e8 + setpoint->acceleration.x - dx / CRAZYFLIE_MASS - c11 * e11 + c8 * (setpoint->velocity.x - x11)) / (2.0f*u1);

      // Y
      e9 = limitError(setpoint->position.y - x9);
      float e12 = limitError(x12 - setpoint->velocity.y - c9 * e9);
      Uy = CRAZYFLIE_MASS * (e9 + setpoint->acceleration.y - dy / CRAZYFLIE_MASS - c12 * e12 + c9 * (setpoint->velocity.y - x12)) / (2.0f*u1);
    }

    //Desired attitude quaternion
    qd[0] = 1.0f;
    qd[1] = -(Uy-x3*x4)/x1;
    qd[2] = (Ux-x2*x4)/x1;
    qd[3] = 0.0f;//desired yaw
    //float norm = arm_sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
    //qd[0] = qd[0]/norm; 
    //qd[1] = qd[1]/norm; 
    //qd[2] = qd[2]/norm; 
    //qd[3] = qd[3]/norm;

    //Current attitude quaternion coniugate
    q[0] = x1;
    q[1] = -x2;
    q[2] = -x3;
    q[3] = -x4;

    //Compute quaternion error
    qe[0] = q[0]*qd[0] - q[1]*qd[1] - q[2]*qd[2] - q[3]*qd[3];
    qe[1] = q[1]*qd[0] + q[0]*qd[1] - q[3]*qd[2] + q[2]*qd[3];
    qe[2] = q[2]*qd[0] + q[3]*qd[1] + q[0]*qd[2] - q[1]*qd[3];
    qe[3] = q[3]*qd[0] - q[2]*qd[1] + q[1]*qd[2] + q[0]*qd[3];
    float norm = arm_sqrt(qe[0]*qe[0] + qe[1]*qe[1] + qe[2]*qe[2] + qe[3]*qe[3]);
    qe[0] = qe[0]/norm; 
    qe[1] = qe[1]/norm; 
    qe[2] = qe[2]/norm; 
    qe[3] = qe[3]/norm;

    /*
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
    */

    /*
    //Roll
    float er1 = limitError(-Uy - state->attitude.roll);//limitError(qe[1]);
    float er2 = limitError(x5 - we[1] - cr1 * er1);
    u2 = CRAZYFLIE_MASS * d * d * (er1 + x6 * x7 - cr2 * er2 + cr1 * (we[1] - x5));

    //Pitch
    float ep1 = limitError(Ux - state->attitude.pitch);//limitError(qe[2]);
    float ep2 = limitError(x6 - we[2] - cp1 * ep1);
    u3 = CRAZYFLIE_MASS * d * d * (ep1 - x5 * x7 - cp2 * ep2 + cp1 * (we[2] - x6));

    //Yaw
    float ey1 = limitError(0 - state->attitude.yaw);//limitError(qe[3]);
    float ey2 = limitError(x7 - we[3] - cy1 * ey1);
    u4 = 2.0f * CRAZYFLIE_MASS * d * d * (ey1 - cy2 * ey2 + cy1 * (we[3] - x7));
    */

    /*
    Control 2 (Full Quaternion BackStepping)
    */
    float c4 = cy2;
    float c44 = cy1;
    float e4 = qe[3];//qd[3] - x4;
    float e44 = 0.5f * (-x3 * x5 + x2 * x6 + x1 * x7) - c4 * e4;
    float xd4d = 0.0f;

    float c3 = cp1;
    float c33 = cp2;
    float e3 = qe[2];//qd[2] - x3;
    float e33 = 0.5f * (x4 * x5 + x1 * x6 - x2 * x7) - c3 * e3;
    float xd3d = 0.0f;

    float c2 = cr1;
    float c22 = cr2;
    float e2 = qe[1];//qd[1] - x2;
    float e22 = 0.5f * (x1 * x5 - x4 * x6 + x3 * x7) - c2 * e2;
    float xd2d = 0.0f;

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
    float div = x1*(x1_2 + x2_2 + x3_2 + x4_2);
    float d_2 = d*d;
    float m = CRAZYFLIE_MASS;

    u2 = 0;
    u3 = 0;
    u4 = 0;
    if(div != 0){
        //Senza compensazione disturbi sui momenti
        //u4 = (mult*(x4_3*x6_2 - x4_3*x5_2 + x4_3*x7_2 + 4*e4*x1_2 + 4*e4*x4_2 - 2*c4*x1_3*x7 + 4*c4*x1_2*xd4d + 4*c4*x4_2*xd4d - 2*x1_3*x5*x6 - x1_2*x4*x5_2 + x1_2*x4*x6_2 + x2_2*x4*x5_2 + x1_2*x4*x7_2 + x2_2*x4*x6_2 + x3_2*x4*x5_2 + x2_2*x4*x7_2 + x3_2*x4*x6_2 + x3_2*x4*x7_2 + 4*e2*x1*x3 - 4*e3*x1*x2 + 4*e2*x2*x4 + 4*e3*x3*x4 - 4*c44*e44*x1_2 - 4*c44*e44*x4_2 - 4*c22*e22*x1*x3 - 4*c22*e22*x2*x4 + 4*c33*e33*x1*x2 - 4*c33*e33*x3*x4 + 4*c2*x1*x3*xd2d - 4*c3*x1*x2*xd3d + 4*c2*x2*x4*xd2d + 4*c3*x3*x4*xd3d - 2*c2*x1_2*x3*x5 + 2*c3*x1_2*x2*x6 - 2*c2*x1*x3_2*x7 - 2*c3*x1*x2_2*x7 - 2*c4*x1_2*x2*x6 + 2*c4*x1_2*x3*x5 + 2*c2*x2*x4_2*x6 - 2*c3*x3*x4_2*x5 - 2*c4*x1*x4_2*x7 - 2*c4*x2*x4_2*x6 + 2*c4*x3*x4_2*x5 + 2*x1_2*x2*x5*x7 - 2*x1*x4_2*x5*x6 + 2*x2*x4_2*x5*x7 - 2*c2*x1*x2*x4*x5 + 2*c3*x1*x2*x4*x5 + 2*c2*x1*x3*x4*x6 - 2*c3*x1*x3*x4*x6 - 2*c2*x2*x3*x4*x7 + 2*c3*x2*x3*x4*x7))/div;
        //u3 = (mult*(x3_3*x5_2 + x3_3*x6_2 + x3_3*x7_2 + 4*e3*x1_2 + 4*e3*x3_2 - 2*c3*x1_3*x6 + 4*c3*x1_2*xd3d + 4*c3*x3_2*xd3d - 2*x1_3*x5*x7 + x1_2*x3*x5_2 + x1_2*x3*x6_2 + x2_2*x3*x5_2 + x1_2*x3*x7_2 + x2_2*x3*x6_2 - x3*x4_2*x5_2 + x2_2*x3*x7_2 + x3*x4_2*x6_2 + x3*x4_2*x7_2 - 4*e2*x1*x4 + 4*e2*x2*x3 + 4*e4*x1*x2 + 4*e4*x3*x4 - 4*c33*e33*x1_2 - 4*c33*e33*x3_2 + 4*c22*e22*x1*x4 - 4*c22*e22*x2*x3 - 4*c44*e44*x1*x2 - 4*c44*e44*x3*x4 - 4*c2*x1*x4*xd2d + 4*c2*x2*x3*xd2d + 4*c4*x1*x2*xd4d + 4*c4*x3*x4*xd4d + 2*c2*x1_2*x4*x5 - 2*c2*x1*x4_2*x6 - 2*c3*x1*x3_2*x6 + 2*c3*x1_2*x2*x7 - 2*c3*x1_2*x4*x5 - 2*c4*x1*x2_2*x6 - 2*c2*x2*x3_2*x7 - 2*c4*x1_2*x2*x7 + 2*c3*x2*x3_2*x7 - 2*c3*x3_2*x4*x5 + 2*c4*x3_2*x4*x5 - 2*x1*x2*x4*x5_2 - 2*x1_2*x2*x5*x6 - 2*x1*x3_2*x5*x7 - 2*x1*x4_2*x5*x7 - 2*c2*x1*x2*x3*x5 + 2*c4*x1*x2*x3*x5 + 2*c2*x1*x3*x4*x7 + 2*c2*x2*x3*x4*x6 - 2*c4*x1*x3*x4*x7 - 2*c4*x2*x3*x4*x6 - 2*x1*x3*x4*x5*x6 + 2*x2*x3*x4*x5*x7))/(2*div);
        //u2 = (mult*(x2_3*x5_2 + x2_3*x6_2 + x2_3*x7_2 + 4*e2*x1_2 + 4*e2*x2_2 - 2*c2*x1_3*x5 + 4*c2*x1_2*xd2d + 4*c2*x2_2*xd2d + 2*x1_3*x6*x7 + x1_2*x2*x5_2 + x1_2*x2*x6_2 + x2*x3_2*x5_2 + x1_2*x2*x7_2 + x2*x3_2*x6_2 - x2*x4_2*x5_2 + x2*x3_2*x7_2 + x2*x4_2*x6_2 + x2*x4_2*x7_2 + 4*e3*x1*x4 + 4*e3*x2*x3 - 4*e4*x1*x3 + 4*e4*x2*x4 - 4*c22*e22*x1_2 - 4*c22*e22*x2_2 - 4*c33*e33*x1*x4 - 4*c33*e33*x2*x3 + 4*c44*e44*x1*x3 - 4*c44*e44*x2*x4 + 4*c3*x1*x4*xd3d + 4*c3*x2*x3*xd3d - 4*c4*x1*x3*xd4d + 4*c4*x2*x4*xd4d - 2*c2*x1*x2_2*x5 - 2*c2*x1_2*x3*x7 + 2*c2*x1_2*x4*x6 - 2*c3*x1*x4_2*x5 - 2*c4*x1*x3_2*x5 - 2*c2*x2_2*x3*x7 + 2*c2*x2_2*x4*x6 - 2*c3*x1_2*x4*x6 + 2*c3*x2_2*x3*x7 + 2*c4*x1_2*x3*x7 - 2*c4*x2_2*x4*x6 + 2*x1*x3*x4*x5_2 + 2*x1_2*x3*x5*x6 + 2*x1*x2_2*x6*x7 + 2*x1*x3_2*x6*x7 + 2*x1*x4_2*x6*x7 + 2*x2_2*x4*x5*x7 - 2*c3*x1*x2*x3*x6 + 2*c4*x1*x2*x3*x6 + 2*c3*x1*x2*x4*x7 - 2*c3*x2*x3*x4*x5 - 2*c4*x1*x2*x4*x7 + 2*c4*x2*x3*x4*x5 - 2*x1*x2*x3*x5*x7 - 2*x1*x2*x4*x5*x6))/(2*div);
        
        //Compensazione disturbi
        u4 = (d_2*m*x4_3*x6_2 - dwz*x1*x2_2 - dwz*x1*x3_2 - dwz*x1*x4_2 - d_2*m*x4_3*x5_2 - dwz*x1_3 + d_2*m*x4_3*x7_2 + 4*d_2*e4*m*x1_2 + 4*d_2*e4*m*x4_2 - 4*c44*d_2*e44*m*x1_2 - 4*c44*d_2*e44*m*x4_2 - 2*c4*d_2*m*x1_3*x7 + 4*c4*d_2*m*x1_2*xd4d + 4*c4*d_2*m*x4_2*xd4d - 2*d_2*m*x1_3*x5*x6 - d_2*m*x1_2*x4*x5_2 + d_2*m*x1_2*x4*x6_2 + d_2*m*x2_2*x4*x5_2 + d_2*m*x1_2*x4*x7_2 + d_2*m*x2_2*x4*x6_2 + d_2*m*x3_2*x4*x5_2 + d_2*m*x2_2*x4*x7_2 + d_2*m*x3_2*x4*x6_2 + d_2*m*x3_2*x4*x7_2 + 4*d_2*e2*m*x1*x3 - 4*d_2*e3*m*x1*x2 + 4*d_2*e2*m*x2*x4 + 4*d_2*e3*m*x3*x4 - 4*c22*d_2*e22*m*x1*x3 - 4*c22*d_2*e22*m*x2*x4 + 4*c33*d_2*e33*m*x1*x2 - 4*c33*d_2*e33*m*x3*x4 + 4*c2*d_2*m*x1*x3*xd2d - 4*c3*d_2*m*x1*x2*xd3d + 4*c2*d_2*m*x2*x4*xd2d + 4*c3*d_2*m*x3*x4*xd3d - 2*c2*d_2*m*x1_2*x3*x5 + 2*c3*d_2*m*x1_2*x2*x6 - 2*c2*d_2*m*x1*x3_2*x7 - 2*c3*d_2*m*x1*x2_2*x7 - 2*c4*d_2*m*x1_2*x2*x6 + 2*c4*d_2*m*x1_2*x3*x5 + 2*c2*d_2*m*x2*x4_2*x6 - 2*c3*d_2*m*x3*x4_2*x5 - 2*c4*d_2*m*x1*x4_2*x7 - 2*c4*d_2*m*x2*x4_2*x6 + 2*c4*d_2*m*x3*x4_2*x5 + 2*d_2*m*x1_2*x2*x5*x7 - 2*d_2*m*x1*x4_2*x5*x6 + 2*d_2*m*x2*x4_2*x5*x7 - 2*c2*d_2*m*x1*x2*x4*x5 + 2*c3*d_2*m*x1*x2*x4*x5 + 2*c2*d_2*m*x1*x3*x4*x6 - 2*c3*d_2*m*x1*x3*x4*x6 - 2*c2*d_2*m*x2*x3*x4*x7 + 2*c3*d_2*m*x2*x3*x4*x7)/div;
        u3 = (d_2*m*x3_3*x5_2 - 2*dwy*x1*x2_2 - 2*dwy*x1*x3_2 - 2*dwy*x1*x4_2 - 2*dwy*x1_3 + d_2*m*x3_3*x6_2 + d_2*m*x3_3*x7_2 + 4*d_2*e3*m*x1_2 + 4*d_2*e3*m*x3_2 - 4*c33*d_2*e33*m*x1_2 - 4*c33*d_2*e33*m*x3_2 - 2*c3*d_2*m*x1_3*x6 + 4*c3*d_2*m*x1_2*xd3d + 4*c3*d_2*m*x3_2*xd3d - 2*d_2*m*x1_3*x5*x7 + d_2*m*x1_2*x3*x5_2 + d_2*m*x1_2*x3*x6_2 + d_2*m*x2_2*x3*x5_2 + d_2*m*x1_2*x3*x7_2 + d_2*m*x2_2*x3*x6_2 - d_2*m*x3*x4_2*x5_2 + d_2*m*x2_2*x3*x7_2 + d_2*m*x3*x4_2*x6_2 + d_2*m*x3*x4_2*x7_2 - 4*d_2*e2*m*x1*x4 + 4*d_2*e2*m*x2*x3 + 4*d_2*e4*m*x1*x2 + 4*d_2*e4*m*x3*x4 + 4*c22*d_2*e22*m*x1*x4 - 4*c22*d_2*e22*m*x2*x3 - 4*c44*d_2*e44*m*x1*x2 - 4*c44*d_2*e44*m*x3*x4 - 4*c2*d_2*m*x1*x4*xd2d + 4*c2*d_2*m*x2*x3*xd2d + 4*c4*d_2*m*x1*x2*xd4d + 4*c4*d_2*m*x3*x4*xd4d + 2*c2*d_2*m*x1_2*x4*x5 - 2*c2*d_2*m*x1*x4_2*x6 - 2*c3*d_2*m*x1*x3_2*x6 + 2*c3*d_2*m*x1_2*x2*x7 - 2*c3*d_2*m*x1_2*x4*x5 - 2*c4*d_2*m*x1*x2_2*x6 - 2*c2*d_2*m*x2*x3_2*x7 - 2*c4*d_2*m*x1_2*x2*x7 + 2*c3*d_2*m*x2*x3_2*x7 - 2*c3*d_2*m*x3_2*x4*x5 + 2*c4*d_2*m*x3_2*x4*x5 - 2*d_2*m*x1*x2*x4*x5_2 - 2*d_2*m*x1_2*x2*x5*x6 - 2*d_2*m*x1*x3_2*x5*x7 - 2*d_2*m*x1*x4_2*x5*x7 - 2*c2*d_2*m*x1*x2*x3*x5 + 2*c4*d_2*m*x1*x2*x3*x5 + 2*c2*d_2*m*x1*x3*x4*x7 + 2*c2*d_2*m*x2*x3*x4*x6 - 2*c4*d_2*m*x1*x3*x4*x7 - 2*c4*d_2*m*x2*x3*x4*x6 - 2*d_2*m*x1*x3*x4*x5*x6 + 2*d_2*m*x2*x3*x4*x5*x7)/(2*div);
        u2 = (d_2*m*x2_3*x5_2 - 2*dwx*x1*x2_2 - 2*dwx*x1*x3_2 - 2*dwx*x1*x4_2 - 2*dwx*x1_3 + d_2*m*x2_3*x6_2 + d_2*m*x2_3*x7_2 + 4*d_2*e2*m*x1_2 + 4*d_2*e2*m*x2_2 - 4*c22*d_2*e22*m*x1_2 - 4*c22*d_2*e22*m*x2_2 - 2*c2*d_2*m*x1_3*x5 + 4*c2*d_2*m*x1_2*xd2d + 4*c2*d_2*m*x2_2*xd2d + 2*d_2*m*x1_3*x6*x7 + d_2*m*x1_2*x2*x5_2 + d_2*m*x1_2*x2*x6_2 + d_2*m*x2*x3_2*x5_2 + d_2*m*x1_2*x2*x7_2 + d_2*m*x2*x3_2*x6_2 - d_2*m*x2*x4_2*x5_2 + d_2*m*x2*x3_2*x7_2 + d_2*m*x2*x4_2*x6_2 + d_2*m*x2*x4_2*x7_2 + 4*d_2*e3*m*x1*x4 + 4*d_2*e3*m*x2*x3 - 4*d_2*e4*m*x1*x3 + 4*d_2*e4*m*x2*x4 - 4*c33*d_2*e33*m*x1*x4 - 4*c33*d_2*e33*m*x2*x3 + 4*c44*d_2*e44*m*x1*x3 - 4*c44*d_2*e44*m*x2*x4 + 4*c3*d_2*m*x1*x4*xd3d + 4*c3*d_2*m*x2*x3*xd3d - 4*c4*d_2*m*x1*x3*xd4d + 4*c4*d_2*m*x2*x4*xd4d - 2*c2*d_2*m*x1*x2_2*x5 - 2*c2*d_2*m*x1_2*x3*x7 + 2*c2*d_2*m*x1_2*x4*x6 - 2*c3*d_2*m*x1*x4_2*x5 - 2*c4*d_2*m*x1*x3_2*x5 - 2*c2*d_2*m*x2_2*x3*x7 + 2*c2*d_2*m*x2_2*x4*x6 - 2*c3*d_2*m*x1_2*x4*x6 + 2*c3*d_2*m*x2_2*x3*x7 + 2*c4*d_2*m*x1_2*x3*x7 - 2*c4*d_2*m*x2_2*x4*x6 + 2*d_2*m*x1*x3*x4*x5_2 + 2*d_2*m*x1_2*x3*x5*x6 + 2*d_2*m*x1*x2_2*x6*x7 + 2*d_2*m*x1*x3_2*x6*x7 + 2*d_2*m*x1*x4_2*x6*x7 + 2*d_2*m*x2_2*x4*x5*x7 - 2*c3*d_2*m*x1*x2*x3*x6 + 2*c4*d_2*m*x1*x2*x3*x6 + 2*c3*d_2*m*x1*x2*x4*x7 - 2*c3*d_2*m*x2*x3*x4*x5 - 2*c4*d_2*m*x1*x2*x4*x7 + 2*c4*d_2*m*x2*x3*x4*x5 - 2*d_2*m*x1*x2*x3*x5*x7 - 2*d_2*m*x1*x2*x4*x5*x6)/(2*div);
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

    //Send Control commands 
    control->thrust = u1/CRAZYFLIE_MASS;
    control->torque[0] = u2;
    control->torque[1] = u3;
    control->torque[2] = u4;
  }
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
LOG_GROUP_STOP(state)

PARAM_GROUP_START(posCtlBks)

PARAM_ADD(PARAM_FLOAT, alpha, &alpha)
PARAM_ADD(PARAM_FLOAT, alphaw, &alphaw)
PARAM_ADD(PARAM_FLOAT, L1, &L1)
PARAM_ADD(PARAM_FLOAT, L2, &L2)

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

//PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)
//PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

//PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, errorLimit,  &errorLimit)
//PARAM_ADD(PARAM_FLOAT, motorInputFilter,  &motorInputFilter)
//PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
//PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlBks)
