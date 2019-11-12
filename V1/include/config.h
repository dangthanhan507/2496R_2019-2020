#ifndef CONFIG_H
#define CONFIG_H

#include "vex.h"
#include "PID.h"
using namespace vex;

//Structures for autonomous
struct Point {
  double x;
  double y;
};
//**********************

//conversion constants
#define DT 5 //delta_time is 20ms
#define PI 3.1415926535
#define TICKS_TO_RAD_36_1 1800/2/PI
#define TICKS_TO_RAD_18_1 900/2/PI
#define WHEEL_RADIUS 4 //4in radius wheels
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI
//**********************


//autonomous constants
#define KVFF 3.392
#define KAFF 0.5

#define KV_INTERCEPT 1020

#define P_TRAP_POS 45
#define I_TRAP_POS 0
#define D_TRAP_POS 0
#define LIMIT_TRAP_POS 0

#define P_THETA_CORRECTION 40
#define I_THETA_CORRECTION 0
#define D_THETA_CORRECTION 0
#define LIMIT_THETA_CORRECT 0 //integral limit

//**********************

//robot constants

//measured 286.478898
#define LR_DISTANCE 535.478898
#define LR_TOLERANCE 0.01

//**********************


//*****DEVICE_SETUPS
extern brain Brain;
extern controller control;
extern motor m_ChasFL;
extern motor m_ChasBL;
extern motor m_ChasFR;
extern motor m_ChasBR;
extern motor m_Strafe;

extern motor m_LiftL;
extern motor m_LiftR;
extern pot p_LiftL;
extern pot p_LiftR;

extern motor claw;
extern pot p_Claw;
extern gyro Gyro;
//**********************

//drive constants
#define P_LIFT 47
#define I_LIFT 0
#define D_LIFT 2000
#define LIMIT_LIFT 50

#define CHASSIS_DEADZONE 3
#define STRAFE_DEADZONE 20


#define LIFT_MAX 1780
#define LIFT_MIN 3080
#define LIFT_MODIFIER -200
#define LIFT_SPEED 100

#define GRABFF 4500
#define CLAW_CLOSE 1133
#define CLAW_OPEN 1700
//**********************



//rising edge setup
#define TOTAL_RE 12
#define INDEX_UP 0
#define INDEX_RIGHT 1
#define INDEX_DOWN 2
#define INDEX_LEFT 3
#define INDEX_X 4
#define INDEX_A 5
#define INDEX_B 6
#define INDEX_Y 7
#define INDEX_L1 8
#define INDEX_L2 9
#define INDEX_R1 10
#define INDEX_R2 11
extern bool re_status[TOTAL_RE-1];
extern bool re_press(bool button, bool &re_status);
extern void v_lift(int volts);
extern void v_claw(int volts);
extern int lift_min;
extern int claw_min;
extern bool pre_auton_status;
//**********************

#endif
