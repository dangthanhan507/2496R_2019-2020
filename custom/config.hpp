#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include "../classes/PID.hpp"
#include "../classes/PIV.hpp"
using namespace pros;

extern Controller control;
extern Motor mtr_chasBL;
extern Motor mtr_chasFL;
extern Motor mtr_chasBR;
extern Motor mtr_chasFR;
extern Motor mtr_lift;
extern Motor mtr_tilt;
extern Motor mtr_rollL;
extern Motor mtr_rollR;

extern ADIEncoder enc_l;
extern ADIEncoder enc_r;

extern ADIAnalogIn pot;
extern ADIDigitalIn limit;
extern ADIDigitalIn bump;
extern ADIDigitalIn limit_index;
/*
  CONSTANTS
*/
#define X 0
#define Y 1
#define THETA 2
#define V 2
#define A 3

#define L 0
#define R 1

#define PI 3.141592653
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

#define WHEELBASE 400.0123
//208.348289

#define GRAVFF 0
#define KP_LIFT 50
#define KI_LIFT 0.2
#define KD_LIFT 200
#define LIMIT_LIFT 100
#define LIFT_MAX 4000

#define LIFTH_BOTTOM 0
#define LIFTH_SMALLTOWER -885
#define LIFTH_BIGTOWER -1185

#define DT 15
#define MS_TO_S 1/1000

extern int pot_offset;

extern Imu imu;

extern double preset_heights[3];
extern PID autolift;
extern int index_heights;
extern double lift_value;
extern void pid_lift();

#endif
