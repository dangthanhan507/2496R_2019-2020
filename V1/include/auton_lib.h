#ifndef AUTON_LIB_H
#define AUTON_LIB_H
#include "config.h"

extern double absolute_heading; //heading from when robot booted.
extern Point absolute; //x and y coordinates of the robot from whence it booted

extern void update_odometry(); //haven't tested yet

extern void volt_chas (int volt_l, int volt_r);
extern void stop_chas(brakeType mode);

extern void reset_chas();
extern double fwd_1D(double target, double velo_max, double accelm);
extern double turn_1D(double target, double theta_vmax, double theta_accelm);


extern double lift_target;
extern bool lift_pid_status;
extern double lift_value;


extern bool claw_pid_status;
extern double claw_value;
extern bool grab;

extern double minimum_lift;
extern double minimum_claw;
extern void device_setup();

extern void lift_claw_subsystem();


extern void place();
extern void place(int offset);
extern void lift();
extern void deploy();
#endif
