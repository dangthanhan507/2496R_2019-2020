#include "../classes/PID.hpp"
#include "config.hpp"
#ifndef LIB_AUTON_H
#define LIB_AUTON_H

/*
extern void fwd(int power);
extern void back(int power);
extern void cstop();
extern void turnright(int power);
extern void turnleft(int power);
extern void outake();
extern void intake();
extern void stoptake();
*/

extern void pwr_intake(int pwr);
extern void volt_chas(double l_pwr, double r_pwr);
extern double feedforward(double value, double gain, double intercept);

extern void forward(double target, double cruise_v, double accel, bool reverse);

extern void turn(double target, double cruise_v, double accel, bool reverse);

extern void arc_turns(double target_angle, double radius, double cruise_v, double accel, bool reverse);

extern void pwr_intake(int pwr);

extern void blue_auton();

extern void lift(int pwr, int time);


#endif
