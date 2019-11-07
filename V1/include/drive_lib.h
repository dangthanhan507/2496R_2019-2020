#ifndef DRIVE_LIB_H
#define DRIVE_LIB_H
#include "config.h"

extern double mpvert;
extern double mphor;

extern void drive_chassis();

extern bool grab_;
extern bool stack_;
extern bool claw_manual;
extern bool lift_manual;
extern bool closed;

extern void drive_claw();

extern void stacker();

extern double claw_value;
extern double lift_value;

#endif
