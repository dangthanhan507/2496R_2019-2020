#include "../include/auton_lib.h"

#ifndef RED_AUTONS_H
#define RED_AUTONS_H

void top_red_tower() {
  device_setup();
  stop_chas(brakeType::hold);
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  fwd_1D(75,800,2000);
  deploy();
  lift();
  fwd_1D(725,800,2000);
  lift_target = lift_min - 330;
  fwd_1D(410,500,2000);
  place(50);
  lift();
  turn_1D(95,100,150);
  fwd_1D(1130,1500,2000);
  place(50);
  lift();
  fwd_1D(560,900,2000);
  turn_1D(20,100,120);
  fwd_1D(1000,1400,2200);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  fwd_1D(-500,-900,-2000);
  t1.interrupt();
  lift_pid_status = false;
  claw_pid_status = false;
  v_lift(0);
}

void bot_red() {//tuned
  stop_chas(brakeType::hold);
  device_setup();
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  deploy();
  fwd_1D(1000,1500,2500);
  grab = true;
  sleepMs(200);
  lift();
  sleepMs(500);
  fwd_1D(510,1000,2000);
  place(70);
  lift();
  sleepMs(500);
  fwd_1D(510,1000,2000);
  place(50);
  lift();
  sleepMs(500);
  fwd_1D(-1100,1500,2000);
  turn_1D(-144,-100,-150);
  fwd_1D(1600,1800,3000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  fwd_1D(-1000,-1000,-2500);
  t1.interrupt();
  lift_pid_status = false;
  claw_pid_status = false;
  v_lift(0);
}

#endif
