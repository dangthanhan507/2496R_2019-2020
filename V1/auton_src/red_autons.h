#include "../include/auton_lib.h"

#ifndef RED_AUTONS_H
#define RED_AUTONS_H

void top_red_tower() {
  device_setup();
  //deploy()
  stop_chas(brakeType::hold);
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  fwd_1D(190,800,1000);
  deploy();
  sleepMs(500);
  lift();
  fwd_1D(750,800,1000);
  place(80);
  lift();
  turn_1D(59,50,60);
  fwd_1D(1150,1500,1500);
  place(80);
  lift();
  fwd_1D(550,900,1000);
  turn_1D(32,50,60);
  fwd_1D(1150,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);//place stack

  fwd_1D(-140,-800,-1000);
  lift_pid_status = false;
  claw_pid_status = false;
  t1.interrupt();
  v_lift(0);}

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
  turn_1D(-84,-50,-80);
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
