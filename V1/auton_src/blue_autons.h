#include "../include/auton_lib.h"

#ifndef BLUE_AUTONS_H
#define BLUE_AUTONS_H


void top_blue_tower() {
  device_setup();
  //deploy()
  stop_chas(brakeType::hold);
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  fwd_1D(190,800,2000);
  deploy();
  sleepMs(500);
  lift();
  fwd_1D(725,800,2000);
  lift_target = lift_min - 320;
  fwd_1D(340,500,2000);
  place(80);
  lift();
  turn_1D(-100,-120,-120);
  fwd_1D(1130,1500,2000);
  place(80);
  lift();
  fwd_1D(550,900,2000);
  turn_1D(-38,-50,-120);
  fwd_1D(900,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);
  lift_pid_status = false;
  claw_pid_status = false;
  t1.interrupt();
  v_lift(0);
  fwd_1D(-1000,-3000,-4000);
}

void skills() {
  /*device_setup();
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  lift_target = lift_min - 500;
  claw_pid_status = true;
  fwd_1D(1700,2000,2500);
  lift_target = lift_min - 260;
  turn_1D(-40,-50,-50);
  fwd_1D(1100,600,1000);
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 250;
  turn_1D(-172,-100,-50);
  fwd_1D(2420,2000,2500);
  turn_1D(40,100,50);
  fwd_1D(1300,2000,2500);
  lift_target = lift_min;
  sleepMs(500);
  grab = false;
  fwd_1D(-400,-1000,-2500);
  t1.interrupt();
  lift_pid_status = false;
  claw_pid_status = false;
  v_lift(0);*/
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
  fwd_1D(700,800,1000);
  place(80);
  lift();
  turn_1D(-52,-70,-80);
  fwd_1D(1200,1500,2000);
  place(80);
  lift();
  fwd_1D(550,900,1000);
  turn_1D(-30,-50,-100);
  fwd_1D(900,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);

  fwd_1D(-250,-800,-1000);
  turn_1D(-85,-100,-100);
  fwd_1D(2450,2000,1500);
  sleepMs(200);
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 1050;
  sleepMs(1500);
  fwd_1D(600,800,2000);
  sleepMs(200);
  grab = false;

  fwd_1D(-30,-600,-800);
  lift_target = lift_min;
  turn_1D(-57,-100,-120);
  fwd_1D(-1500,1800,1500);
  sleepMs(200);
  fwd_1D(3500,2300,1800);
  /*grab = true;
  sleepMs(200);
  lift_target = lift_min - 1050;
  fwd_1D(600,800,1000);
  grab = false;*/
}


//eh
void bot_blue() {
  device_setup();
  stop_chas(brakeType::hold);
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
  place(100);
  lift();
  sleepMs(500);
  fwd_1D(510,1000,2000);
  place(50);
  lift();
  sleepMs(500);
  fwd_1D(-1000,1500,2000);
  turn_1D(140,130,150);
  fwd_1D(1900,2500,3500);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);
  t1.interrupt();
  lift_pid_status = false;
  claw_pid_status = false;
  fwd_1D(-750,2000,2000);
  v_lift(0);
}


#endif
