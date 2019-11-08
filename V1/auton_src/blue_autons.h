#include "../include/auton_lib.h"

#ifndef BLUE_AUTONS_H
#define BLUE_AUTONS_H

//Andres was here
void top_blue_tower() {
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
  lift_pid_status = false;
  claw_pid_status = false;
  t1.interrupt();
  v_lift(0);
  fwd_1D(-1000,-3000,-4000);
}

void skills() {
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
  fwd_1D(650,800,1000);
  place(80);
  lift();
  turn_1D(-60,-50,-80);
  fwd_1D(1150,1500,1500);
  place(80);
  lift();
  fwd_1D(550,900,1000);
  turn_1D(-30,-50,-80);
  fwd_1D(850,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);//place stack

  fwd_1D(-230,-800,-1000);
  turn_1D(-85,-50,-80);
  fwd_1D(2450,2000,1500);
  sleepMs(200);
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 1050;
  sleepMs(1500);
  fwd_1D(600,800,2000);
  sleepMs(200);
  grab = false;//first tower
  lift_target = lift_min;
  fwd_1D(-200,-800,-800);
  turn_1D(-58,-80,-80);
  fwd_1D(-1400,-1500,-1500);
  fwd_1D(4000,2000,1000);
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
  place(80);
  lift();
  sleepMs(500);
  fwd_1D(510,1000,2000);
  place(80);
  lift();
  sleepMs(500);
  fwd_1D(-1000,1500,2000);
  turn_1D(85,90,100);
  fwd_1D(1300,2500,3000);
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
