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
  fwd_1D(800,800,1000);
  place(80);
  lift();
  turn_1D(-64,-50,-60);
  fwd_1D(1150,1500,1500);
  place(80);
  lift();
  fwd_1D(550,900,1000);
  turn_1D(-31,-50,-60);
  fwd_1D(1050,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);//place stack

  fwd_1D(-140,-800,-1000);
  lift_pid_status = false;
  claw_pid_status = false;
  t1.interrupt();
  v_lift(0);
}

void skills() {
  /*device_setup();
  //deploy()
  stop_chas(brakeType::hold);
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  fwd_1D(190,800,1000);
  deploy();
  sleepMs(500);
  lift();
  fwd_1D(800,800,1000);
  place(80);
  lift();
  turn_1D(-62,-50,-60);
  fwd_1D(1150,1500,1500);
  place(80);
  lift();
  fwd_1D(550,900,1000);
  turn_1D(-31,-50,-60);
  fwd_1D(950,1300,2000);
  lift_target = lift_min;
  sleepMs(1000);
  grab = false;
  sleepMs(200);//place stack

  fwd_1D(-140,-800,-1000);
  turn_1D(-85,-50,-60);
  fwd_1D(2450,2000,1500);
  sleepMs(200);
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 1250;
  sleepMs(1500);
  fwd_1D(600,800,2000);
  sleepMs(200);
  grab = false;//first tower

  fwd_1D(-1100,-1000,-1000);
  lift_target = lift_min;
  
  Gyro.startCalibration();
  sleepMs(2000);
  turn_1D(-62,-60,-60);
  fwd_1D(-1200,-1000,-1000);//wall lineup

  sleepMs(1000);
  fwd_1D(1820,1500,800);
  sleepMs(500);
  lift();
  fwd_1D(-400,-800,-800);
  turn_1D(-85,-50,-60);
  fwd_1D(1000,1000,1500);
  lift_target = lift_min - 800;
  sleepMs(1500);
  fwd_1D(1500,1000,1500);
  sleepMs(500);
  lift_target =lift_min - 500;
  sleepMs(500);
  grab = false;
  sleepMs(500);
  fwd_1D(-2000,-2000,-1500);
  */


  //lift();

  device_setup();
  stop_chas(brakeType::hold);
  thread t1(lift_claw_subsystem);
  lift_pid_status = true;
  lift_target = lift_min;
  fwd_1D(1010,1500,1500);
  deploy();
  sleepMs(500);
  fwd_1D(-25,-400,-600);
  turn_1D(62,50,60);
  sleepMs(200);
  fwd_1D(440,800,1000);
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 1250;
  sleepMs(1500);
  fwd_1D(600,800,2000);
  sleepMs(200);
  grab = false;//first tower
  fwd_1D(-60,-500,-800);
  lift_target = lift_min;
  turn_1D(-60,-50,-60);
  fwd_1D(600,800,1000);
  turn_1D(-30,-50,-60);
  fwd_1D(1000,600,800);
  lift();
}


//eh
void bot_blue() {
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
  turn_1D(90,50,80);
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
