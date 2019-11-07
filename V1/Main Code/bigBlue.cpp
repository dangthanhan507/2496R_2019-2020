#include "../include/config.h"
#include "../auton_src/drive_lib.cpp"
#include "../auton_src/red_autons.h"
#include "../auton_src/blue_autons.h"
#include <cmath>
competition Competition;

int time = 0;
void pre_auton() {
  if(pre_auton_status == false) {
    if(lift_min == -5000000) {
      stop_chas(brakeType::coast);
      sleepMs(200);
      lift_min = p_LiftR.value(analogUnits::range12bit);
      claw_min = p_Claw.value(analogUnits::range12bit);
    }
    Gyro.startCalibration();
    while(Gyro.isCalibrating()) {
      time++;
      sleepMs(1);
    }
    pre_auton_status = true;
  }
}

void autonomous() {
  Brain.Screen.setCursor(1,3);
  Brain.Screen.print("time: %d", time);
  top_blue_tower();
}


/*
***************DRIVER CONTROL CODE***************8
*/



void drivercontrol() {
  pre_auton();
  lift_pid_status = false;
  claw_pid_status = false;
  stop_chas(brakeType::coast);
  thread stacker_t(stacker);

  while(true) {
    drive_chassis();
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
