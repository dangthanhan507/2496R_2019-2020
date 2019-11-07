#include "vex.h"
using namespace vex;
competition Competition;
brain Brain;
controller control(controllerType::primary);
gyro c_gyro(Brain.ThreeWirePort.A);

void pre_auton() {
  c_gyro.startCalibration();
  while(c_gyro.isCalibrating()){}
}

void autonomous() {

  while(true){
    Brain.Screen.printAt(100, 70, "gyro deg: %.2f", c_gyro.value(rotationUnits::deg));
  }
}


void drivercontrol() {
   while (c_gyro.isCalibrating()){}
    while(true) {
      Brain.Screen.printAt(100, 70, "Yo gyro deg: %.3f", (double)c_gyro.value(analogUnits::range8bit));
      sleepMs(50);
    }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
