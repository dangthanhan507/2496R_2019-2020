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

int prev_gyro = 0, gyro_r = 0;
int modifier = 0;
void drivercontrol() {
   while (c_gyro.isCalibrating()){}
   sleepMs(3000);
    while(true) {
      prev_gyro = gyro_r;
      gyro_r = c_gyro.value(analogUnits::range8bit) + modifier;

      if((int)c_gyro.value(analogUnits::range8bit) <= 5 && prev_gyro > 200) {
        modifier += 225;
      }
      else if (gyro_r >= -5 && prev_gyro < -200) {
        modifier += -225;
      }
      Brain.Screen.printAt(100, 70, "Yo gyro deg: %d", gyro_r);
      Brain.Screen.newLine();
      sleepMs(5);
    }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
