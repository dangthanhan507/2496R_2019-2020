#include "../../include/config.h"
#include "../../include/auton_lib.h"
competition Competition;

void pre_auton() {

}

void autonomous() {
  fwd_1D(2000,2000,2500);
}

void drivercontrol() {
  while(true) {
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
