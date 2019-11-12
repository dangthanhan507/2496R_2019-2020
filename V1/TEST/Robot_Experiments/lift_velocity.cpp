#include "../../include/config.h"
#include "../../include/auton_lib.h"
#include <cmath>

#include <iostream>
#include <fstream>
competition Competition;

using namespace std;

void pre_auton() {
  lift_min = p_LiftR.value(analogUnits::range12bit);
  sleepMs(1000);
}

void autonomous() {

}

int delta_lift = 0;
int lift_current = 0;
int lift_prev = 0;
double velocity = 0;

void move(int volts){
    m_LiftL.spin(directionType::fwd,volts,voltageUnits::mV);
    m_LiftR.spin(directionType::fwd,volts,voltageUnits::mV);
}

void drivercontrol() {
  pre_auton();
  control.Screen.clearScreen();
  while(true) {

    lift_prev = lift_current;
    lift_current = p_LiftR.value(analogUnits::range12bit);
    delta_lift = lift_current - lift_prev;

    velocity = delta_lift*1000/20;

    move(-500);
    printf("Velocity: %.2f\n", velocity);
    control.Screen.print(velocity);
    sleepMs(20);
    if(re_press(control.ButtonX.pressing(),re_status[INDEX_X])) break;
    if(p_LiftR.value(analogUnits::range12bit) < lift_min - 1100) break;
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
