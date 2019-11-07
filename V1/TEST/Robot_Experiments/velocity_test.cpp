#include "include/config.h"
#include "include/auton_lib.h"
#include "auton_src/red_autons.cpp"
#include "auton_src/blue_autons.cpp"
#include <cmath>

#include <iostream>
#include <fstream>
competition Competition;

using namespace std;

void pre_auton() {

}

void autonomous() {

}

void reset() {
    m_ChasBL.resetRotation();
    m_ChasFL.resetRotation();
    m_ChasFR.resetRotation();
    m_ChasBR.resetRotation();
}

void move(int mv) {
    m_ChasBL.spin(directionType::fwd, mv , voltageUnits::mV);
    m_ChasFL.spin(directionType::fwd,  mv , voltageUnits::mV);
    m_ChasBR.spin(directionType::fwd,  mv , voltageUnits::mV);
    m_ChasFR.spin(directionType::fwd, mv , voltageUnits::mV);
}

void drivercontrol() {

    double now_tick = 0;
    double prev_tick = 0;
    double delta_tick = 0;
    double velocity = 0; //velocity is in in/ms
    control.Screen.clearScreen();
    reset();
    while(true) {
        move(12000);
        now_tick = std::abs(m_ChasBL.rotation(rotationUnits::raw)) + std::abs(m_ChasBR.rotation(rotationUnits::raw));
        now_tick += std::abs(m_ChasFL.rotation(rotationUnits::raw)) + std::abs(m_ChasFR.rotation(rotationUnits::raw));
        now_tick = now_tick / 4;
        delta_tick = now_tick - prev_tick;
        velocity = delta_tick * 4 / 900 / 20 * 1000;
        control.Screen.print(velocity);
        sleepMs(20);
        prev_tick = now_tick;
        if(re_press(control.ButtonX.pressing(),re_status[INDEX_X])) break;
    }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
