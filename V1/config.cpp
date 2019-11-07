#include "include/config.h"
#include "vex.h"
using namespace vex;

//*****DEVICE_SETUPS
brain Brain;
controller control(controllerType::primary);
motor m_ChasFL(PORT20, gearSetting::ratio18_1, false);
motor m_ChasBL(PORT10, gearSetting::ratio18_1, false);
motor m_ChasFR(PORT16, gearSetting::ratio18_1, true);
motor m_ChasBR(PORT9, gearSetting::ratio18_1, true);
motor m_Strafe(PORT17, gearSetting::ratio36_1, true);
motor m_LiftL(PORT11, gearSetting::ratio36_1, false);
motor m_LiftR(PORT2, gearSetting::ratio36_1, true);
gyro Gyro(Brain.ThreeWirePort.A);
pot p_LiftR(Brain.ThreeWirePort.B);

motor claw(PORT12, gearSetting::ratio36_1, false);
pot p_Claw(Brain.ThreeWirePort.C);
//*****


bool pre_auton_status = false;
int lift_min = -5000000; //error value
int claw_min = -5000000;


//this function and boolean array gives programmers the ability to do rising_edge. (check the indexes in config.h)
bool re_status[TOTAL_RE-1] = { 0 };
bool re_press(bool button, bool &re_status) {
  if (button == re_status) {
    return false;
  }
  else {
    re_status = button;
    return button;
  }
}

void v_lift(int volts){
    m_LiftL.spin(directionType::fwd,volts,voltageUnits::mV);
    m_LiftR.spin(directionType::fwd,volts,voltageUnits::mV);
}

void v_claw(int volts){
    claw.spin(directionType::fwd,volts,voltageUnits::mV);
}
