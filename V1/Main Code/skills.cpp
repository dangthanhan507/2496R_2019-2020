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
  skills();
}


/*
***************DRIVER CONTROL CODE***************8
*/


double mpvert;
double mphor;
double exponential_drive(int input, int c) {
  if(input >= 0) {
    return  c*std::pow(10.0, abs(input) * log10(100.0/c + 1.0) /127.0 ) - c;
  }
  else {
    return -(c*std::pow(10.0, abs(input) * log10(100.0/c + 1.0) /127.0 ) - c);
  }
}

void drive_chassis() {

  if(std::abs(control.Axis2.value())>std::abs(CHASSIS_DEADZONE)){
    mpvert =  exponential_drive(control.Axis2.value(), 60);
  }
  else{
    mpvert = 0;
  }

  if(std::abs(control.Axis4.value())>std::abs(CHASSIS_DEADZONE)){
    mphor = 0.5 * exponential_drive(control.Axis4.value(), 5);
  }
  else{
    mphor = 0;
  }

  m_ChasBL.spin(directionType::fwd, mpvert+mphor, velocityUnits::pct);
  m_ChasFL.spin(directionType::fwd, mpvert+mphor, velocityUnits::pct);
  m_ChasBR.spin(directionType::fwd, mpvert-mphor, velocityUnits::pct);
  m_ChasFR.spin(directionType::fwd, mpvert-mphor, velocityUnits::pct);

  if(std::abs(control.Axis1.value()) > STRAFE_DEADZONE){
      m_Strafe.spin(directionType::fwd, control.Axis1.value() , velocityUnits::pct);
  }
  else{
    m_Strafe.spin(directionType::fwd, 0, velocityUnits::pct);
  }
}



bool grab_ = false;
bool stack_ = false;
bool claw_manual = false;
bool lift_manual = false;
bool closed = false;
void drive_lift() {
  if(control.ButtonL1.pressing()){
    m_LiftL.spin(directionType::fwd,LIFT_SPEED,velocityUnits::pct);
    m_LiftR.spin(directionType::fwd,LIFT_SPEED,velocityUnits::pct);
  }
  else if(control.ButtonR1.pressing()){
    m_LiftL.spin(directionType::rev,LIFT_SPEED,velocityUnits::pct);
    m_LiftR.spin(directionType::rev,LIFT_SPEED,velocityUnits::pct);
  }
  else{
    m_LiftL.stop(brakeType::hold);
    m_LiftR.stop(brakeType::hold);
  }
}

void drive_claw() {
  if (control.ButtonL2.pressing()){
    claw.spin(directionType::rev,100,velocityUnits::pct);
    closed = false;
  }
  else if (control.ButtonR2.pressing()){
    claw.spin(directionType::fwd,100,velocityUnits::pct);
    closed = true;
  }
  else if (closed){
    claw.spin(directionType::fwd,2500,voltageUnits::mV);
  }
  else{
    claw.stop(brakeType::hold);
  }
}

void stacker() {
  PID claw_(30,0,0);
  PID lift_(P_LIFT,I_LIFT,D_LIFT);
  double claw_value, lift_value;

  double stack_target = lift_min;
  control.Screen.clearScreen();
  while(true) {
    //buttons to toggle manual or pid
    if(re_press(control.ButtonX.pressing(), re_status[INDEX_X])) lift_manual = !lift_manual;
    if(re_press(control.ButtonB.pressing(), re_status[INDEX_B])) claw_manual = !claw_manual;


    //buttons to modify lift height pid
    if(re_press(control.ButtonA.pressing(), re_status[INDEX_A])) {
      lift_min = p_LiftR.value(analogUnits::range12bit);
    }

    if(re_press(control.ButtonL1.pressing(), re_status[INDEX_L1]))  {
      if(stack_target!=LIFT_MIN+100){
        stack_target -= LIFT_MODIFIER;
      }
      else{
        stack_target+=100; //lift mod for lifting slightly off floor
      }
    }
    else if(re_press(control.ButtonR1.pressing(), re_status[INDEX_R1]))  /*&& lift_target >= LIFT_MIN*/{
          stack_target+=LIFT_MODIFIER;
    }
    else if(re_press(control.ButtonLeft.pressing(), re_status[INDEX_LEFT])){
        stack_target+=35;
    }
    else if(re_press(control.ButtonRight.pressing(), re_status[INDEX_RIGHT])){
      stack_target-=35;
    }

    //buttons to automate stacker
    if(control.ButtonR2.pressing()) grab_ = true;
    else if (control.ButtonL2.pressing()) grab_ = false;

    if(control.ButtonUp.pressing()) {
      stack_ = true;
      stack_target = lift_min-300;
      grab_ = true;
      lift_manual = false;
      claw_manual = false;
    }
    else if (control.ButtonDown.pressing()) {
      stack_target = lift_min;
      stack_ = false;
      lift_manual = false;
      claw_manual = false;
      lift_value = lift_.calculate_lift(stack_target, p_LiftR.value(analogUnits::range12bit), LIMIT_LIFT);
      if(lift_value <= 1000) lift_value = 1000;
      v_lift(lift_value);
      sleepMs(200);
      grab_ = false;
    }
    claw_.Calculate(claw_min - 1200, p_Claw.value(analogUnits::range12bit), 0);
    lift_.calculate_lift(stack_target, p_LiftR.value(analogUnits::range12bit), LIMIT_LIFT);


    //code to run the claw and lift (whether pid or manual)
    if(claw_manual) {
      drive_claw();
    }
    else {
      if(grab_) claw_value = GRABFF - claw_.Calculate(claw_min - 1250, p_Claw.value(analogUnits::range12bit), 0);
      else claw_value = -claw_.Calculate(claw_min-50,p_Claw.value(analogUnits::range12bit), 0);
      v_claw(claw_value);
    }

    if(lift_manual) {
      drive_lift();
    }
    else if (std::abs(claw_.error) < 300 || claw_manual == true) {
      lift_value = lift_.calculate_lift(stack_target, p_LiftR.value(analogUnits::range12bit), LIMIT_LIFT);
      if(lift_value >= 1000 && lift_target == lift_min){
        lift_value = 1000;
      }
      v_lift(lift_value);
    }

    control.Screen.setCursor(1,1);
    control.Screen.print(lift_.error);
    control.Screen.newLine();

    sleepMs(DT);
  }
}

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
