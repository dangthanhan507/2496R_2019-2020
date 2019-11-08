#include "../include/auton_lib.h"
#include "../include/config.h"
#include "../include/motion_profile.h"
#include "../include/PID.h"
#include <iostream>



//Odometry for 2D MP
double d_theta = 0; //delta_theta
double lr_avg = 0; //taking the average of left and right
thread t1;

double enc_l = 0, enc_r = 0, enc_b = 0;
double prev_enc_l, prev_enc_r, prev_enc_b;
double delta_l, delta_r, delta_b;

//absolute are variables representing the robot's position from when it first booted.
Point absolute; // returns robot's x,y position in cm's
double absolute_heading = 0; // represents robot's current angle in degrees (global)

void update_odometry() {
  prev_enc_l = enc_l; //update previous variables before updating current variables
  prev_enc_r = enc_r;
  prev_enc_b = enc_b;
  enc_l = (m_ChasBL.rotation(rotationUnits::raw) + m_ChasFL.rotation(rotationUnits::raw)) / 2; //avg of both one side motors
  enc_r = (m_ChasBR.rotation(rotationUnits::raw) + m_ChasFR.rotation(rotationUnits::raw)) / 2;
  enc_b = m_Strafe.rotation(rotationUnits::raw);

  delta_l = (enc_l - prev_enc_l); //delta = current - prev ; also delta is converted to cm
  delta_r = (enc_r - prev_enc_r);//encoders

  lr_avg = (delta_r + delta_l) / 2;
  d_theta = (delta_r - delta_l) / LR_DISTANCE; // this is calculated in documentation

  absolute.y += lr_avg * cos(absolute_heading * DEG_TO_RAD + d_theta / 2); //calculated in documentation
  absolute.x += -lr_avg * sin(absolute_heading * DEG_TO_RAD + d_theta / 2);

  /*if(std::abs(delta_l - delta_r) < LR_TOLERANCE) { //this makes sure that back wheel is only measured when left and right are moving the same speed
    absolute.y += delta_b * sin(absolute_heading * DEG_TO_RAD);
    absolute.x += delta_b * cos(absolute_heading * DEG_TO_RAD);
  }*/


  absolute_heading += RAD_TO_DEG * d_theta;

}
//************ 2D MP ************


void volt_chas (int volt_l, int volt_r) {
  m_ChasBL.spin(directionType::fwd, volt_l, voltageUnits::mV);
  m_ChasFL.spin(directionType::fwd, volt_l, voltageUnits::mV);
  m_ChasBR.spin(directionType::fwd, volt_r, voltageUnits::mV);
  m_ChasFR.spin(directionType::fwd, volt_r, voltageUnits::mV);
}

void stop_chas(brakeType mode) {
  m_ChasBL.stop(mode);
  m_ChasFL.stop(mode);
  m_ChasBR.stop(mode);
  m_ChasFR.stop(mode);
}

void reset_chas() {
  m_ChasBL.resetRotation();
  m_ChasFL.resetRotation();
  m_ChasBR.resetRotation();
  m_ChasFR.resetRotation();
  m_Strafe.resetRotation();
}


double feedforward(double velocity, double acceleration) {
  if(velocity >= 0) {
    return KVFF * velocity + KV_INTERCEPT + KAFF * acceleration;
  }
  else {
    return KVFF * velocity - KV_INTERCEPT + KAFF * acceleration;
  }
}


double fwd_1D(double target, double velo_max, double accelm) { //dead reackoning movement
  MotionProfile fwd(std::abs(velo_max), std::abs(accelm));
  PID go(P_TRAP_POS,I_TRAP_POS,D_TRAP_POS);
  PID straight(P_THETA_CORRECTION,I_THETA_CORRECTION,D_THETA_CORRECTION);
  double velo = 0;
  double accel = 0;
  double pos = 0;
  double projected_pos = 0;
  double current_l = 0, prev_l = 0;
  double current_r = 0, prev_r = 0;
  double pwm_l, pwm_r;
  double theta = 0;

  double sgn = target / std::abs(target);

  reset_chas();
  while(true) {
    prev_r = current_r;
    prev_l = current_l;

    current_l = (m_ChasBL.rotation(rotationUnits::raw) + m_ChasFL.rotation(rotationUnits::raw)) / 2;
    current_r = (m_ChasFR.rotation(rotationUnits::raw) + m_ChasBR.rotation(rotationUnits::raw)) / 2;

    delta_l = (current_l - prev_l);
    delta_r = (current_r - prev_r);
    theta += (delta_r - delta_l) / LR_DISTANCE;

    pos += (delta_r + delta_l)/2;

    fwd.trapezoid_1D(velo, accel, projected_pos, std::abs(target));

    pwm_l = feedforward(sgn * velo, sgn * accel) + go.Calculate(sgn * projected_pos, pos, LIMIT_TRAP_POS);
    pwm_r = feedforward(sgn * velo, sgn * accel) + go.Calculate(sgn * projected_pos, pos, LIMIT_TRAP_POS);
    volt_chas(pwm_l, pwm_r);

    printf("velo: %.2f      ",velo);
    printf("projected_pos: %.2f     ",projected_pos);
    printf("pos: %.2f\n",pos);
    printf("\n");

    if(velo == 0) break;
    sleepMs(DT);
  }
  printf("\n\n\n\n");
  stop_chas(brakeType::hold);
  return pos;
}

double turn_1D(double target, double theta_vmax, double theta_accelm) {

  double init_gyro = -Gyro.value(analogUnits::range8bit);
  MotionProfile turn(std::abs(theta_vmax), std::abs(theta_accelm));
  PID tur(400,0,0);



  double ang_velo = 0;
  double ang_accel = 0;
  double theta = 0;
  double projected_theta = 0;
  double current_l = 0, prev_l = 0;
  double current_r = 0, prev_r = 0;
  double pwm = 0;

  double sgn = target / std::abs(target);

  reset_chas();

  while(true) {
    prev_r = current_r;
    prev_l = current_l;

    current_l = (m_ChasBL.rotation(rotationUnits::raw) + m_ChasFL.rotation(rotationUnits::raw)) / 2;
    current_r = (m_ChasFR.rotation(rotationUnits::raw) + m_ChasBR.rotation(rotationUnits::raw)) / 2;

    delta_l = (current_l - prev_l);
    delta_r = (current_r - prev_r);

    //theta += (delta_r-delta_l) / (LR_DISTANCE) * RAD_TO_DEG;

    theta = -Gyro.value(analogUnits::range8bit) - init_gyro;

    turn.trapezoid_1D(ang_velo, ang_accel, projected_theta, std::abs(target));

    pwm = feedforward(sgn * ang_velo * DEG_TO_RAD / LR_DISTANCE, sgn * ang_accel * DEG_TO_RAD / LR_DISTANCE) + tur.Calculate(sgn * projected_theta, theta, 0);

    volt_chas(-pwm, pwm);

    control.Screen.print("gyro: %.2f", Gyro.value(analogUnits::range8bit));
    control.Screen.newLine();
    printf("velo: %.2f      ",ang_velo);
    printf("projected_theta: %.2f     ", projected_theta);
    printf("theta: %.2f\n", theta);

    if(ang_velo == 0) {
      printf("\n\n\n\n"); return theta;
    }
    sleepMs(DT);
  }
  stop_chas(brakeType::hold);
  printf("\n\n\n\n");
}


double lift_target = 0;
bool lift_pid_status = false;
double lift_value = 0;


bool claw_pid_status = false;
double claw_value = 0;
bool grab = false;

void device_setup() {
  lift_min = p_LiftR.value(analogUnits::range12bit);
  claw_min = p_Claw.value(analogUnits::range12bit);
}

void lift_claw_subsystem() {
  PID lift(P_LIFT, I_LIFT, D_LIFT);
  PID claw(30,0,0);
  control.Screen.clearScreen();
  while(true) {
    control.Screen.print(Gyro.value(analogUnits::range8bit));
    control.Screen.newLine();
    if(lift_pid_status) {
      lift_value = lift.calculate_lift(lift_target, p_LiftR.value(analogUnits::range12bit) ,LIMIT_LIFT);
      if(lift_value >= 3000 && lift_target == lift_min){
        lift_value = 3000;
      }
      v_lift(lift_value);
    }

    if(claw_pid_status) {
      if(grab) claw_value = GRABFF - claw.Calculate(claw_min-1500, p_Claw.value(analogUnits::range12bit), 0);
      else claw_value = -claw.Calculate(claw_min-200, p_Claw.value(analogUnits::range12bit), 0);
      v_claw(claw_value);
    }


    sleepMs(DT);
  }
}


void place() {
  lift_target = lift_min + 20;
  sleepMs(280);
  grab = false;
  sleepMs(650);
}
void place(int offset) {
  lift_target = lift_min + 20+offset;
  sleepMs(300);
  grab = false;
  sleepMs(650);
}
void lift() {
  grab = true;
  sleepMs(200);
  lift_target = lift_min - 405;
}
void deploy() {
  lift_target = lift_min - 200;
  claw_pid_status = true;
  grab = false;
  sleepMs(450);
  lift_target = lift_min;
  sleepMs(300);
}
