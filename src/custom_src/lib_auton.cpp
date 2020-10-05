#include "custom/lib_auton.hpp"
#include "custom/config.hpp"
#include "classes/Motion_Profile.hpp"
#define KVFF_FWD 7.3529
#define KAFF_FWD 2.5
#define KVFF_INT_FWD 1500.0

#define KP_FWD 50
#define KI_FWD 0.1
#define KD_FWD 0
#define LIMIT_FWD 100
#define MAX_FWD 500

#define KVFF_TURN 22.5339
#define KAFF_TURN 5.5
#define KVFF_INT_TURN 1459.96

#define KP_TURN 200
#define KI_TURN 0.8
#define KD_TURN 0
#define LIMIT_TURN 20
#define MAX_TURN 2000.0


void volt_chas(double l_pwr, double r_pwr) {
  mtr_chasBL.move_voltage(l_pwr);
  mtr_chasFL.move_voltage(l_pwr);
  mtr_chasBR.move_voltage(r_pwr);
  mtr_chasFR.move_voltage(r_pwr);
}

double feedforward(double value, double gain, double intercept) {
  return value * gain + intercept;
}

double a = 0, b = 0, d = 0, e = 0, f = 0, g = 0;
void print(void *param) {
  while(true) {
    printf("encoder_avg: %.2f   projected_pos: %.2f     projected_velo: %.2f      projected_accel: %.2f     voltage: %.2f     ang_velo: %.2f\n", f, b, d, e, a, g);
    delay(50);
  }
}

void forward(double target, double cruise_v, double accel, bool reverse) {
  PID chassis_pid(KP_FWD, KI_FWD, KD_FWD);
  PID turn_pid(KP_TURN, 0, KD_TURN);
  Motion_Profile profile(std::abs(cruise_v),std::abs(accel));
  double projected_pos = 0, projected_velo = 0, projected_accel = 0;

  double voltage = 0;
  double velocity = 0;
  double prev_encoder_avg = 0;
  double encoder_avg = 0;

  double imu_offset = imu.get_yaw();
  double angle = 0;
  double angle_volt;

  enc_l.reset();
  enc_r.reset();

  while(true) {
    prev_encoder_avg = encoder_avg;
    profile.trap_1d(target, projected_pos, projected_velo, projected_accel);

    encoder_avg = (enc_l.get_value() + enc_r.get_value())/2;

    angle = imu.get_yaw() - imu_offset;


    if(!reverse) {
      voltage = chassis_pid.Calculate(projected_pos, std::abs(encoder_avg), LIMIT_FWD, MAX_FWD);
      voltage += feedforward(projected_velo, KVFF_FWD, KVFF_INT_FWD);
      voltage += feedforward(projected_accel, KAFF_FWD, 0);
    }
    else {
      voltage = -chassis_pid.Calculate(projected_pos, std::abs(encoder_avg), LIMIT_FWD, MAX_FWD);
      voltage -= feedforward(projected_velo, KVFF_FWD, KVFF_INT_FWD);
      voltage -= feedforward(projected_accel, KAFF_FWD, 0);
    }

    angle_volt = turn_pid.Calculate(imu_offset,angle,LIMIT_TURN, MAX_TURN);
    volt_chas(voltage + angle_volt, voltage - angle_volt);

    if(std::abs(chassis_pid.error) <= 10 && projected_pos == target) break;

    delay(15);
  }
  volt_chas(0,0);
}

void turn(double target, double cruise_v, double accel, bool reverse) {
  PID turn_pid(KP_TURN, KI_TURN, KD_TURN);
  Motion_Profile profile(std::abs(cruise_v),std::abs(accel));
  double projected_theta = 0, projected_velo = 0, projected_accel = 0;

  Task task1 (print, (void*) "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "CONTROL_TASK" );

  double voltage = 0;

  double angle_offset = imu.get_yaw();
  double angle = 0;
  double prev_angle = 0;
  while(true) {
    profile.trap_1d(target, projected_theta, projected_velo, projected_accel);
    prev_angle = angle;
    angle = imu.get_yaw() - angle_offset;

    if(!reverse) {
      voltage = turn_pid.Calculate(projected_theta, std::abs(angle), LIMIT_TURN, MAX_TURN);
      voltage += feedforward(projected_velo, KVFF_TURN, KVFF_INT_TURN);
      voltage += feedforward(projected_accel, KAFF_TURN, 0);
    }
    else {
      voltage = -turn_pid.Calculate(projected_theta, std::abs(angle), LIMIT_FWD, MAX_FWD);
      voltage -= feedforward(projected_velo, KVFF_TURN, KVFF_INT_TURN);
      voltage -= feedforward(projected_accel, KAFF_TURN, 0);
    }

//    if(std::abs(turn_pid.error) <= 1) break;
    volt_chas(voltage, -voltage);

    delay(15);
  }
}

void arc_turns(double target_angle, double radius, double cruise_v, double accel, bool reverse) {
  //basically uses purepursuit to get arc_length
  //mb use velocity pid
  double curvature = 1/radius;
  double target_dist = target_angle * DEG_TO_RAD * radius; // arc = theta * r
  double projected_pos_l = 0, projected_pos_r = 0;
  double projected_velocity = 0;
  double projected_accel = 0;


  while(true) {

    delay(15);
  }
}



void pwr_intake(int pwr) {
  mtr_rollR.move(pwr);
  mtr_rollL.move(pwr);
}

void lift(int pwr, int time){
  mtr_lift.move(pwr);
  delay(time);
  mtr_lift.move(0);

}

void blue_auton(){
  pwr_intake(127);
  lift(90, 300);
  forward(300, 600, 500, false);
  forward(600, 300, 300, false);
  pwr_intake(0);
  forward(600, 400,350, true);


}
