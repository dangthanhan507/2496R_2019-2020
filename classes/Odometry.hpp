#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../custom/config.hpp"

#define C1 0
#define C2 0
#define T 0

#define AX 0
#define AY 0
#define ATHETA 0
#define AK 0

#define COV_1 0
#define COV_2 0
#define COV_3 0


using Position = std::vector<double>;

class Odometry {
  Position odo_world = {0,0,0};
  Position gyro_world = {0,0,0};
  Position ukf_world = {0,0,0};


  Position error = {0,0,0};
  Position predicted_error = {0,0,0};
  double prev_ek = 0;
  double ek = C2; //error in deg/s

  Odometry(double x, double y, double theta) {
    //initialise worlds
    odo_world = {x, y, theta};
    gyro_world = {x, y, theta};
  }

  void calculate_predicted(double enc_avg) {//assuming best fit goes into deg/s
    double d_theta;
    prev_ek = ek;
    ek = (T*ek + DT * MS_TO_S * (C1+C2))/(T+DT*MS_TO_S);

    d_theta = (ek - prev_ek) * DT * MS_TO_S;
    predicted_error[X] = enc_avg * cos((predicted_error[THETA]+ d_theta/2) * DEG_TO_RAD);
    predicted_error[Y] = enc_avg * sin((predicted_error[THETA]+ d_theta/2) * DEG_TO_RAD);
    predicted_error[THETA] += d_theta;
  }

  void calculate_gyro(double gyro_value, double enc_avg) {
    double d_theta = gyro_value * DT * MS_TO_S; // in degrees
    gyro_world[X] += enc_avg * cos((gyro_world[THETA] + d_theta/2) * DEG_TO_RAD);
    gyro_world[Y] += enc_avg * sin((gyro_world[THETA] + d_theta/2) * DEG_TO_RAD);
    gyro_world[THETA] += d_theta;
  }

  void calculate_odo(double enc_l, double enc_r) {
    double enc_avg = (enc_l + enc_r)/2;
    double d_theta = (enc_r - enc_l)/WHEELBASE; // in radians
    odo_world[X] = enc_avg * cos(odo_world[THETA] * DEG_TO_RAD + d_theta/2);
    odo_world[Y] = enc_avg * sin(odo_world[THETA] * DEG_TO_RAD + d_theta/2); //include or not depending on experiment
    odo_world[THETA] += d_theta * RAD_TO_DEG;
  }

  void calculate_ukf() {
    //must have done all calculate methods beforehand before calling this
    error[X] = gyro_world[X] - odo_world[X];
    error[Y] = gyro_world[Y] - odo_world[Y];
    error[THETA] = gyro_world[THETA] - odo_world[THETA];

    std::vector<double> lambda = {AX * AX * (1+AK), AY * AY * (1+AK), ATHETA * ATHETA * (1+AK)};
    
  }
};

#endif
