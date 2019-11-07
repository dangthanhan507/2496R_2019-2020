#ifndef PID_H
#define PID_H

class PID {
  public:
    double kp;
    public:double ki;
    double kd;
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;
    double value = 0;
    PID(double P, double I, double D) {
      kp = P;
      ki = I;
      kd = D;
    }

    double Calculate(double target, double input, int limit) {
      prev_error = error;
      error = target - input;
      abs((int)error) < limit? integral += error : integral = 0;
      //if(error==0) integral = 0;
      derivative = error - prev_error;
      return kp*error + ki *integral + kd*derivative;
    }

    double calculate_lift(double target, double input, int limit) {
      //integral is holding power of robot

      prev_error = error;
      error = target - input;
      //if(error==0) integral = 0;
      derivative = error - prev_error;

      if(abs((int)error) < limit) {
        if (error > 0) integral += error/3; //compensate for acceleration due to gravity
        else integral += error;
      }
      else integral = 0;

      //capping error
      if(error >= 5500/kp) error = 5500/kp;
      else if (error <= -5500/kp) error = -5500/kp;


      //capping integral
      if(integral >= 6000/ki) integral = 6000/ki;
      else if (integral <= -6000/ki) integral = -6000/ki;



      value = kp*error + ki*integral + kd*derivative;
      if(error > 0) value = kp*error/3 + ki*integral + kd*derivative;
      if(value >= 3000) {
        value = 3000;
      }
      else if (value <= -7000) {
        value = -7000;
      }


      return value;
    }
};

#endif
