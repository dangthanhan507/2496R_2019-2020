#ifndef PID_H
#define PID_H

#include <cmath>

class PID {
  public:
    double kp;
    double ki;
    double kd;
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;

    double kpp;

    PID(double P, double I, double D) {
      kp = P;
      ki = I;
      kd = D;
    }

    double Calculate(double target, double input, double limit, double max) {
      prev_error = error;
      error = target - input;

      std::abs((int)error) < limit? integral += error : integral = 0;

      integral >= 0? integral = fmin(integral, max): integral = fmax(integral, -max);

      derivative = error - prev_error;
      return kp*error + ki *integral + kd*derivative;
    }
};

#endif
