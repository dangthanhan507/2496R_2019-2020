#ifndef PIV_H
#define PIV_H

#include <cmath>
#include <vector>
class PIV {
  public:
    double kp;
    double ki;
    double kv;
    double error = 0;
    double prev_error = 0;
    double integral = 0;

    double error_velo = 0;

    std::vector<double> velocity;
    double size_n;

    PIV(double P, double I, double V, double size) {
      kp = P;
      ki = I;
      kv = V;
      size_n = size;
    }

    void add_velo(double velo) {
      velocity.push_back(velo);
      if(velocity.size() > size_n) velocity.erase(velocity.begin());
    }

    double average() {
      double sum = 0;
      for(int i = 0; i < velocity.size(); i++) {sum += velocity[i];}
      return sum/(size_n);
    }

    double Calculate(double target, double target_velo, double input, double limit, double max) {
      prev_error = error;
      error = target - input;

      std::abs((int)error) < limit? integral += error : integral = 0;

      integral >= 0? integral = fmin(integral, max): integral = fmax(integral, -max);

      error_velo = target_velo - average();

      return kp*error + ki *integral + kv*target_velo;
    }
};

#endif
