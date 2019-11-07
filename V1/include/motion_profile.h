#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H
#include <cmath>
#include "config.h"

#define MS_TO_S 0.001
class MotionProfile {
  public:
    //acceleration is in cm/s/s
    //velocity is in cm/s
    double acceleration;
    double target_velocity;

    MotionProfile (double v, double a) {
      target_velocity = v;
      acceleration = a;
    }


    void trapezoid_1D(double &current_v, double &accel, double &projected_pos, double target_pos) {//works with turns or linear movements
      //the output of this function is current_v, current_pos, and accel through pass by reference

      double triangle_distance = target_velocity * target_velocity / 2 / acceleration; //area under triangle
      if(projected_pos < triangle_distance) { //rising phase of trapezoid
        // pos = pos0 + v0*t + 1/2*a*t^2
        // pos0 = 0
        //a = constant
        projected_pos += current_v * DT/1000 + 0.5 * acceleration * DT/1000 * DT/1000;
        current_v += acceleration * DT/1000;
        //vf = vi + a*t
        accel = acceleration;
        if(current_v > target_velocity) current_v = target_velocity;
        return;
      }
      else if (projected_pos < target_pos - std::abs(triangle_distance)) { //cruising phase
        //pos = pos0 + v*t
        //v = constant
        current_v = target_velocity;
        projected_pos += target_velocity * DT * MS_TO_S;
        accel = 0;
        return;
      }
      else if (projected_pos < target_pos){ //falling phase
        projected_pos += current_v * DT/1000 - 0.5 * acceleration * DT/1000 * DT/1000; //same but this time acceleration is (-)
        current_v -= acceleration * DT/1000;
        if(current_v <= 0) {
          current_v = 0;
          projected_pos = target_pos;
          accel = 0;
          return;
        }
        accel = -acceleration;
        return;
      }
      current_v = 0;
      accel = 0;
      projected_pos = target_pos;
    }
};


#endif
