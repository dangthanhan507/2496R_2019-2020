#ifndef SPLINE_H
#define SPLINE_H
#include "../custom/config.hpp"


class Spline{
  private:
    std::vector<double> start = {0,0,0};//x,y,theta
    std::vector<double> end = {0,0,0};
  public:
    double ax,bx,cx,dx;
    double ay,by,cy,dy;
    std::vector<std::vector<double>> path;

    Spline(std::vector<double> start_, std::vector<double> end_, double t_interval) {
      start = start_;
      end = end_;
      calculate_constants();
      path = point_generation(t_interval);
    }

    void calculate_constants() {
      double slope_xi, slope_xf;//i for inital, f for final
      double slope_yi, slope_yf;

      //magnitude of vector from start vector to end vector
      double mag_vector = sqrt( pow(end[X]-start[X],2) + pow(end[Y]-start[Y],2) );

      //calculating "slopes" characteristics
      slope_xi = cos(start[THETA]*DEG_TO_RAD) * mag_vector;
      slope_xi = sin(start[THETA]*DEG_TO_RAD) * mag_vector;

      slope_xf = cos(end[THETA]*DEG_TO_RAD) * mag_vector;
      slope_yf = sin(end[THETA]*DEG_TO_RAD) * mag_vector;

      //calculating CONSTANTS (math in documentation)
      ax = 2 * start[X] - 2 * end[X] + slope_xf + slope_xi;
      bx = -2 * slope_xi - slope_xf - 3 * start[X] + 3 * end[X];
      cx = slope_xi;
      dx = start[X];

      ay = 2 * start[Y] - 2 * end[Y] + slope_yf + slope_yi;
      by = -2 * slope_yi - slope_yf - 3 * start[Y] + 3 * end[Y];
      cy = slope_yi;
      dy = end[Y];
    }

    std::vector<double> position(double t) {
      std::vector<double> pos = {0,0}; //x, y, angle, velocity (will be used for trajectory generation)
      pos[X] = ax*pow(t,3) + bx*pow(t,2) + cx*t + dx;
      pos[Y] = ay*pow(t,3) + by*pow(t,2) + cy*t + dy;
      return pos;
    }
    std::vector<double> velocity(double t) {
      std::vector<double> vel = {0,0};
      vel[X] = 3*ax*pow(t,2) + 2*bx*t + cx;
      vel[Y] = 3*ay*pow(t,2) + 2*by*t + cy;
      return vel;
    }

    double speed(double t) {
      std::vector<double> velo = velocity(t);
      return hypot(velo[X], velo[Y]);
    }

    double parametric_length(double start, double end) {
      //calculates parametric length of spline from start to end
      // start -> [0,1) : end -> (0,1]
      //essentially does integral(start->end, x(t) & y(t))
      //uses gaussian quadrature approximation to calculate accurate to third degree

      double w; //weight
      double t0, t1; //nodes after transformed integral(-1->1, x(t) & y(t))
      t0 = 0.5 * (end - start) * (-1/sqrt(3)) + 0.5 * (end+start);
      t1 = 0.5 * (end - start) * (1 / sqrt(3)) + 0.5 * (end + start);
      return w * (speed(t0) + speed(t1));
    }

    std::vector<std::vector<double>> point_generation(double t_interval) {
      //make sure 1 and t_interval divides nicely into an integer pwease
      std::vector<std::vector<double>> list;
      std::vector<double> temp = {0,0,0,0};
      for(int t = 0; t <= 1; t+=t_interval) {
        temp[X] = position(t)[X];
        temp[Y] = position(t)[Y];
        list.push_back(temp);
      }
      return list;
    }
};

#endif
