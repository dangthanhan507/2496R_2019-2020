#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H
#include "../custom/config.hpp"
//constants to refer to vector components


//vector robot has 3 components (x,y,angle)
//vector anything else has 2 components(x,y)

class PurePursuit {
  double curvature = 1;
  double lk_radius = 0;
  std::vector<double> lk = {0,0}; //x,y

  PurePursuit(double radius) {
    lk_radius = radius; //gives lookahead radius
  }

  bool update_lk(std::vector<double> start, std::vector<double> end, std::vector<double> robot) {
    //updates variable lk, or the coordinates of lookahead intersection

    std::vector<double> d = {0,0}; //directional vector from start point to end point
    std::vector<double> f = {0,0}; //directional vector from robot point to end point

    d[X] = end[X] - start[X]; // giving vector d: x and y vector components
    d[Y] = end[Y] - start[Y];
    f[X] = start[X] - robot[X]; //giving vector f: x and y vector components
    f[Y] = start[Y] - robot[Y];

    double a,b,c,discriminant; //a,b,c stand for ax^2 + bc + c = 0 standard quadratic equation
    a = d[X] * d[X] + d[Y] * d[Y]; // a = dotproduct(d,d)
    b = 2 * (f[X]*d[X] + f[Y] * d[Y]); //b = 2 * dotproduct(f,d)

    c = (f[X] * f[X] + f[Y] * f[Y]) - lk_radius * lk_radius; // c = dotproduct(f,f) - lookahead_radius^2

    discriminant = b*b - 4 * a * c; //discriminant -> b^2 - 4*a*c (quadratic formula)

    double t1,t2; //two solutions of parametric intersection
    std::vector<double> temp = {0,0}; //actual intersection point that will be stored in lk

    temp[X] = start[X]; //init temp
    temp[Y] = start[Y];

    if (discriminant >= 0) { //if there is a solution (hence discriminant >=0)
      discriminant = sqrt(discriminant);
      t1 = (-b-discriminant)/2/a; //quadratic formula
      t2 = (-b+discriminant)/2/a;
      if(t1 >= 0 && t1 <= 1) {
        temp[X] += t1*d[X];
        temp[Y] += t1*d[Y];
      }

      if(t2 >= 0 && t2 <= 1) {
        temp[X] += t2 * d[X];
        temp[Y] += t2 * d[Y];
      }

      lk = temp;
      return true;
    }
    else {
      return false;
    }
  }

  int find_side(double heading, std::vector<double> robot) { //need to have lk defined already ... also return 1 or -1 for direction
        //this function returns the side in which the point is relative to robot
        double temp;
        std::vector<double> B = {0,0};
        B[X] = robot[X] + cos(heading);
        B[Y] = robot[Y] + sin(heading);

        temp = (B[Y] - robot[Y]) * (lk[X] - robot[X]) - (B[X] -robot[X]) * (lk[Y] - robot[Y]); //crossproduct of B and lk with robot as reference point

        if(temp > 0) {
          return 1; //positive direction (right) ->
        }
        else {
          return -1; //negative direction (left) <-
        }
  }

  void find_curvature(double heading,std::vector<double> robot) {//calculating curvature
      double a,b,c; //ax +by +c = 0 standard form
      std::vector<double> delta; //delta(difference vector component from lk and robot)

      delta[X] = lk[X] - robot[X];
      delta[Y] = lk[Y] - robot[Y];

      double d;
      a = -tan(heading);
      b = 1;
      c = tan(heading) * robot[X] - robot[Y];

      d = std::abs(a * lk[X] + b * lk[Y] + c);
      d = d / sqrt(a * a + b* b);
      curvature = 2 * d / pow(delta[X] * delta[X] + delta[Y] * delta[Y], 2); // 2 * x / L / L = curvature
      curvature *= find_side(heading, robot); //make sure curvature is on the correct side
    }


};

#endif
