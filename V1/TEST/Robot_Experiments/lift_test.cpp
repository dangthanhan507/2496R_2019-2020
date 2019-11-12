#include "../../include/config.h"
#include "../../include/auton_lib.h"
#include "../../include/motion_profile.h"
#include <cmath>

competition Competition;


#define FF_LIFT 6.784689
#define AFF_LIFT 0
#define INT_LIFT 500

//negative is up
//positive is down
double feedforward(int pot_velo) {
  if(pot_velo <= 0) {
    return FF_LIFT * pot_velo -INT_LIFT;
  }
  else {
    return FF_LIFT * pot_velo - INT_LIFT;
  }
}

void pre_auton() {
  lift_min = p_LiftR.value(analogUnits::range12bit);
  sleepMs(1000);
}

void autonomous() {

}



void drivercontrol() {
  PID lift_pid(45,0,0);
  MotionProfile lift_mp(1500,1500);
  pre_auton();
  control.Screen.clearScreen();

  bool mp_status = true;
  double start_target = 0;
  double end_target = -200;
  double velo = 0;
  double accel = 0;
  double lift_tick = 0;
  double prev_lift_tick = 0;
  double projected_tick = 0;
  double pwm = 0;
  double velo_a = 0;
  double sgn = -1;

  while(true) {
    if(re_press(control.ButtonX.pressing(), re_status[INDEX_X])) {
      start_target = end_target;
      end_target -= 200;
      mp_status = true;
      sgn = -1;
    }
    else if(re_press(control.ButtonB.pressing(), re_status[INDEX_B])) {
      start_target = end_target;
      end_target += 200;
      mp_status = true;
      sgn = 1;
    }
    else if(re_press(control.ButtonY.pressing(), re_status[INDEX_Y])) {
      start_target = end_target;
      end_target -= 25;
      mp_status = true;
      sgn = -1;
    }
    else if(re_press(control.ButtonA.pressing(), re_status[INDEX_A])) {
      start_target = end_target;
      end_target += 25;
      mp_status = true;
      sgn = 1;
    }

    prev_lift_tick = lift_tick;
    lift_tick = p_LiftR.value(analogUnits::range12bit) - lift_min;

    velo_a = (lift_tick - prev_lift_tick)*100;

    if(mp_status) lift_mp.trapezoid_lift(velo, accel, projected_tick, end_target, start_target);
    pwm = feedforward(sgn * velo) + lift_pid.Calculate(sgn * projected_tick, lift_tick, 0);
    v_lift(pwm);


    if(velo == 0) mp_status = false;
    printf("velo: %.2f      velo_a: %.2f      ",velo,velo_a);
    printf("projected_tick: %.2f     ",projected_tick);
    printf("tick: %.2f\n",lift_tick);
    printf("\n");

    sleepMs(10);
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous); //these should be treated as threads
  Competition.drivercontrol(drivercontrol);
}
