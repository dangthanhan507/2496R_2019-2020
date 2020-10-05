#include "custom/config.hpp"
#include "classes/PID.hpp"
#define POT_TO_DEGREES 0

Controller control(E_CONTROLLER_MASTER);


//motor(port, gear, reverse, measurement)
Motor mtr_chasBL(4, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasFL(20, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasBR(6, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_COUNTS);
Motor mtr_chasFR(7, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_lift(3, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_tilt(10, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_rollL(5, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_COUNTS);
Motor mtr_rollR(13, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_COUNTS);

//encoder(port,adjacent_port, reverse)

//ADI(port)
ADIAnalogIn pot(1);
ADIDigitalIn limit(7);
ADIDigitalIn bump(2);
ADIEncoder enc_r(3,4,true);
ADIEncoder enc_l(5,6,false);
ADIDigitalIn limit_index(8);

Imu imu(12);

int pot_offset = 0;

double preset_heights[3] = {LIFTH_BOTTOM, LIFTH_SMALLTOWER, LIFTH_BIGTOWER};
PID autolift(KP_LIFT, KI_LIFT, KD_LIFT);
int index_heights = 0;
double lift_value = 0;
void pid_lift() {


  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
    index_heights++;
    index_heights = fmin(index_heights , sizeof(preset_heights) / sizeof(*preset_heights) - 1 );
  }

  if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
    index_heights--;
    index_heights = fmax(index_heights , 0);
  }

  autolift.Calculate(preset_heights[index_heights], pot.get_value()-pot_offset, LIMIT_LIFT, LIFT_MAX);
  if(std::abs(autolift.error) > 20) lift_value = -autolift.Calculate(preset_heights[index_heights], pot.get_value()-pot_offset, LIMIT_LIFT, LIFT_MAX) + GRAVFF;

  mtr_lift.move_voltage(lift_value); // it move gamer arm
  if(index_heights == 0 && std::abs(autolift.error) < 15) mtr_lift.move_velocity(0);
  //target = preset_heights[index_heights]


}
