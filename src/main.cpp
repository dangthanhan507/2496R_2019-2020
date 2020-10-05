#include "custom/config.hpp"
#include "custom/lib_drive.hpp"
#include "custom/lib_auton.hpp"
void initialize() {
	enc_l.reset();
	enc_r.reset();
	delay(500);
	pot_offset = pot.get_value();
}

//test comment 123
void disabled() {

}

void competition_initialize() {
	while(imu.is_calibrating()) {}
}

double angle = 0;

void autonomous() {
	//while(imu.is_calibrating()) {}
	//blue_auton();
	/*
	pwr_intake(0);
	mtr_chasBL.move_velocity(127);
	mtr_chasFL.move_velocity(127);
	mtr_chasBR.move_velocity(127);
	mtr_chasFR.move_velocity(127);
	delay(750);
	pwr_intake(-100);
	mtr_chasBL.move_velocity(-127);
	mtr_chasFL.move_velocity(-127);
	mtr_chasBR.move_velocity(-127);
	mtr_chasFR.move_velocity(-127);
	delay(750);
	pwr_intake(0);
	mtr_chasBL.move_velocity(0);
	mtr_chasFL.move_velocity(0);
	mtr_chasBR.move_velocity(0);
	mtr_chasFR.move_velocity(0);
	*/
	while(true) {
		angle = (enc_l.get_value() - enc_r.get_value())/ WHEELBASE * RAD_TO_DEG;
		printf("angle: %.2f\n", angle);
	}


}

bool lift_auto = false;
void opcontrol() {




	mtr_rollR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_rollL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	mtr_lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	while (true) {


		drive_chassis();

		if(control.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) lift_auto = !lift_auto;
		lift_auto? pid_lift() : drive_lift();

		drive_intake();
		drive_tray();

		delay(15);
	}
}
