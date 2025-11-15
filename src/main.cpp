#include "main.h"
#include "subsystems.h"
#include "constants.h"

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);


	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){
		// Get joystick values
		int leftY = controller.get_analog(ANALOG_LEFT_Y);
		int rightY = controller.get_analog(ANALOG_RIGHT_Y);

		// Dead zone for both motors
		if(abs(leftY) > DEADZONE) {
			leftMotors.move(leftY);
		} else {
			leftMotors.move(0);
		}

		if(abs(rightY) > DEADZONE) {
			rightMotors.move(rightY);
		} else { 
			rightMotors.move(0);
		}

		// Main intake
		if (controller.get_digital(DIGITAL_L1)){
			mainIntake.move(-127);
		} else if (controller.get_digital(DIGITAL_L2)){
			mainIntake.move(127);
		} else {
			mainIntake.move(0);
		}

		// Front intake 
		if (controller.get_digital(DIGITAL_R1)){
			frontIntake.move(127);
		} else if (controller.get_digital(DIGITAL_R2)){
			frontIntake.move(-127);
	    } else {
			frontIntake.move(0);
		}


		// Delay added to prevent crashing
		pros::delay(20);
	}
}