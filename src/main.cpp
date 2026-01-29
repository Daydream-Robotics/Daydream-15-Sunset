#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "autoFunctions.h"
#include "odometry.h"

void initialize() {
	pros::lcd::initialize();
	imuUpper.reset();
	// imuLower.reset();

	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	centerScore.set_value(true);
	descorer.set_value(true);

	// move(200,200,1);
	move_time_s(40, 3.5, 2 ,1);
	turn_pid(90);

	unloader.toggle();
	centerScore.set_value(false);
	move_intake(MAX_VOLTAGE, MAX_VOLTAGE, -HIGH_VOLTAGE); 
	pros::delay(1000);

	forward_backward_move(40, 1);

	forward_backward_move(-25, 0.25);
	forward_backward_move(25, 0.25);

	pros::delay(3000);

	// Move to long goal
	unloader.toggle();
	move_time_s(-40, 2.5, 1, 1);

	// Unlodge Balls
	move_intake(STOP, -HIGH_VOLTAGE, -HIGH_VOLTAGE);
	pros::delay(400);
	
	// Score on long goal
	move_intake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE);
	pros::delay(3000);
	move_intake(STOP, STOP, STOP);

	// Move forward to align for park
	move_time_s(40, 1, 1, 1);
	turn_pid(0);

	// Back towards parking zone
	move_time_s(-50, 1.1, 0.5, 1);
	turn_pid(90);

	move_time(40, 1.7);
	turn_pid(0);

	move_time(-120, 1.75);

}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// Set intake motors to brake
	lowIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	midIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	highIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	

	bool centerScoreToggle = false;

	while(true) {

		/* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

		drive(DriveType::TANK);

		/* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L1)) {
			unloader.toggle();
		}

		/* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L2)) {
			centerScore.toggle();
			centerScoreToggle = !centerScoreToggle;
		}

		/* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_X)) {
			descorer.toggle();
		}

		if (controller.get_digital_new_press(DIGITAL_A)) {
			centerScore.set_value(true);
		}

		if (controller.get_digital_new_press(DIGITAL_B)) {
			centerScore.set_value(false);
		}

		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */

		if (centerScoreToggle) {
			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE); // scoring center high (L2)
		} else if (controller.get_digital(DIGITAL_R1)) {
			move_intake(MAX_VOLTAGE, MAX_VOLTAGE, -HIGH_VOLTAGE); // intaking, top wheels reversed
		} else if (controller.get_digital(DIGITAL_R2)) {
			move_intake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE); // scoring long goals
		} else if (controller.get_digital(DIGITAL_A)) {
			move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, HIGH_VOLTAGE); // outtaking / scoring center low
		} else {
			move_intake(STOP);
		}

		// Delay added to prevent crashing
		pros::delay(20);
	}
}

void move_intake(int low, int mid, int high, double seconds) {

	// check for stalling later and stop motors if stalling
	lowIntake.move(low);
	midIntake.move(mid);
	highIntake.move(high);


	if (seconds != 0) {
		pros::delay(seconds * 1000);

		lowIntake.move(STOP);
		midIntake.move(STOP);
		highIntake.move(STOP);
	}
}

void drive(DriveType type) {
	switch (type) {
		case DriveType::TANK: {
			// Get joystick values
			int leftY = controller.get_analog(ANALOG_LEFT_Y);
			int rightY = controller.get_analog(ANALOG_RIGHT_Y);

			// Dead zone for both motors
			if(abs(leftY) > DEADZONE) {
				leftMotors.move(leftY);
			} else {
				leftMotors.move(STOP);
			}

			if(abs(rightY) > DEADZONE) {
				rightMotors.move(rightY);
			} else { 
				rightMotors.move(STOP);
			}
			break;
		}
		case DriveType::SPLIT_ARCADE: {
			// Get joystick values
			int power = controller.get_analog(ANALOG_LEFT_Y);
			int turn = controller.get_analog(ANALOG_RIGHT_X);

			int left = power + turn;
			int right = power - turn;

			// Dead zone for both motors
			if(abs(left) > DEADZONE) {
				leftMotors.move(left);
			} else {
				leftMotors.move(0);
			}

			if(abs(right) > DEADZONE) {
				rightMotors.move(right);
			} else { 
				rightMotors.move(0);
			}
			break;
		}
	}
}