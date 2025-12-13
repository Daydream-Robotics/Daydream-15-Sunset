#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"

void initialize() {
	pros::lcd::initialize();
	imuUpper.reset();
	imuLower.reset();

	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	while (imuUpper.is_calibrating()) {
		pros::delay(20);
	}

	/* - - - - - - - - - - - - - - [MATCH LOADER] - - - - - - - - - - - - - - */

	// move to match loader
	slew_move_pid(-14);
	slew_turn_pid(86);
	unloader.extend();

	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.0);

	move(25, 1.5);

	// get blocks
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 3.0);

	/* - - - - - - - - - - - - - - [LONG GOAL] - - - - - - - - - - - - - - */

	// move to long goal
	slew_move_pid(-10);

	move(-10, 2.0);

	// score 4 team color blocks
	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.5);
	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.75);
	
	unloader.retract();

	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0);
	
	/* - - - - - - - - - - - - - - [CENTER] - - - - - - - - - - - - - - */

	// move back from long goal
	slew_move_pid(8);

	moveIntake(STOP, STOP, STOP, STOP, 0);

	// got to middle
	slew_turn_pid(-45);
	slew_move_pid(12);

	slew_turn_pid(-86);
	slew_move_pid(8);
}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// Set intake motors to brake
	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainUpperIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainLowerIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	bool centerScoreToggle = false;

	while(true){

		/* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

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

		/* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L1)) {
			unloader.toggle();
		}

		/* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L2)) {
			centerScore.toggle();
			centerScoreToggle = !centerScoreToggle;
		}

		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */

		if (centerScoreToggle) {
			moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -MAX_VOLTAGE); // scoring center high (L2)
		} else if (controller.get_digital(DIGITAL_R1)) {
			moveIntake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE, -MAX_VOLTAGE); // intaking, top wheels reversed
		} else if (controller.get_digital(DIGITAL_R2)) {
			moveIntake(MAX_VOLTAGE, MAX_VOLTAGE, HIGH_VOLTAGE, MAX_VOLTAGE); // scoring long goals
		} else if (controller.get_digital(DIGITAL_A)) {
			moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE); // outtaking / scoring center low
		} else {
			moveIntake(STOP);
		}

		// Delay added to prevent crashing
		pros::delay(20);
	}
}

void moveIntake(int front, int mainUpper, int mainLower, int back, double seconds) {

	// check for stalling later and stop motors if stalling

	frontIntake.move(front);
	mainUpperIntake.move(mainUpper);
	mainLowerIntake.move(mainLower);
	backIntake.move(back);

	if (seconds != 0) {
		pros::delay(seconds * 1000);

		frontIntake.move(STOP);
		mainUpperIntake.move(STOP);
		mainLowerIntake.move(STOP);
		backIntake.move(STOP);
	}
}