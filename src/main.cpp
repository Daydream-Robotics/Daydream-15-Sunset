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

	while (imuUpper.is_calibrating() /* || imuLower.is_calibrating()*/) {
		pros::delay(20);
	}

	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.5);

	// move to match loader
	test_move_pid(-17); // inches
	slew_turn_pid(86); // degrees
	unloader.extend();
	move(25, 1.5);

	// get blocks
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 3);

	test_move_pid(-10);

	move(-10, 2.0);

	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.5);
	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 2.5);
	
	unloader.retract();

	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0);
	
	// move back from long goal
	test_move_pid(8);

	moveIntake(STOP, STOP, STOP, STOP, 0);

	// got to other side
	slew_turn_pid(3);
	test_move_pid(45);

	// get 2 blue
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 0);

	move(10, 2.5);

	move(0, 1.0);

	moveIntake(STOP, STOP, STOP, STOP, 0);

	// go to 2nd match unloader
	test_move_pid(-5.5);

	slew_turn_pid(86);
	unloader.extend();
	move(25, 2.0);

	// get blocks in 2nd long goal
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -HIGH_VOLTAGE, 3);

	test_move_pid(-10);

	move(-10, 1.5);

	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.5);
	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0.1);
	moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 2.5);
	
	unloader.retract();

	moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, 0);

}

void opcontrol() {
	autonomous();

	while(true) {
		pros::delay(50);
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