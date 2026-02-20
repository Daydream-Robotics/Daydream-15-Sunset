#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"
#include "autonomous.hpp"
#include "pid.hpp"
#include "intake.h"

void initialize() {
	pros::lcd::initialize();
	imu.reset();

	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	Autonomous auton = Autonomous();
	pros::delay(100);

	// line up for botton right unloader
	centerScore.set_value(true);
	lowIntake.move_velocity(100);
	auton.travel(34.5, 100, 0, -1);
	centerScore.set_value(false);
	lowIntake.move_velocity(0);

	// turn towards unloader
	unloader.set_value(true);
	auton.turnTo(90);
	
	// go to unloader
	lowIntake.move_velocity(100);
	midIntake.move_velocity(30);
	highIntake.move_velocity(-50);

    auton.travel(20, 200, 90, 1);


	// hump loader
	loadFromLoader(auton);
	// pros::delay(2000);
	// go to long goal
	auton.travel(-35, 50, 90, 3.25);
	// auton.turnTo(93);
	// auton.travel(-20, 100, 95, 2);
	unloadLongGoal(auton);

	// go to upper right balls
	auton.travel(10, 50, 90, -1);
	auton.turnTo(0);
	auton.travel(-15, 50, 0, -1);
	auton.turnTo(-90);
	auton.travel(83, 150, -90, -1);
	auton.turnTo(0);
	unloader.set_value(false);

	// get top right balls
	move_intake(100, 100, -100);
	auton.travel(38, 70, 0, 2);
	auton.travel(-2, 100, 0, 1);
	auton.travel(4, 100, 0, 1);
	move_intake(0, 0, 0);
	
	pros::delay(100);
	// line up with upper right unloader
	auton.travel(-15, 50, 0, -1);//lo hi 17
	unloader.set_value(true);
	auton.turnTo(-90);

	//travel to unloader
	lowIntake.move_velocity(100);
	midIntake.move_velocity(100);
	highIntake.move_velocity(-50);

	auton.travel(20, 200, -90, 1);

	//hump unloader
	loadFromLoader(auton);

	// go to long goal
	auton.travel(-35, 50, 200, 3.25);

	unloadLongGoal(auton);

	auton.travel(10, 50, 90, 4);








	
	pros::delay(100);
	controller.rumble("..");
	
	
	
	


}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// Set intake motors to brake
	lowIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	midIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	highIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	

// 	bool centerScoreToggle = false;

// 	while(true) {

// 		/* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

// 		drive(DriveType::TANK);

// 		/* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

// 		if (controller.get_digital_new_press(DIGITAL_L1)) {
// 			unloader.toggle();
// 		}

// 		/* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

// 		if (controller.get_digital_new_press(DIGITAL_L2)) {
// 			centerScore.toggle();
// 			centerScoreToggle = !centerScoreToggle;
// 		}

// 		/* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

// 		if (controller.get_digital_new_press(DIGITAL_X)) {
// 			descorer.toggle();
// 		}

// 		if (controller.get_digital_new_press(DIGITAL_A)) {
// 			centerScore.set_value(true);
// 		}

// 		if (controller.get_digital_new_press(DIGITAL_B)) {
// 			centerScore.set_value(false);
// 		}

// 		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */

// 		if (centerScoreToggle) {
// 			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE); // scoring center high (L2)
// 		} else if (controller.get_digital(DIGITAL_R1)) {
// 			move_intake(MAX_VOLTAGE, MAX_VOLTAGE, -HIGH_VOLTAGE); // intaking, top wheels reversed
// 		} else if (controller.get_digital(DIGITAL_R2)) {
// 			move_intake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE); // scoring long goals
// 		} else if (controller.get_digital(DIGITAL_A)) {
// 			move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, HIGH_VOLTAGE); // outtaking / scoring center low
// 		} else {
// 			move_intake(STOP);
// 		}

// 		// Delay added to prevent crashing
// 		pros::delay(20);
// 	}
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