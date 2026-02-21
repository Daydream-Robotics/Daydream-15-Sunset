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
	double last_distance_moved;
	pros::delay(10);

	// line up for botton right unloader
	centerScore.set_value(true);
	descorer.set_value(true);
	lowIntake.move_velocity(100);
	auton.travel(33.5, 100, 0, -1); //33
	centerScore.set_value(false);
	lowIntake.move_velocity(0);

	// turn towards unloader
	unloader.set_value(true);
	auton.turnTo(90);
	
	// go to unloader
	lowIntake.move_velocity(100);
	midIntake.move_velocity(30);
	highIntake.move_velocity(-50);

	double before_loader_x = auton.pos_x;
    last_distance_moved = auton.travel(60, 200, 90, 1);
	pros::lcd::print(1, "Distance in loader: %lf", last_distance_moved);
	// check if not entered loader
	if (last_distance_moved < 11) {
		int counter = 0;
		do {
			double back_distance = auton.travel(-3, 200, 90,0.7);
			last_distance_moved = auton.travel(70, 200, 90, 0.7) + back_distance;
			pros::lcd::print(2, "Distance in loader: %lf", last_distance_moved);
		} while (last_distance_moved < 1 and ++counter < 2);
	}
	auton.travel(-1, 100, 90);
	
	
	// hump loader
	// pros::delay(100);
	hump(auton);
	
	double x_diff = auton.pos_x - before_loader_x;
	pros::lcd::print(1, "X Diff %lf", x_diff);

	// go to long goaln
	int target_heading = 100 - x_diff*6;
	pros::lcd::print(2, "Target Heading %d", target_heading);
	auton.travel(-50, 200, target_heading, 2); // s:50 t:3.25
	unloadLongGoal(auton);
	unloader.set_value(false);

	// go to upper right balls
	move_intake(-100, -100, 0);
	auton.travel(10, 50, 90, -1);
	auton.turnTo(0);
	move_intake(100, 100, 100);
	// auton.travel(-15, 50, 0, -1);
	auton.travelToX(22, 50, 0);
	auton.turnTo(-90);
	auton.travel(71, 150, -90, -1); // 83 150 -90 -1
	// move_intake(0, 0, 0);
	
	// get top right balls
	auton.turnTo(-15);
	// unloader.set_value(false);
	move_intake(100, 100, -100);
	auton.travel(38, 70, -15, 2.1); // 38 70 9 1.9
	// auton.travel(4, 100, 0, 0.5);
	auton.travel(-2, 100, 0, 0.5);
	auton.travel(4, 100, 0, 0.5);

	
	pros::delay(100);
	// line up with upper right unloader
	// auton.travel(-15, 50, 0);//15
	auton.travelToX(33.5, 50, 0);
	unloader.set_value(true);
	auton.turnTo(-90);
	pros::delay(50);

	//travel to top right unloader
	lowIntake.move_velocity(100);
	midIntake.move_velocity(100);
	highIntake.move_velocity(-50);
	
	pros::lcd::print(1, "Distance in loader 1: %lf", last_distance_moved);
	before_loader_x = auton.pos_x;
	last_distance_moved = auton.travel(30, 200, -90, 1);
	if (last_distance_moved < 11) {
		int counter = 0;
		do {
			double back_distance = auton.travel(-3, 200, -90, 0.7);
			last_distance_moved = auton.travel(50, 200, -90, 0.7) + back_distance;
			pros::lcd::print(2, "Distance in loader 2: %lf", last_distance_moved);
		} while (last_distance_moved < 1 and ++counter < 2);
	}
	auton.travel(-1, 50, -90);
	// hump top right unloader
	hump(auton);

	x_diff = auton.pos_x - before_loader_x;
	pros::lcd::print(1, "X Diff %lf", x_diff);

	// go to top right of long goal
	target_heading = -95 - x_diff*5;
	pros::lcd::print(2, "Target Heading %d", target_heading);
	auton.travel(-50, 200, target_heading, 2);

	unloadLongGoal(auton);

	// move from goal to park area
	auton.travel(10, 50, -90, -1);
	auton.turnTo(0);
	auton.travelToX(18, 50, 0);
	// auton.travel(-19, 50, 0, -1);
	pros::lcd::print(5, "X_pos: %lf", auton.pos_x);
	auton.turnTo(90);
	unloader.set_value(false);
	move_intake(100, 100, 100);
	auton.travel(95, 200, 90, 2.25 );

	//park
	auton.turnTo(170);
	leftMotors.move_velocity(200);
	rightMotors.move_velocity(200);
	pros::delay(1100);
	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);












	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);
	pros::delay(100);








	
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