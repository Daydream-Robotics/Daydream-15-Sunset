#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"
#include "autonomous.hpp"
#include "pid.hpp"
#include "intake.h"
#include "autotuner.hpp"

void initialize() {
	pros::lcd::initialize();
	imu.reset();

	while(imu.is_calibrating()) {
		pros::delay(20);
	}

	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	pros::delay(500); // Allow system to settle
	Autonomous auton = Autonomous();
	// AutoTuner::run(auton);

	// while(true) {
	// 	auton.updatePose();
	// 	pros::lcd::print(1, "X: %lf", auton.pos_x);
	// 	pros::lcd::print(2, "Y: %lf", auton.pos_y);
	// 	pros::lcd::print(3, "Heading: %lf", auton.heading);
	// 	pros::delay(50);
	// }

	// auton.travelToPoint(48, 0, 200);
	// controller.rumble("-");
	// auton.travelToPoint(48, -48, 200);
	// controller.rumble("-");
	// auton.travelToPoint(0, -48, 200);
	// controller.rumble("-");
	// auton.travelToPoint(0, 0, 200);
	// controller.rumble("-");
	// auton.turnTo(0);

	// initilize and line up with bottom right matchloader
	centerScore.set_value(true);
	descorer.set_value(true);
	lowIntake.move_velocity(200);
	auton.travelToPoint(34.33, 0, 200);
	lowIntake.move_velocity(0);
	centerScore.set_value(false);


	// matchload from bottom right
	move_intake(200, 200, -200);
	auton.travelToPoint(34, 12, 200, false, 2);
	leftMotors.move_velocity(50);
	rightMotors.move_velocity(50);
	pros::delay(3000);

	// score on bottom right of long goal
	auton.travelToPoint(36, -18, 200, true, 4);  // lo 35
	unloadLongGoal(auton);
	move_intake(0, 0, 0);

	// line up with top right red balls
	auton.travelToPoint(36, -10, 200);
	auton.travelToPoint(22, -10, 200);
	auton.travelToPoint(22, -80, 200);

	// retrieve top right red balls
	auton.travelToPoint(40, -90, 200, false, 2);

	// line up with top right matchloader
	auton.travelToPoint(33, -90, 200, true);

	// matchload from top right
	auton.travelToPoint(33, -100, 200, false, 2);

	// score on top right of long goal
	auton.travelToPoint(36.33, -75, 200, true);
	move_intake(0, 0, 0);

	// move to park
	auton.travelToPoint(22, -90, 200);
	auton.travelToPoint(22, 10, 200, true);

	// park
	auton.turnTo(170);
	leftMotors.move_velocity(200);
	rightMotors.move_velocity(200);
	pros::delay(1200);
	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);









	// ==========================================
	// was working
	// =========================================

	// double last_distance_moved;
	// pros::delay(10);

	// // line up for botton right unloader
	// centerScore.set_value(true);
	// descorer.set_value(true);
	// lowIntake.move_velocity(200);
	// auton.travel(33, 200, 0, -1); //33.5
	// centerScore.set_value(false);
	// lowIntake.move_velocity(0);

	// // turn towards unloader
	// unloader.set_value(true);
	// auton.turnTo(93 );
	
	// // go to unloader
	// lowIntake.move_velocity(200);
	// midIntake.move_velocity(30);
	// highIntake.move_velocity(-50);

	// double before_loader_x = auton.pos_x;
    // last_distance_moved = auton.travel(60, 150, 90, 1);
	// pros::lcd::print(1, "Distance in loader: %lf", last_distance_moved);
	// // check if not entered loader
	// if (last_distance_moved < 11) {
	// 	int counter = 0;
	// 	do {
	// 		double back_distance = auton.travel(-3, 200, 90,0.7);
	// 		last_distance_moved = auton.travel(70, 200, 90, 0.7) + back_distance;
	// 		pros::lcd::print(2, "Distance in loader: %lf", last_distance_moved);
	// 	} while (last_distance_moved < 1 and ++counter < 2);
	// }
	// auton.travel(-1, 200, 90);
	
	
	// // hump loader
	// // pros::delay(200);
	// hump(auton);
	
	// double x_diff = auton.pos_x - before_loader_x;
	// pros::lcd::print(1, "X Diff %lf", x_diff);

	// // go to long goaln
	// int target_heading = 200 - x_diff*6;
	// pros::lcd::print(2, "Target Heading %d", target_heading);
	// auton.travel(-50, 200, target_heading, 2); // s:50 t:3.25
	// unloadLongGoal(auton);
	// unloader.set_value(false);

	// // go to upper right balls
	// move_intake(-200, -200, 0);
	// auton.travel(10, 50, 90, -1);
	// auton.turnTo(0);
	// move_intake(200, 200, 200);
	// // auton.travel(-15, 50, 0, -1);
	// auton.travelToX(22, 50, 0);
	// auton.turnTo(-90);
	// auton.travel(77, 150, -90); // 71, 150, -90, -1
	// // move_intake(0, 0, 0);
	
	// // get top right balls
	// auton.turnTo(-15);
	// // unloader.set_value(false);
	// move_intake(200, 200, -200);
	// auton.travel(38, 70, -15, 2.1); // 38 70 9 1.9
	// // auton.travel(4, 200, 0, 0.5);
	// auton.travel(-2, 200, 0, 0.5);
	// auton.travel(4, 200, 0, 0.5);

	
	// pros::delay(200);
	// // line up with upper right unloader
	// // auton.travel(-15, 50, 0);//15
	// auton.travelToX(33, 50, 0);
	// unloader.set_value(true);
	// auton.turnTo(-86);
	// pros::delay(50);

	// //travel to top right unloader
	// lowIntake.move_velocity(200);
	// midIntake.move_velocity(200);
	// highIntake.move_velocity(-50);
	
	// pros::lcd::print(1, "Distance in loader 1: %lf", last_distance_moved);
	// before_loader_x = auton.pos_x;
	// last_distance_moved = auton.travel(30, 200, -90, 1);
	// if (last_distance_moved < 11) {
	// 	int counter = 0;
	// 	do {
	// 		double back_distance = auton.travel(-3, 200, -90, 0.7);
	// 		last_distance_moved = auton.travel(50, 200, -90, 0.7) + back_distance;
	// 		pros::lcd::print(2, "Distance in loader 2: %lf", last_distance_moved);
	// 	} while (last_distance_moved < 1 and ++counter < 2);
	// }
	// auton.travel(-1, 50, -90);
	// // hump top right unloader
	// hump(auton);

	// x_diff = auton.pos_x - before_loader_x;
	// pros::lcd::print(1, "X Diff %lf", x_diff);

	// // go to top right of long goal
	// target_heading = -95 - x_diff*5;
	// pros::lcd::print(2, "Target Heading %d", target_heading);
	// auton.travel(-50, 200, target_heading, 2);

	// unloadLongGoal(auton);

	// // move from goal to park area
	// auton.travel(10, 50, -90, -1);
	// auton.turnTo(0);
	// auton.travelToX(18, 50, 0);
	// // auton.travel(-19, 50, 0, -1);
	// pros::lcd::print(5, "X_pos: %lf", auton.pos_x);
	// auton.turnTo(90);
	// unloader.set_value(false);
	// move_intake(200, 200, 200);
	// auton.travel(95, 200, 90, 2.25 );

	// //park
	// auton.turnTo(170);
	// leftMotors.move_velocity(200);
	// rightMotors.move_velocity(200);
	// pros::delay(1200);
	// leftMotors.move_velocity(0);
	// rightMotors.move_velocity(0);












	// leftMotors.move_velocity(0);
	// rightMotors.move_velocity(0);
	// pros::delay(200);








	
	pros::delay(200);
	controller.rumble("..");
	
	
	
	


}

void opcontrol() {
	initialize();
	pros::delay(500); // Allow system to settle


	Autonomous auton = Autonomous();
	// AutoTuner::run(auton);

	// while(true) {
	// 	auton.updatePose();
	// 	pros::lcd::print(1, "X: %lf", auton.pos_x);
	// 	pros::lcd::print(2, "Y: %lf", auton.pos_y);
	// 	pros::lcd::print(3, "Heading: %lf", auton.heading);
	// 	pros::delay(50);
	// }

	auton.travelToPoint(48, 0, 200);
	controller.rumble("-");
	auton.travelToPoint(48, -48, 200);
	controller.rumble("-");
	auton.travelToPoint(0, -48, 200);
	controller.rumble("-");
	auton.travelToPoint(0, 0, 200);
	controller.rumble("-");
	auton.turnTo(0);

	// // Set chassis brake mode to coast
	// leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	// rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// // Set intake motors to brake
	// lowIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// midIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// highIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	

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
		pros::delay(seconds * 2000);

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