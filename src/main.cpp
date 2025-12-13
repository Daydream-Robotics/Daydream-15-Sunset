#include "main.h"
#include "subsystems.h"
#include "constants.h"

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	while (imuUpper.is_calibrating() || imuLower.is_calibrating()) {
		pros::delay(20);
	}
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

	bool unloaderToggle = false, unloaderLatch = false, centerScoreToggle = false, centerScoreLatch = false, descoreActive = false;

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

		// Match unloader
		if (unloaderToggle) {
			unloader.set_value(true); // turns clamp solenoid on
		} else {
			unloader.set_value(false); // turns clamp solenoid off
		}

		pros::delay(10);

		if (controller.get_digital_new_press(DIGITAL_L1)) {
			if(!unloaderLatch){ // if latch is false, flip toggle one time and set latch to true
				unloaderToggle = !unloaderToggle;
				unloaderLatch = true;
			}
		}
		else {
			unloaderLatch = false; // once button is released then release the latch too
		}

		if (centerScoreToggle) {
			centerScore.set_value(true); // turns clamp solenoid on
		} else {
			centerScore.set_value(false); // turns clamp solenoid off
		}

		pros::delay(10);

		if (controller.get_digital_new_press(DIGITAL_L2)) {
			if(!centerScoreLatch){ // if latch is false, flip toggle one time and set latch to true
				centerScoreToggle = !centerScoreToggle;
				centerScoreLatch = true;
			}
		}
		else {
			centerScoreLatch = false; // once button is released then release the latch too
		}

		// start center goal scoring
		if (centerScoreToggle) {
			moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, -MAX_VOLTAGE); 
		} else if (controller.get_digital(DIGITAL_R1)) {
			if(descoreActive) {
				moveIntake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
			} else {
				moveIntake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE, -MAX_VOLTAGE);
			}
		} else if (controller.get_digital(DIGITAL_R2)) {
			moveIntake(MAX_VOLTAGE, MAX_VOLTAGE, HIGH_VOLTAGE, MAX_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_A)) {
			moveIntake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE);
		} else {
			moveIntake(STOP);
		}

		// Descore and signal open scorer (ADD WITH DESCORER)

		// if (controller.get_digital_new_press(DIGITAL_R2)) {
		// 	// Move descore piston when added
		// 	descoreActive = !descoreActive;
		// }

		// Delay added to prevent crashing
		pros::delay(20);
	}
}

void moveIntake(int front, int mainUpper, int mainLower, int back) {

		// check for stalling later and stop motors if stalling

		frontIntake.move(front);
		mainUpperIntake.move(mainUpper);
		mainLowerIntake.move(mainLower);
		backIntake.move(back);
	}