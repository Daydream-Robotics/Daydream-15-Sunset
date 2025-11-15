#include "main.h"
#include "odometry.h"
#include "constants.h"
#include "subsystems.h"

#include "stdlib.h"

//#include <algorithm>
#include <cmath>

double heading;
double optimized_angle;
double temp_angle;


void turn_pid(double target) {
	int correctCount = 0;
	heading = get_yaw_quaternion();
	double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_prev_error = 0, turn_PID = 0;
	
	while (correctCount <= 10) {
		
		heading = get_yaw_quaternion() - 180;
		optimized_angle = target - heading;
			
		if(target > heading){
			temp_angle = ((360 - target) + heading) * -1;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		} else {
			temp_angle = (360 - heading) + target;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		}

		pros::lcd::print(1, "Optimized Angle: %lf", optimized_angle);
		pros::lcd::print(2, "Heading Angle: %lf", heading);
		pros::lcd::print(3, "Correct Count: %d", correctCount);

		// proportion
		turn_error = optimized_angle;

		// integral
		turn_total_error += turn_error;

		// derivative
		turn_derivative = turn_error - turn_prev_error;
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = ((TURN_KP * turn_error) / 360);
		turn_PID += (TURN_KD * turn_derivative);
		turn_PID += (TURN_KI * turn_total_error);

		int turnSpeed = turn_PID * 127;
	
		if(abs(optimized_angle) > 0.15) {
			int lowerBound = std::max(abs(turnSpeed), 10);
			turnSpeed = std::min(lowerBound, 64);
			leftMotors.move((int)copysign(turnSpeed, turn_PID));
			rightMotors.move(-(int)copysign(turnSpeed, turn_PID));
		} else {
			leftMotors.move(0);
			rightMotors.move(0);
		}
		
		if(abs(optimized_angle) <= 0.2) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}

void move(int seconds) {
	const auto start_time = std::chrono::steady_clock::now();
	const auto timer_duration = std::chrono::seconds(seconds);

	while ((std::chrono::steady_clock::now() - start_time) < timer_duration)
	{
		leftMotors.move(-20);
		rightMotors.move(-25);
	}
}

void move_pid(double target) {
	int correctCount = 0;
	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);

	// Convert inches to degrees
	target = inchesToDegrees(target);

	// Make movement relative
	target -= ((leftMotors.get_position() + rightMotors.get_position()) / 2);
	
	double move_error = 0, move_total_error = 0, move_derivative = 0, move_prev_error = 0, effort = 0;
	
	while (correctCount <= 10) {
		
		double position = (leftMotors.get_position() + rightMotors.get_position()) / 2;
		double dist = target - position;

		// proportion
		move_error = dist;

		// integral
		move_total_error += move_error;

		// derivative
		move_derivative = move_error - move_prev_error;
		
		// get prev error for next instance
		move_prev_error = move_error;

		effort = ((MOVE_KP * move_error) / 360);
		effort += (MOVE_KD * move_derivative);
		effort += (MOVE_KI * move_total_error);

		int moveSpeed = effort * 127;
	
		if(abs(dist) > 0.45) {
			int lowerBound = std::max(abs(moveSpeed), 20);
			moveSpeed = std::min(lowerBound, 127);
			leftMotors.move((int)copysign(moveSpeed, effort));
			rightMotors.move((int)copysign(moveSpeed + 5, effort));
		} else {
			leftMotors.move(0);
			rightMotors.move(0);
		}
		
		if(abs(dist) <= 0.5) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}

void move_pid_back(double target) {
	int correctCount = 0;
	leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);

	// Convert inches to degrees
	target = inchesToDegrees(target);

	// Make movement relative
	target -= ((leftMotors.get_position() + rightMotors.get_position()) / 2);
	
	double move_error = 0, move_total_error = 0, move_derivative = 0, move_prev_error = 0, effort = 0;
	
	while (correctCount <= 10) {
		
		double position = (leftMotors.get_position() + rightMotors.get_position()) / 2;
		double dist = target - position;

		// proportion
		move_error = dist;

		// integral
		move_total_error += move_error;

		// derivative
		move_derivative = move_error - move_prev_error;
		
		// get prev error for next instance
		move_prev_error = move_error;

		effort = ((MOVE_KP * move_error) / 360);
		effort += (MOVE_KD * move_derivative);
		effort += (MOVE_KI * move_total_error);

		int moveSpeed = effort * 127;
	
		if(abs(dist) > 0.45) {
			int lowerBound = std::max(abs(moveSpeed), 20);
			moveSpeed = std::min(lowerBound, 127);
			leftMotors.move((int)copysign(moveSpeed, -effort));
			rightMotors.move((int)copysign(moveSpeed + 5, -effort));
		} else {
			leftMotors.move(0);
			rightMotors.move(0);
		}
		
		if(abs(dist) <= 0.5) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}

double get_yaw_quaternion() {
	pros::quaternion_s_t qt = imu.get_quaternion();

	//error fetching quat, retry
	if (qt.w == PROS_ERR_F) {
		qt = imu.get_quaternion();
		if (qt.w == PROS_ERR_F) {
			// pros::lcd::set_text(5, "ERROR: IMU Quaternion Fetch Failed");
			return -1.0;
		}
	}

	//convert quat to yaw
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z)))); //yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))

	//returns yaw converted from rad to deg; angle is returned from -180 to 180 (+ 180 for [0, 360])
	return ((yaw * (180 / M_PI)) + 180);
}

double inchesToDegrees (double inches) {
	return (inches / (M_PI * 3.25)) * 360.0;
}