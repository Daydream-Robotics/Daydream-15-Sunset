#include "main.h"
#include "odometry.h"
#include "constants.h"
#include "subsystems.h"

#include "stdlib.h"

#include <cmath>

// returns quaternion 0 to 360
// upon error returns -1
double get_yaw_quaternion() {
    pros::quaternion_s_t qt = imuUpper.get_quaternion();

    // ERROR CHECKING BLOCK
    if (qt.w == PROS_ERR_F) {
        qt = imuUpper.get_quaternion();
        
        if (qt.w == PROS_ERR_F) {
            pros::lcd::set_text(5, "IMU FAILURE: RETURNING -1.0"); 
            return -1.0;
        }
    }

    // Convert quat to yaw (Yaw is in RADIANS, from -pi to pi)
    // Formula: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    double yaw = atan2(2 * (qt.w * qt.z + qt.x * qt.y), 1 - 2 * (qt.y * qt.y + qt.z * qt.z)); 

    // Final conversion: rad to deg, then offset to [0, 360] range for consistency
    return ((yaw * (180.0 / M_PI)) + 180.0);
}

void turn_pid(double target) {
    double heading, tempAngle;
    double optimized_angle;
    double prevTheta= 0; // remove the init if broke
    double theta = 0;

	int correctCount = 0;
	heading = get_yaw_quaternion();
	double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_PID = 0;
    std::optional<double> turn_prev_error;
	
	while (correctCount <= 10) {	
		heading = get_yaw_quaternion() - 180;
		optimized_angle = target - heading;

		if (optimized_angle > 180) optimized_angle -= 360;
		else if (optimized_angle < -180) optimized_angle += 360;
		if (optimized_angle == 180) optimized_angle = 179.99;

		// proportion
		turn_error = optimized_angle;

		// integral
		turn_total_error += turn_error;

		// derivative
        if(turn_prev_error.has_value()){
            turn_derivative = turn_error - turn_prev_error.value();
        }
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = (TURN_KP * turn_error);
		turn_PID += (TURN_KD * turn_derivative);
		if (abs(turn_total_error) < 2000) {
			turn_PID += (TURN_KI * turn_total_error);
		}

		int turnSpeed = turn_PID * 65;
	
		if(abs(optimized_angle) > 0.2) {
			turnSpeed = std::clamp(std::abs(turnSpeed), 2, 65);

			leftMotors.move_velocity((int)copysign(turnSpeed, turn_PID));
			rightMotors.move_velocity(-(int)copysign(turnSpeed, turn_PID));

		}else{
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}