// #include "main.h"
// #include "odometry.h"
// #include "constants.h"
// #include "subsystems.h"

// #include "stdlib.h"

// #include <cmath>

// // returns quaternion 0 to 360
// // upon error returns -1
// double get_yaw_quaternion() {
//     pros::quaternion_s_t qt = imu.get_quaternion();

//     // ERROR CHECKING BLOCK
//     if (qt.w == PROS_ERR_F) {
//         qt = imu.get_quaternion();
        
//         if (qt.w == PROS_ERR_F) {
//             pros::lcd::print(5, "IMU FAILURE: RETURNING -1.0"); 
//             return -1.0;
//         }
//     }

//     // Convert quat to yaw (Yaw is in RADIANS, from -pi to pi)
//     // Formula: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
//     double yaw = atan2(2 * (qt.w * qt.z + qt.x * qt.y), 1 - 2 * (qt.y * qt.y + qt.z * qt.z)); 

//     // Final conversion: rad to deg, then offset to [0, 360] range for consistency
//     return ((yaw * (180.0 / M_PI)) + 180.0);
// }

// void turn_pid(double target) {
//     double heading, tempAngle;
//     // double turn_error;
//     double prevTheta= 0; // remove the init if broke
//     double theta = 0;

// 	int correctCount = 0;
// 	heading = get_yaw_quaternion();
// 	double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_PID = 0;
//     std::optional<double> turn_prev_error;
	
// 	while (correctCount <= 5) {	
// 		heading = get_yaw_quaternion();
// 		if (heading < 0) {
// 			pros::lcd::print(1, "IMU Failure Quitting Turn %lf", heading);
// 			break;
// 		}
// 		heading -= 180;

// 		turn_error = target - heading;

// 		if (turn_error > 180) turn_error -= 360;
// 		else if (turn_error < -180) turn_error += 360;
// 		if (turn_error == 180) turn_error = 179.99;

// 		// integral
// 		turn_total_error += turn_error;

// 		// derivative
//         if(turn_prev_error.has_value()){
//             turn_derivative = turn_error - turn_prev_error.value();
//         }
		
// 		// get prev error for next instance
// 		turn_prev_error = turn_error;

// 		turn_PID = (TURN_KP * turn_error);
// 		turn_PID += (TURN_KD * turn_derivative);
// 		turn_PID += (TURN_KI * turn_total_error);
// 		controller.print(0, 0, "Turn PID: %lf", turn_PID);
// 		// controller.print(0, 0, "P: %lf", TURN_KP * turn_error);
// 		// controller.print(0, 0, "I: %lf", TURN_KI * turn_total_error);
// 		// controller.print(0, 0, "D: %lf", TURN_KD * turn_derivative);

// 		int turnSpeed = std::clamp(std::abs((int)turn_PID), 1, 65);
// 		leftMotors.move_velocity((int)copysign(turnSpeed, turn_PID));
// 		rightMotors.move_velocity(-(int)copysign(turnSpeed, turn_PID));

// 		if(abs(turn_error) < 0.15){
// 			correctCount++;
// 		}else{
// 			correctCount = 0;
// 		}

// 		pros::delay(10);
// 	}

// 	leftMotors.move(0);
// 	rightMotors.move(0);

// 	pros::delay(100);
	
// }