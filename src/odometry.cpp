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

// void slew_turn_pid(double target) {
//     double optimizedAngle;
//     int correctCount = 0;

// 	/* - - - - - - - - - - - - - - [INITIALIZATION] - - - - - - - - - - - - - - */
        
//     // Calculate and normalize the heading/error BEFORE the loop starts
//     heading = get_yaw_quaternion(); 
    
//     // Heading adjustment logic: [0, 360] -> [-180, 180]
//     heading = heading - 180; 
//     double optimizedAngleInitial = target - heading;
    
//     if (optimizedAngleInitial > 180) optimizedAngleInitial -= 360;
//     else if (optimizedAngleInitial < -180) optimizedAngleInitial += 360;
//     if (optimizedAngleInitial == 180) optimizedAngleInitial = 179.99;
    
//     // Initialize turnPrevError with the first calculated error
//     double turnPrevError = optimizedAngleInitial; 
    
//     // Other PID variables
//     double turnError = 0, turnTotalError = 0, turnDerivative = 0, turnPID = 0;

// 	// This holds the motor power that was sent in the PREVIOUS loop iteration
//     double currentSlewPower = 0.0; 
    
//     while (correctCount <= 5) {
        
// 		/* - - - - - - - - - - - - - - [ANGLE CALCULATION] - - - - - - - - - - - - - - */

//         heading = get_yaw_quaternion() - 180;
//         optimizedAngle = target - heading; // optimizedAngle is a global variable
            
//         // pros::lcd::print(2, "Heading = %lf, Target = %lf", heading, target);

//         if (optimizedAngle > 180) optimizedAngle -= 360;
//         else if (optimizedAngle < -180) optimizedAngle += 360;
//         if (optimizedAngle == 180) optimizedAngle = 179.99;

// 		/* - - - - - - - - - - - - - - [PID CALCULATION] - - - - - - - - - - - - - - */

//         turnError = optimizedAngle; // proportion
        
//         // integral
//         turnTotalError += turnError;

//         // derivative
//         turnDerivative = turnError - turnPrevError;
//         turnPrevError = turnError;

//         // PID output calculation
//         turnPID = ((TURN_KP * turnError) / 360);
//         turnPID += (TURN_KD * turnDerivative);
//         if (abs(turnTotalError) < 2000) {
//             turnPID += (TURN_KI * turnTotalError);
//         }

// 		/* - - - - - - - - - - - - - - [MOTOR POWER CALCULATION & SLEW RATE APPLICATION] - - - - - - - - - - - - - - */
        
//         // Raw motor speed requested by the PID loop (Target Speed)
//         double requestedPower = turnPID * 50.0;
        
//         // Calculate the difference between requested speed and current limited speed
//         double powerDifference = requestedPower - currentSlewPower;

//         // Limit the acceleration (positive slew)
//         if (powerDifference > SLEW_STEP) {
//             currentSlewPower += SLEW_STEP;
//         } 
//         // Limit the deceleration (negative slew)
//         else if (powerDifference < -SLEW_STEP) {
//             currentSlewPower -= SLEW_STEP;
//         } 
//         // If difference is within the step size, update to target power
//         else {
//             currentSlewPower = requestedPower;
//         }

//         // Final speed to send to motors after slewing
//         int finalSpeed = (int)currentSlewPower;
    
// 		/* - - - - - - - - - - - - - - [MOTOR COMMANDS] - - - - - - - - - - - - - - */

//         if(abs(optimizedAngle) > 0.2) {
// 			finalSpeed = std::clamp(std::abs(finalSpeed), 1, 55);

// 			leftMotors.move_velocity((int)copysign(finalSpeed, turnPID));
// 			rightMotors.move_velocity(-(int)copysign(finalSpeed, turnPID));

// 		} else {
//             // Stop and count if within the target tolerance
//             leftMotors.move_velocity(0);
//             rightMotors.move_velocity(0);
//             correctCount++;
//         }

//         pros::delay(10);
//     }

// 	/* - - - - - - - - - - - - - - [CLEANUP] - - - - - - - - - - - - - - */

//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);
//     pros::delay(250);
// }

// void slew_move_pid(double travelDistance) {
//     int correctCount = 0;
        
//     /* - - - - - - - - - - - - - - [INITIALIZATION] - - - - - - - - - - - - - - */

//     // Get current absolute position (total distance traveled since autonomous started)
//     double currentPosition = get_total_distance_traveled();
    
//     // Set the NEW absolute target position: Current Position + Desired Travel Distance
//     double absoluteTarget = currentPosition + travelDistance;
    
//     pros::lcd::print(4, "Start Pos = %lf in", currentPosition);
    
//     // Set initial error to the desired travel distance
//     double movePrevError = travelDistance; 

//     // Record initial yaw for straightening
//     double targetYaw = get_yaw_quaternion();
    
//     // PID variables
//     double moveError = 0, moveTotalError = 0, moveDerivative = 0, movePID = 0;

// 	// Slew variables
//     double currentSlewPower = 0.0; 
    
//     while (correctCount <= 5) {

// 		/* - - - - - - - - - - - - - - [DISTANCE PID CALCULATION] - - - - - - - - - - - - - - */

//         // Get current absolute position
//         double current_distance = get_total_distance_traveled();
//         pros::lcd::print(5, "current_distance = %lf in", current_distance);
        
//         // Error is the distance remaining to the absolute target
//         moveError = absoluteTarget - current_distance; 

//         // integral
//         moveTotalError += moveError;

//         // derivative
//         moveDerivative = moveError - movePrevError;
//         movePrevError = moveError; 

//         // PID output calculation
//         movePID = (MOVE_KP * moveError);
//         movePID += (MOVE_KD * moveDerivative);
//         if (abs(moveTotalError) < 2000) { // windup guard
//             movePID += (MOVE_KI * moveTotalError);
//         }

// 		/* - - - - - - - - - - - - - - [SLEW RATE APPLICATION] - - - - - - - - - - - - - - */

//         double requestedPower = movePID * 50.0; // Raw speed from PID
//         double powerDifference = requestedPower - currentSlewPower;

//         // Limit acceleration/deceleration
//         if (powerDifference > SLEW_STEP) currentSlewPower += SLEW_STEP;
//         else if (powerDifference < -SLEW_STEP) currentSlewPower -= SLEW_STEP;
//         else currentSlewPower = requestedPower;

//         int finalSpeed = (int)currentSlewPower;
    
// 		/* - - - - - - - - - - - - - - [STRAIGHTENING CORRECTION] - - - - - - - - - - - - - - */

//         double currentYaw = get_yaw_quaternion();
//         double yawError = targetYaw - currentYaw;
        
//         // Normalize yaw error to [-180, 180]
//         if (yawError > 180) yawError -= 360;
//         else if (yawError < -180) yawError += 360;
        
//         // Calculate the turn correction
//         double turnCorrection = yawError * STRAIGHTENING_KP;

// 		/* - - - - - - - - - - - - - - [MOTOR COMMANDS] - - - - - - - - - - - - - - */

//         if(abs(moveError) > 0.25) { // Distance tolerance (e.g., 0.5 inches)
            
//             finalSpeed = std::clamp(finalSpeed, -50, 50); 

//             // Apply forward speed + turn correction
//             leftMotors.move_velocity(finalSpeed + turnCorrection);
//             rightMotors.move_velocity(finalSpeed - turnCorrection);

//         } else {
//             // Stop and count if within the target tolerance
//             leftMotors.move_velocity(0);
//             rightMotors.move_velocity(0);
//             correctCount++;
//         }

//         pros::delay(10); // Loop delay
//     }

// 	/* - - - - - - - - - - - - - - [CLEANUP] - - - - - - - - - - - - - - */

//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);
//     pros::delay(250);
// }

// double get_total_distance_traveled() {
//     // Assuming leftTracker and rightTracker have been reset at the start of auto
//     double leftDegrees = leftTracker.get_position(); 
//     double rightDegrees = rightTracker.get_position();
    
//     // Calculates the total distance (in inches) from 0, based on raw encoder counts
//     double totalDistance = (leftDegrees + rightDegrees) / 2.0 * CONVERSION_FACTOR;
    
//     return totalDistance;
// }
