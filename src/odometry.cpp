#include "main.h"
#include "odometry.h"
#include "constants.h"
#include "subsystems.h"

#include "stdlib.h"

#include <cmath>

double heading, optimized_angle, temp_angle, start_angle, mid_angle;

void slew_turn_pid(double target) {
    int correctCount = 0;
        
    // Calculate and normalize the heading/error BEFORE the loop starts
    heading = get_yaw_quaternion(); 
    
    // Your original heading adjustment logic: [0, 360] -> [-180, 180]
    heading = heading - 180; 
    double optimizedAngleInitial = target - heading;
    
    if (optimizedAngleInitial > 180) optimizedAngleInitial -= 360;
    else if (optimizedAngleInitial < -180) optimizedAngleInitial += 360;
    if (optimizedAngleInitial == 180) optimizedAngleInitial = 179.99;
    
    // Initialize turn_prev_error with the first calculated error
    double turn_prev_error = optimizedAngleInitial; 
    
    // --- SLEW RATE VARIABLES (camelCase) ---
    // This holds the motor power that was sent in the PREVIOUS loop iteration
    double currentSlewPower = 0.0; 
    // This defines the MAX change allowed per loop (e.g., max change of 5 power units every 10ms)
    const double SLEW_STEP = 1.0; 
    // --- END SLEW VARIABLES ---
    
    // The rest of the PID variables
    double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_PID = 0;
    
    while (correctCount <= 5) {
        
        // --- ANGLE CALCULATION ---
        heading = get_yaw_quaternion() - 180;
        optimized_angle = target - heading; // optimized_angle is a global variable
            
        pros::lcd::print(2, "Heading = %lf, Target = %lf", heading, target);

        if (optimized_angle > 180) optimized_angle -= 360;
        else if (optimized_angle < -180) optimized_angle += 360;
        if (optimized_angle == 180) optimized_angle = 179.99;

        // --- PID CALCULATION ---
        turn_error = optimized_angle; // proportion
        
        // integral (with windup guard)
        turn_total_error += turn_error;

        // derivative
        turn_derivative = turn_error - turn_prev_error;
        turn_prev_error = turn_error; // get prev error for next instance

        // PID output calculation
        turn_PID = ((TURN_KP * turn_error) / 360);
        turn_PID += (TURN_KD * turn_derivative);
        if (abs(turn_total_error) < 2000) {
            turn_PID += (TURN_KI * turn_total_error);
        }

        // --- MOTOR POWER CALCULATION & SLEW RATE APPLICATION ---
        
        // This is the raw motor speed requested by the PID loop (Target Speed)
        double requestedPower = turn_PID * 50.0;
        
        // Calculate the difference between requested speed and current limited speed
        double powerDifference = requestedPower - currentSlewPower;

        // Limit the acceleration (positive slew)
        if (powerDifference > SLEW_STEP) {
            currentSlewPower += SLEW_STEP;
        } 
        // Limit the deceleration (negative slew)
        else if (powerDifference < -SLEW_STEP) {
            currentSlewPower -= SLEW_STEP;
        } 
        // If difference is within the step size, update to target power
        else {
            currentSlewPower = requestedPower;
        }

        // Final speed to send to motors after slewing
        int finalSpeed = (int)currentSlewPower;
    
        // --- MOTOR COMMANDS ---

        if(abs(optimized_angle) > 0.2) {
			finalSpeed = std::clamp(std::abs(finalSpeed), 1, 55);

			leftMotors.move_velocity((int)copysign(finalSpeed, turn_PID));
			rightMotors.move_velocity(-(int)copysign(finalSpeed, turn_PID));

		} else {
            // Stop and count if within the target tolerance
            leftMotors.move_velocity(0);
            rightMotors.move_velocity(0);
            correctCount++;
        }

        pros::delay(10);
    }

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(250);
}

void test_move_pid(double travelDistance) { // Renamed parameter for clarity in my mind
    int correctCount = 0;
        
    // --- INITIALIZATION ---
    
    // 1. Get current absolute position (total distance traveled since autonomous started)
    double current_position = get_total_distance_traveled();
    
    // 2. Set the NEW absolute target position: Current Position + Desired Travel Distance
    double absolute_target = current_position + travelDistance;
    
    pros::lcd::print(4, "Start Pos = %lf in", current_position);
    
    // Set initial error to the desired travel distance
    double move_prev_error = travelDistance; 

    // Record initial yaw for straightening
    double target_yaw = get_yaw_quaternion();
    
    // Slew variables
    double currentSlewPower = 0.0; 
    const double SLEW_STEP = 1.0; 
    
    // PID variables
    double move_error = 0, move_total_error = 0, move_derivative = 0, move_PID = 0;
    
    while (correctCount <= 5) {
        
        // --- DISTANCE PID CALCULATION ---
        // Get the robot's current absolute position
        double current_distance = get_total_distance_traveled();
        pros::lcd::print(5, "current_distance = %lf in", current_distance);
        
        // Error is the distance remaining to the absolute target
        move_error = absolute_target - current_distance; 

        // integral
        move_total_error += move_error;

        // derivative
        move_derivative = move_error - move_prev_error;
        move_prev_error = move_error; 

        // PID output calculation
        move_PID = (MOVE_KP * move_error);
        move_PID += (MOVE_KD * move_derivative);
        if (abs(move_total_error) < 2000) { // windup guard
            move_PID += (MOVE_KI * move_total_error);
        }

        // --- SLEW RATE APPLICATION ---
        double requestedPower = move_PID * 50.0; // Raw speed from PID
        double powerDifference = requestedPower - currentSlewPower;

        // Limit acceleration/deceleration
        if (powerDifference > SLEW_STEP) currentSlewPower += SLEW_STEP;
        else if (powerDifference < -SLEW_STEP) currentSlewPower -= SLEW_STEP;
        else currentSlewPower = requestedPower;

        int finalSpeed = (int)currentSlewPower;
    
        // --- STRAIGHTENING CORRECTION (P-Controller) ---
        double current_yaw = get_yaw_quaternion();
        double yaw_error = target_yaw - current_yaw;
        
        // Normalize yaw error to [-180, 180]
        if (yaw_error > 180) yaw_error -= 360;
        else if (yaw_error < -180) yaw_error += 360;
        
        // Calculate the turn correction
        double turn_correction = yaw_error * STRAIGHTENING_KP;

        // --- MOTOR COMMANDS ---
        if(abs(move_error) > 0.25) { // Distance tolerance (e.g., 0.5 inches)
            
            finalSpeed = std::clamp(finalSpeed, -50, 50); 

            // Apply forward speed + turn correction
            leftMotors.move_velocity(finalSpeed + turn_correction);
            rightMotors.move_velocity(finalSpeed - turn_correction);

        } else {
            // Stop and count if within the target tolerance
            leftMotors.move_velocity(0);
            rightMotors.move_velocity(0);
            correctCount++;
        }

        pros::delay(10); // Loop delay
    }

    // --- CLEANUP ---
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(250);
}

void move(double speed, double seconds) {
	leftMotors.move_velocity(speed);
	rightMotors.move_velocity(speed);

	pros::delay(seconds * 1000);

	leftMotors.move_velocity(STOP);
	rightMotors.move_velocity(STOP);
}

double get_yaw_quaternion() {
    pros::quaternion_s_t qtUpper = imuUpper.get_quaternion();
    // pros::quaternion_s_t qtLower = imuLower.get_quaternion();

    // ERROR CHECKING BLOCK (Check for immediate failure)
    if (qtUpper.w == PROS_ERR_F /*|| qtLower.w == PROS_ERR_F*/) {
        // Attempt a retry
        qtUpper = imuUpper.get_quaternion();
        // qtLower = imuLower.get_quaternion();
        
        if (qtUpper.w == PROS_ERR_F /*|| qtLower.w == PROS_ERR_F*/) {
            // PROS_ERR_F is typically -1.0. This prints to the V5 screen.
            pros::lcd::set_text(5, "IMU FAILURE: RETURNING -1.0"); 
            return -1.0; 
        }
    }

    // Convert quat to yaw (Yaw is in RADIANS, from -pi to pi)
    // Formula: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    double yawUpper = atan2(2 * (qtUpper.w * qtUpper.z + qtUpper.x * qtUpper.y), 1 - 2 * (qtUpper.y * qtUpper.y + qtUpper.z * qtUpper.z)); 
    // double yawLower = atan2(2 * (qtLower.w * qtLower.z + qtLower.x * qtLower.y), 1 - 2 * (qtLower.y * qtLower.y + qtLower.z * qtLower.z));

    // pros::lcd::print(1, "yawUpper: %lf, yawLower: %lf", yawUpper, yawLower); // Debug print here

    // // --- Dual IMU Averaging and Wraparound Fix (in RADIANS) ---
    // double diff = yawUpper - yawLower;
    // double yawAvg;
    
    // // If the difference is greater than PI (180 degrees), we have crossed the -pi/pi boundary.
    // if (std::abs(diff) > M_PI) { 
    //     if (diff > 0.0) {
    //         // Upper is high (+3.0), Lower is low (-3.0). Add 2*PI to Lower to align it.
    //         yawLower += 2 * M_PI;
    //     } else {
    //         // Upper is low (-3.0), Lower is high (+3.0). Add 2*PI to Upper to align it.
    //         yawUpper += 2 * M_PI;
    //     }
    // }
    
    // // Now that they are aligned in the same 2*PI rotation, average them.
    // yawAvg = (yawUpper + yawLower) / 2.0;

    // // Normalize the average back to the -pi to pi range if necessary
    // while (yawAvg > M_PI) yawAvg -= 2 * M_PI;
    // while (yawAvg <= -M_PI) yawAvg += 2 * M_PI;


    // Final conversion: rad to deg, then offset to [0, 360] range for consistency
    return ((/*yawAvg*/ yawUpper * (180.0 / M_PI)) + 180.0);
}

double get_total_distance_traveled() {
    // Distance per degree: (PI * Diameter) / 360
    const double CONVERSION_FACTOR = WHEEL_DIAMETER * M_PI / 360.0; 

    // Assuming leftTracker and rightTracker have been reset at the start of auto
    double left_degrees = leftTracker.get_position(); 
    double right_degrees = rightTracker.get_position();
    
    // Calculates the total distance (in inches) from 0, based on raw encoder counts
    double total_distance = (left_degrees + right_degrees) / 2.0 * CONVERSION_FACTOR;
    
    return total_distance;
}
