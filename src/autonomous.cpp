#include "autonomous.hpp"
#include "subsystems.h"
#include "constants.h"

// ====== Helper functions ======
namespace {
	// Normalize angle between [-180, 180]
	double normalizeAngle(double a) {
		return std::atan2(std::sin(a), std::cos(a));
	}
	
	// Return Euclidean distance btwn points p1 and p2
	double getDistance(Position p1, Position p2) {
		return std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
	}
	
	// Convert Degrees to Radians
	double convertDegToRad(double degree) {
		return degree * (std::numbers::pi / 180.0);
	}
	
	// Covert Radians to Degrees
	double convertRadToDeg(double rad) {
		return rad * (180.0 / M_PI);
	}
	
	// Subtract angles with [-180, 180] wrapping
	double angleDiffDeg(double a, double b) {
		double c = a - b;
		while (c > 180.0) c -= 360.0;
		while (c <= -180.0) c += 360.0;
		return c;
	}
	
	// Determine deceleration speed scaling
	double computeDecelScale(double remaining, double totalDistance) {
		double decelDistance = std::max(0.2, std::fabs(totalDistance) * 0.15);
	
		if (remaining >= decelDistance)
			return 1.0;
	
		double x = std::clamp(remaining / decelDistance, 0.0, 1.0);
	
		// Step smoothing
		double smooth = x * x * (3.0 - 2.0 * x);
	
		// Return speed scaling
		return std::clamp(smooth, 0.0, 1.0);
	}
	
	// Limit acceleration takeoff
	double accelLimit(double prev, double target, double dt, double accelLimit) {
		double maxDelta = accelLimit * dt;
		double delta = target - prev;
	
		if (delta > maxDelta) delta = maxDelta;
		if (delta < -maxDelta) delta = -maxDelta;
	
		return prev + delta;
	}
	
	// Utility class for smoothing heading changes
	class HeadingFilter {
		private:
			double alpha;
			double heading;
			bool initialized = false;
	
		public:
			HeadingFilter(double alpha) : alpha(alpha) {}
	
			double update(double raw) {
				if (!initialized) {
					heading = raw;
					initialized = true;
					return heading;
				}
	
				heading += alpha * angleDiffDeg(raw, heading);
				return heading;
			}
	
			void reset() {
				initialized = false;
			}
	};
}


// TODO: Tune PID parameters
Autonomous::Autonomous() 
	: distancePID(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD, DISTANCE_KI_THRESHOLD),  // 5.0, 2.0, 0.0, 1.0
	headingPID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_KI_THRESHOLD), // 0.002, 0.0, 0.0, 0.0
	turnPID(TURN_KP, TURN_KI, TURN_KD, TURN_KI_THRESHOLD) { // 1.22, 0.000, 0.063875, 180
		leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	}

void Autonomous::turnTo(double targetHeading) {
	turnPID.reset();
    turnPID.setTarget(targetHeading);

	// pros::lcd::print(0, "Turning to %lf degrees", targetHeading);

	// TODO: Tune exit conditions
    turnPID.exit_condition_set(
        0.3, 75,     // small error (deg), time (ms)
        0.9, 50,     // big error (deg), time
       1 ,100,          // velocity settle time
        0          // timeout
    );

	// TODO: Tune alpha
    HeadingFilter headingFilter(0.3);

		// Initialize clocking
	using clock = std::chrono::steady_clock;
    auto lastTime = clock::now();
	double prevHeading = getYaw();

	while (true) {
		// Heading calculation
		double rawHeading = getYaw();
		updatePose();

		if (rawHeading < 0) {
			pros::lcd::print(0, "IMU Failure!");
			// TODO: Add more verbose error handling
			return;
		}

		// Calculate angular velocity
		auto now = clock::now();
		std::chrono::duration<double> dt_dur = now - lastTime;
		double dt = dt_dur.count();
		lastTime = now;
		
		// Determine PID correction using smoothed heading
		double filteredHeading = headingFilter.update(rawHeading - 180);
		double correction = turnPID.compute(filteredHeading,true);
		//pros::lcd::print(1, "Correction: %lf", correction);
		
		// Compute turnSpeed based on correction
		double turnSpeed = std::clamp(
			std::fabs(correction),
            1.0, // 2
            100.0 // 70
        );
		
        turnSpeed = std::copysign(turnSpeed, correction);
		
		leftMotors.move_velocity(turnSpeed);
        rightMotors.move_velocity(-turnSpeed);
		
		double currentVelocity = (turnSpeed < 20) ? angleDiffDeg(rawHeading, prevHeading) / dt : 999.0;
		if (turnPID.exit_condition(currentVelocity) != PID::RUNNING)
			break;
		prevHeading = rawHeading;
		
		pros::delay(10);
	}

	// pros::lcd::print(2, "Exited Turn Loop");

	leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(10);

	updatePose();
}

void Autonomous::setPose(double x, double y) {
    this->pos_x = x;
    this->pos_y = y;
}

double Autonomous::travel(double distance, double speed, double targetHeading, double timer_s) {
	double traveled = 0;

    auto clamp = [](double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    };

    auto normalizeDeg = [](double a) {
        while (a >= 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    };
	// int count = 0;
    distancePID.setTarget(distance);
    distancePID.exit_condition_set(
        0.1, 10,
        0.4, 30,
        1, 50,
        timer_s * 1000
    );

    headingPID.setTarget(0.0);

    Position start(pos_x, pos_y);
    double direction = (distance >= 0.0) ? 1.0 : -1.0;

	double headingRad = convertDegToRad(targetHeading);
	Position headingUnit {
		std::cos(headingRad),
		std::sin(headingRad)
	};

    double prevVelocity = 0.0;
    double prevDistance = 0.0;

	using clock = std::chrono::steady_clock;
	auto lastTime = clock::now();

    while (true) {
		// get elapesed time since last loop
		auto now = clock::now();
		std::chrono::duration<double> dt_dur = now - lastTime;
		double dt = dt_dur.count();
		lastTime = now;

        updatePose();
		
		if (pos_x < -1) {
			pros::lcd::print(7, "OUT OF BOUNDS!");
			break;
		}

		// controller.print(0,0, "%.2f, %.2f", pos_x, pos_y);
        // Compute traveled distance along heading vector
        Position delta { pos_x - start.x, pos_y - start.y };
        // double traveled = delta.y * headingUnit.x + delta.x * headingUnit.y;
		traveled = delta.x * headingUnit.x + delta.y * headingUnit.y;

		// pros::lcd::print(1, "Delta: x: %.2f, y: %.2f", delta.x, delta.y);
		// pros::lcd::print(2, "Heading Unit: x: %.2f, y: %.2f", headingUnit.x, headingUnit.y);
		// pros::lcd::print(3, "Traveled: %.2f", traveled);


        double v = distancePID.compute(traveled);
        v = clamp(v, -speed, speed);

        // Slew rate limiter to prevent slipping
        v = accelLimit(prevVelocity, v, dt, accelLimitRate);
        prevVelocity = v;

        // Heading error
        double rawHeading = getYaw();
        if (rawHeading < 0) {
            pros::lcd::print(0, "IMU Failure!");
            break;
        }

        double headingError = normalizeDeg(targetHeading - rawHeading);

        // Heading correction
        double omega = headingPID.compute(headingError) * (std::fabs(v) / speed);

        // Differential drive
		
        double left  = v + omega;
        double right = v - omega;

        // Magnitude Scaling
        double maxMag = std::max(std::fabs(left), std::fabs(right));
        if (maxMag > speed) {
            double scale = speed / maxMag;
            left  *= scale;
            right *= scale;
        }

        leftMotors.move_velocity(left);
        rightMotors.move_velocity(right);

		// Exit if any exit condition is met. 100 set to prevent velocity timeout for now
		double currVel = (fabs(v) < 10) ? (traveled - prevDistance) / dt : 999.0;
        if (distancePID.exit_condition(currVel) != PID::RUNNING){
			pros::lcd::print(0,0,"Exit Condition Meet");
            break;
		}
        prevDistance = traveled;
		//   controller.print(0,0, "%.2f", rawHeading);
		// count++;
        pros::delay(10);
    }

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(10);
	return traveled;
}


// TODO: TEST
void Autonomous::moveToPoint(double targetX, double targetY, double speed, bool reverse, double timer_s) {
    updatePose();
    double dx = targetX - pos_x;
    double dy = targetY - pos_y;
    double targetHeading = std::atan2(dy, dx) * 180.0 / M_PI;
    
    if (reverse) {
        targetHeading += 180;
        while (targetHeading > 180) targetHeading -= 360;
    }
    
    // Initial turn to face the target
    turnTo(targetHeading);
    
    // Setup PIDs
    distancePID.reset();
    distancePID.setTarget(0);
    // Use similar exit conditions to travel
    distancePID.exit_condition_set(0.5, 50, 2.0, 100, 2, 100, timer_s*1000);
    
    headingPID.reset();
    
    double prevVelocity = 0;
    double prevDistance = 0;
    
    using clock = std::chrono::steady_clock;
    auto lastTime = clock::now();
    
    while (true) {
        auto now = clock::now();
        std::chrono::duration<double> dt_dur = now - lastTime;
        double dt = dt_dur.count();
        lastTime = now;
        
        updatePose();
        
        dx = targetX - pos_x;
        dy = targetY - pos_y;
        double dist = std::hypot(dx, dy);
        
        // Continuously update target heading to point to target
        targetHeading = std::atan2(dy, dx) * 180.0 / M_PI;
        if (reverse) {
            targetHeading += 180;
            while (targetHeading > 180) targetHeading -= 360;
        }
        
        double rawHeading = getYaw();
        
        // Distance PID input: negative distance if moving forward (to approach 0 from negative)
        double distInput = reverse ? dist : -dist;
        double v = distancePID.compute(distInput);
        
        // Clamp and Slew
        if (v > speed) v = speed;
        if (v < -speed) v = -speed;
        v = accelLimit(prevVelocity, v, dt, accelLimitRate);
        prevVelocity = v;
        
        // Heading PID
        headingPID.setTarget(targetHeading);
        // Note: We negate the result because of how travel() vs moveToPoint() calculates error
        double omega = -headingPID.compute(rawHeading, true) * (std::fabs(v) / speed);
        
        double left = v + omega;
        double right = v - omega;
        
        // Scale to max speed
        double maxMag = std::max(std::fabs(left), std::fabs(right));
        if (maxMag > speed) {
            double scale = speed / maxMag;
            left *= scale;
            right *= scale;
        }
        
        leftMotors.move_velocity(left);
        rightMotors.move_velocity(right);
        
        // Exit condition based on velocity of distance error
        double distVel = (std::fabs(v) < 10) ? (distInput - prevDistance) / dt : 999.0;
        if (distancePID.exit_condition(distVel) != PID::RUNNING) break;
        
        prevDistance = distInput;
        pros::delay(10);
    }
    
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}


void Autonomous::updatePose(void) {
	// Calculate distance travelled by each tracking wheel
	WheelLengths arcs = getOdomWheelTravel();

	const double yaw = getYaw();

	if (yaw < 0) {
		pros::lcd::print(0, "IMU Failure!");
		return;
	}

	static double prevTheta = convertDegToRad(yaw - 180);

	// Get orientation from IMU
	double theta = convertDegToRad(yaw - 180);
	theta = normalizeAngle(theta);

	// Determine change in heading 
	double del_theta = normalizeAngle(theta - prevTheta);

	// Determine change in local x and in local y
	double dx_local = arcs.parallel;
	double dy_local = arcs.perpendicular - (del_theta * PERPINDICULAR_TRACKING_WHEEL_DISTANCE);

	double theta_mid = prevTheta + del_theta / 2.0;
    theta_mid = normalizeAngle(theta_mid);

    // Compute change in x and y based on heading and local changes
	double del_x = std::cos(theta_mid) * dx_local - std::sin(theta_mid) * dy_local;
	double del_y = std::sin(theta_mid) * dx_local + std::cos(theta_mid) * dy_local;

	// Increment position and angle by calculated changes
	pos_x += del_x;
	pos_y += del_y;

	prevTheta = theta;
	
	//  controller.print(0, 0, "O: %.2lf\n", pos_x);
}

double Autonomous::getYaw(void) {
    pros::quaternion_s_t qt = imu.get_quaternion();

    // If encounter IMU failure, and retry
    if (qt.w == PROS_ERR_F) {
        qt = imu.get_quaternion();
        if (qt.w == PROS_ERR_F) {
            return -1.0;
        }
    }

    // yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z))));

	// returns yaw converted from rad to deg; angle is returned from -180 to 180 (+ 180 for [0, 360])
	return ((yaw * (180 / M_PI)) + 180);
}

WheelLengths Autonomous::getOdomWheelTravel(void) {
    // Get Initial Wheel Position
    static double lastParallel = parallelTrackingWheel.get_position();
	static double lastPerpendicular = perpendicularTrackingWheel.get_position();


	// pros::delay(100);

    // Get current centidegree position of tracking wheels
	double currParallel = parallelTrackingWheel.get_position();
	double currPerpendicular = perpendicularTrackingWheel.get_position();

	// controller.print(0,0, "%d", currParallel);
    // Get delta between current and last frame
	double dTicksL = currParallel - lastParallel; 
	double dTicksS = currPerpendicular - lastPerpendicular;


	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (dTicksL / 36000.0) * PARALLEL_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double delPerpendicular = (dTicksS / 36000.0) * PERPENDICULAR_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 

    // Save current position as previous
	lastParallel = currParallel;
	lastPerpendicular = currPerpendicular;
	// controller.print(0, 0, "O: %.2f\n", delParallel);
	// Create a structure of lengths
	WheelLengths del(delParallel, delPerpendicular);
	

	return del;
}


// void Autonomous::travelToX(double x_targ, double speed, double target_heading, int timer) {
//     // double distanc;

//     updatePose();
//     Position start(pos_x, pos_y);
    
//     double dx = x_targ - start.x;

// 	if (target_heading == 180) {
// 		dx *= -1;
// 	}
//     // double dy = y_targ - start.y;

//     // distance = std::hypot(dx, dy); //euclidean distance from start to end point

//     // target_heading = std::atan2(dy, dx) * 180.0  / M_PI; // degrees


//     travel(dx, speed, target_heading, timer);
//     return;
// }

bool Autonomous::travelToPoint(double targetX, double targetY, double maxSpeed, bool reverse, int timer) {
	updatePose();
	Position start(pos_x, pos_y);
	double dx = targetX - start.x;
	double dy = targetY - start.y;
	
	double distance = std::hypot(dx, dy); //euclidean distance from start to end point	
	double targetHeading = std::atan2(dy, dx) * 180.0  / M_PI;
	
	if (reverse) {
		targetHeading += 180;
		if (targetHeading > 180) targetHeading -= 360;
		
		distance = -distance;
	}

	turnTo(targetHeading);
	travel(distance, maxSpeed, targetHeading, timer);

	pros::delay(10);

	return true;
}