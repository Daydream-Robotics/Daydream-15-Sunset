#include "main.h"
#include "constants.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Rotation parallelTrackingWheel(PARALLEL_TRACKING_WHEEL_PORT);
pros::Rotation perpendicularTrackingWheel(PERPENDICULAR_TRACKING_WHEEL_PORT);

pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS);
pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS);
pros::MotorGroup allMotors(ALL_DRIVE_WHEEL_PORTS);

pros::Motor lowIntake(LOW_INTAKE_PORT);
pros::Motor midIntake(MID_INTAKE_PORT); 
pros::Motor highIntake(HIGH_INTAKE_PORT);

pros::IMU imu(IMU_PORT_ONE);
// pros::IMU imuLower(IMU_PORT_TWO);

pros::adi::Pneumatics unloader(UNLOADER_PORT, false);
pros::adi::Pneumatics centerScore(CENTER_SCORE_PORT, false);
pros::adi::Pneumatics descorer(DESCORE_PORT, false);