#include "main.h"
#include "constants.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS);
pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS);

pros::Motor frontIntake(FRONT_INTAKE_PORT);
pros::Motor mainUpperIntake(MAIN_UPPER_INTAKE_PORT);
pros::Motor mainLowerIntake(MAIN_LOWER_INTAKE_PORT);
pros::Motor backIntake(BACK_INTAKE_PORT);

pros::IMU imuUpper(IMU_PORT_ONE);
pros::IMU imuLower(IMU_PORT_TWO);

pros::adi::Pneumatics unloader(UNLOADER_PORT, false);
pros::adi::Pneumatics centerScore(CENTER_SCORE_PORT, false);

