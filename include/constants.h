/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int PARALLEL_TRACKING_WHEEL_PORT = 6;
constexpr int PERPENDICULAR_TRACKING_WHEEL_PORT = 21;

#define LEFT_DRIVE_WHEEL_PORTS {-7, -8, -9}
#define RIGHT_DRIVE_WHEEL_PORTS {1, 2, 3}
#define ALL_DRIVE_WHEEL_PORTS {-7,-8,-9,1,2,3}

constexpr int LOW_INTAKE_PORT = -10;
constexpr int MID_INTAKE_PORT = -11;
constexpr int HIGH_INTAKE_PORT = -4;

constexpr int IMU_PORT_ONE = 5;
// constexpr int IMU_PORT_TWO = 8;

constexpr char UNLOADER_PORT = 'F';
constexpr char CENTER_SCORE_PORT = 'H';
constexpr char DESCORE_PORT = 'G';

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 0.027; // 10 (?)
constexpr double TURN_KI = 0.00; // 0.15 (?)
constexpr double TURN_KD = 0.00; // 0.25 (?)



constexpr double MOVE_KP = 1.0;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.5;

constexpr double STRAIGHTENING_KP = 0.0;

constexpr double MOVE_HEADING_KP = 2.25;
constexpr double MOVE_HEADING_KI = 0.0;
constexpr double MOVE_HEADING_KD = 0.0;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

constexpr double PERPINDICULAR_TRACKING_WHEEL_DISTANCE = 1.00;

constexpr double PARALLEL_TRACKING_WHEEL_DIAMETER = 2.00;
constexpr double PERPENDICULAR_TRACKING_WHEEL_DIAMETER = 2.00;



/* - - - - - - - - - - - - - [MISCELLANEOUS] - - - - - - - - - - - - - */

// Defines the MAX change allowed per loop (e.g., max change of 5 power units every 10ms)
const double SLEW_STEP = 1.0; 

enum class DriveType {
    SPLIT_ARCADE,
    TANK
};

#endif