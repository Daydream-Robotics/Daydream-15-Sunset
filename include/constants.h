/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

// constexpr int LEFT_TRACKING_WHEEL_PORT = -3;
// constexpr int RIGHT_TRACKING_WHEEL_PORT = 6;

// #define LEFT_DRIVE_WHEEL_PORTS {LEFT_TRACKING_WHEEL_PORT, -4, -5}
// #define RIGHT_DRIVE_WHEEL_PORTS {RIGHT_TRACKING_WHEEL_PORT, 7, 8}

#define LEFT_DRIVE_WHEEL_PORTS {-7, -8, -9}
#define RIGHT_DRIVE_WHEEL_PORTS {1, 2, 3}

constexpr int LOW_INTAKE_PORT = -10;
constexpr int MID_INTAKE_PORT = -11;
constexpr int HIGH_INTAKE_PORT = 4;

constexpr int IMU_PORT_ONE = 5;
// constexpr int IMU_PORT_TWO = 8;

constexpr char UNLOADER_PORT = 'F';
constexpr char CENTER_SCORE_PORT = 'H';
constexpr char DESCORE_PORT = 'G';

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 5.0;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.2;

constexpr double MOVE_KP = 1.0;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.5;

constexpr double STRAIGHTENING_KP = 0.0;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

constexpr double WHEEL_DIAMETER = 3.25; // inches

// Distance per degree: (PI * Diameter) / 360
constexpr double CONVERSION_FACTOR = WHEEL_DIAMETER * M_PI / 360.0; // converting degrees to inches

// Defines the MAX change allowed per loop (e.g., max change of 5 power units every 10ms)
const double SLEW_STEP = 1.0; 

enum class DriveType {
    SPLIT_ARCADE,
    TANK
};

#endif