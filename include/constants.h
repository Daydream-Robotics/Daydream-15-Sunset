/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int LEFT_TRACKING_WHEEL_PORT = -3;
constexpr int RIGHT_TRACKING_WHEEL_PORT = 6;

#define LEFT_DRIVE_WHEEL_PORTS {LEFT_TRACKING_WHEEL_PORT, -4, -5}
#define RIGHT_DRIVE_WHEEL_PORTS {RIGHT_TRACKING_WHEEL_PORT, 7, 8}

constexpr int FRONT_INTAKE_PORT = 17;
constexpr int MAIN_LOWER_INTAKE_PORT = -14;
constexpr int MAIN_UPPER_INTAKE_PORT = 15;
constexpr int BACK_INTAKE_PORT = -16;

constexpr int IMU_PORT_ONE = 13;
constexpr int IMU_PORT_TWO = 8;

constexpr char UNLOADER_PORT = 'A';
constexpr char CENTER_SCORE_PORT = 'B';
constexpr char DESCORE_PORT = 'C';

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 1.0;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.0;

constexpr double MOVE_KP = 1.0;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.0;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

#endif