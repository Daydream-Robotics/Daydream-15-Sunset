/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int LEFT_TRACKING_WHEEL_PORT = -11;
constexpr int RIGHT_TRACKING_WHEEL_PORT =19;
constexpr int BACK_TRACKING_WHEEL_PORT = 1;//no back tracking port change later

#define LEFT_DRIVE_WHEEL_PORTS {LEFT_TRACKING_WHEEL_PORT, -12, -16}
#define RIGHT_DRIVE_WHEEL_PORTS {RIGHT_TRACKING_WHEEL_PORT, 20, 17}

constexpr int FRONT_INTAKE_PORT = 21;
constexpr int MAIN_INTAKE_PORT = 3;

constexpr int IMU_PORT = 0; // CHANGE LATER

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 1.0;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.0;

constexpr double MOVE_KP = 1.0;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.0;

#endif