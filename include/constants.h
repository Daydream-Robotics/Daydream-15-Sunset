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

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

#endif