/*
 * \file odometry.h
 *
 *  Contains function declarations and constants for odometry
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>

/**
 * Turn to a specific angle with slew limiter
 * 
 * @param target Target angle in degrees
 */
void slew_turn_pid(double target);

/**
 * Move a specific distance with slew limiter
 * 
 * @param targetDistance Distance to move in inches
 */
void slew_move_pid(double targetDistance);

/**
 * Move at a specific speed for a certain amount of time
 * 
 * @param speed Speed in RPM
 * @param seconds Duration to move in seconds
 */
void move(double speed, double seconds);

/**
 * Get yaw from IMU in quaternion format
 * 
 * @return Yaw angle in degrees [0, 360)
 */
double get_yaw_quaternion(void);

/**
 * Get total distance traveled by the robot by averaging tracking wheels
 * 
 * @return Total distance traveled in inches
 */
double get_total_distance_traveled(void);

#endif