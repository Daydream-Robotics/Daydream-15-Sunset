/*
 * \file odometry.h
 *
 *  Contains function declarations and constants for odometry
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>

// Turn to specified target angle
void turn_pid(double target);

void move_pid(double target);

void move(int seconds);

void move_pid_back(double target);

// Get yaw from imu
double get_yaw_quaternion();

double inchesToDegrees (double inches);

void turn(int seconds);

#endif