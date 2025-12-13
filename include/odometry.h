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

// Turn to specific target angle with slew limiter
void slew_turn_pid(double target);

void slew_move_pid(double targetDistance);

void test_move_pid(double targetDistance);

void move(double speed, double seconds);

double get_yaw_quaternion();

double get_average_encoder_distance(double initial_degrees);

double get_total_distance_traveled();

#endif