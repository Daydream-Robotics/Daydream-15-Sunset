/*
 * \file odometry.h
 *
 *  Contains function declarations and constants for odometry
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>
#include <chrono>

// Struct for holding lengths travelled by wheels
typedef struct ArcLengths {
    double parallel;
    double perpendicular;
} ArcLengths;

typedef struct Position {
    double x;
    double y;
} Position;

// Calculate and update global position and orientation structs
void update_position_and_angle(void);

// Normalize the angle between pi and -pi
double normalizeAngle(double a);

// Get arc lengths travelled and update encoder values
ArcLengths get_wheel_travel(void);

// Return change in heading based on arc lengths travelled
double compute_heading_change(ArcLengths arcs);

// Convert degrees to radians
double convert_deg_to_rad(double degrees);

// Convert radians to degrees
double convert_rad_to_deg(double rad);

// Find distance between two positions
double get_distance(Position p1, Position p2);

// Turn to a specified heading
void turn_pid(double target);

/// Move to specified position (backward == 1 for reverse) (timer == -1 disables timer)
void move_pid(Position target, int speed, double weightAdjustment, int backwards, int timer);

// Move a specified distance (backwards == -1 for reverse)
void move_dist_pid(double targetDistance, int speed, int timer, int backwards);

// get yaw from imu
double get_yaw_quaternion();

// Move a specified distance and direction
void travel_distance_with_heading(double distance, double speed, double target_heading, int timer);

// Return the shortest differnence between angle A and angle B, normalized (-180,180)
double angleDiffDeg(double a, double b);

#endif