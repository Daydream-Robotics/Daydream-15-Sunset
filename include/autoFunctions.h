#ifndef AUTO_FUNCTIONS_H
#define AUTO_FUNCTIONS_H

#include "main.h"

void move(int leftVelocity, int rightVelocity);
void move(int speed);
void move_time(int leftVelocity, int rightVelocity, double seconds);


void move_time_pid_heading(int speed, double seconds, double heading);
void move_time_pid(int speed, double seconds);
void move_time_pid_stop(int speed, double seconds);

void move_time_s(int speed, double seconds, double ramp_duration, int ramp_strength);
void rampDown_s(int speed, double ramp_duration, int ramp_strength);

#endif
