#ifndef AUTO_FUNCTIONS_H
#define AUTO_FUNCTIONS_H

#include "main.h"

void move(int leftVelocity, int rightVelocity, int turnVelocity=0);
void move(int speed);

void move_time(int speed, double seconds);
void move_time(int leftVelocity, int rightVelocity, double seconds);
void move_time_s(int speed, double seconds, double ramp_duration, int ramp_strength);

void rampUp_s(int speed, double ramp_duration, int ramp_strength=1);
void rampUp_s_time(int speed, double seconds, double ramp_duration, int ramp_strength=1);

void rampDown_s(int speed, double ramp_duration, int ramp_strength);

void turn(int speed, int direction, double time);
void forward_backward_move(int velocity,double seconds);

#endif
