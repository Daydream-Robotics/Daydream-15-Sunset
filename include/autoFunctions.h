#ifndef AUTO_FUNCTIONS_H
#define AUTO_FUNCTIONS_H

#include "main.h"

void move(int leftVelocity, int rightVelocity, double time);
void move(int leftVelocity, int rightVelocity);
void move(int speed);
void move_time_s(int speed, double seconds, int ramp_strength);

#endif // AUTO_FUNCTIONS_H