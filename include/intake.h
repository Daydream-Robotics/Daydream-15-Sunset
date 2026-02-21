#ifndef INTAKE_H
#define INTAKE_H

#include "autonomous.hpp"

void hump(Autonomous auton);
void unloadLongGoal(Autonomous auton);
void checkOuttakeStall();
void travelToPoint(double x_targ, double y_targ, double speed, int timer);

#endif