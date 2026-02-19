#include "intake.h"
#include "main.h"
#include "constants.h"
#include "subsystems.h"

void unloadLongGoal() {
    unloader.set_value(true);
    lowIntake.move_velocity(-200);
    midIntake.move_velocity(-200);
    highIntake.move_velocity(-200);
    pros::delay(500);
    lowIntake.move_velocity(200);
    midIntake.move_velocity(200);
    highIntake.move_velocity(200);
    pros::delay(2000);
    lowIntake.move_velocity(0);
    midIntake.move_velocity(0);
    highIntake.move_velocity(0);
}
