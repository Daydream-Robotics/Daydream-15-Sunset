#include "intake.h"
#include "main.h"
#include "constants.h"
#include "subsystems.h"
#include "autonomous.hpp"

void unloadLongGoal(Autonomous auton) {
    double heading = auton.getYaw();
    
    // dislodge
    unloader.set_value(true);
    lowIntake.move_velocity(-100);
    midIntake.move_velocity(-100);
    highIntake.move_velocity(-200);
    pros::delay(600);

    // 
    lowIntake.move_velocity(300);
    midIntake.move_velocity(300);
    highIntake.move_velocity(300);

    // grab the ball at intake
    auton.travel(12, 50, heading, 0.2);
    auton.travel(-12, 50, heading, 0.4);

    // wait
    pros::delay(2000);

    // stop
    lowIntake.move_velocity(0);
    midIntake.move_velocity(0);
    highIntake.move_velocity(0);
}


void loadFromLoader(Autonomous auton) {
    for (int i = 0; i < 3; i++){
		auton.travel(-12, 50, 90, 0.25);
		auton.travel(12, 60, 90, 0.35);
	 	pros::delay(500);
	}
}
