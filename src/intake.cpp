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
    midIntake.move_velocity(300); // -100
    highIntake.move_velocity(300); // -200
    pros::delay(600);

    // h
    lowIntake.move_velocity(300);
    midIntake.move_velocity(300);
    highIntake.move_velocity(300);

    // grab the ball at intake
    auton.travel(12, 50, heading, 0.3);
    auton.travel(-12, 50, heading, 0.5);

    // wait
    pros::delay(900);

    // stop
    lowIntake.move_velocity(0);
    midIntake.move_velocity(0);
    highIntake.move_velocity(0);
}

// the "hump" function
void hump(Autonomous auton) {
    for (int i = 0; i < 1; i++){
        pros::delay(300);
        auton.travel(-12, 50, 90, 0.4);
        pros::delay(200);
		// auton.travel(24, 150, 90, 0.4);
        leftMotors.move_velocity(200);
        rightMotors.move_velocity(200);
        pros::delay(600);
        leftMotors.move_velocity(-30);
        rightMotors.move_velocity(-30);
        pros::delay(100);
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
	}
    pros::delay(200);
}
