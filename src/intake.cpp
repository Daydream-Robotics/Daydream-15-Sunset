#include "intake.h"
#include "main.h"
#include "constants.h"
#include "subsystems.h"
#include "autonomous.hpp"

void unloadLongGoal(Autonomous auton) {
    double heading = auton.getYaw();
    
    // dislodge
    unloader.set_value(true);
    lowIntake.move_velocity(-70);
    midIntake.move_velocity(300); // -100
    highIntake.move_velocity(300); // -200
    pros::delay(600);

    // h
    lowIntake.move_velocity(600);
    midIntake.move_velocity(600);
    highIntake.move_velocity(600);

    // grab the ball at intake
    auton.travel(12, 50, heading, 0.3);
    auton.travel(-12, 50, heading, 0.5);

    // wait
    // pros::delay(900);

    for (int i = 0; i < 10; i++) {
        checkOuttakeStall();
        pros::delay(100);
    }

    // stop
    lowIntake.move_velocity(0);
    midIntake.move_velocity(0);
    highIntake.move_velocity(0);
}

// the "hump" function
void hump(Autonomous auton) {
    move_intake(200, 200, -100);
    for (int i = 0; i < 2; i++){
        // auton.travel(24, 150, 90, 0.4);
        leftMotors.move_velocity(-30);
        rightMotors.move_velocity(-30);
        pros::delay(250);  // lo 200
        leftMotors.move_velocity(33);
        rightMotors.move_velocity(33);
        pros::delay(400);
        leftMotors.move_velocity(-5);
        rightMotors.move_velocity(-5);
        pros::delay(700);
	}
    // pros::delay(200);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    move_intake(0, 0, 0);
}

void checkOuttakeStall() {
    if (lowIntake.get_actual_velocity() == 0 || midIntake.get_actual_velocity() == 0 || highIntake.get_actual_velocity() == 0) {
        lowIntake.move_velocity(-300);
        midIntake.move_velocity(-300);
        highIntake.move_velocity(-300);
        pros::delay(100);
        lowIntake.move_velocity(300);
        midIntake.move_velocity(300);
        highIntake.move_velocity(300);
    }
}
