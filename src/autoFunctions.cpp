#include "autoFunctions.h"
#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "chrono"

void move(int leftVelocity, int rightVelocity) {
    leftMotors.move_velocity(leftVelocity);
    rightMotors.move_velocity(rightVelocity);
}

void forward_backward_move(int velocity,double seconds){
    allMotors.move_velocity(velocity);
    pros::delay(1000*seconds);
    allMotors.move_velocity(0);
}

void move(int speed){
    move(speed, speed);
}

void move_time(int leftVelocity, int rightVelocity, double seconds) {
    move(leftVelocity, rightVelocity);
    pros::delay(1000*seconds);
    move(0);
}

void move_time(int speed, double seconds) {
    move_time(speed, speed, seconds);
}


void move_time_s(int speed, double seconds, double ramp_duration, int ramp_strength) {
    // pros::lcd::print(1, "[Move_time] Entered");
    // double s_curve_duration_sec = 2;
    if (seconds < ramp_duration) {
        ramp_duration = seconds;
    }

    auto start = std::chrono::steady_clock::now();
    
    // pros::lcd::print(1, "[Move_time] Enter Constant");
    move(speed);    

    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    double s_curve_end_time = seconds - ramp_duration;
    while (elapsed.count() < s_curve_end_time) {
        elapsed = std::chrono::steady_clock::now() - start;
        move(speed);    
        pros::delay(10);
    }
    
    rampDown_s(speed, ramp_duration, ramp_strength);
    
    move(0);
    // pros::lcd::print(1, "[Move_time] Exit Function");
}

void rampDown_s(int speed, double ramp_duration, int ramp_strength) {
    auto ramp_start = std::chrono::steady_clock::now();

    // pros::lcd::print(1, "[ramp_down] Enter Ramp");
    while (true) {
        std::chrono::duration<double> ramp_elapsed = std::chrono::steady_clock::now() - ramp_start;
        // pros::lcd::print(2, "ramp_elapsed = %lf", ramp_elapsed.count());
        // pros::lcd::print(3, "ramp_duration = %lf", ramp_duration);
        
        if (ramp_elapsed.count() > ramp_duration) {
            break;
        }

        double curve = (std::cos((ramp_elapsed.count() / ramp_duration) * M_PI) + 1.0) / 2.0;
        // pros::lcd::print(4, "curve = %lf", curve);
        double final_modifier = std::pow(curve, ramp_strength);

        int move_speed = speed * final_modifier;
        move(move_speed);

        pros::delay(10);
    }

    move(0);
    // pros::lcd::print(1, "[Move_time] Exit Function");
}




