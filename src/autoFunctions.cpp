#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "chrono"

void move(int leftVelocity, int rightVelocity) {
    leftMotors.move_velocity(leftVelocity);
    rightMotors.move_velocity(rightVelocity);
}

void move(int speed){
    move(speed, speed);
}

void move_time_s(int speed, double seconds, int ramp_strength) {
    double s_curve_duration_sec = 2;
    if (seconds < s_curve_duration_sec) {
        s_curve_duration_sec = seconds;
    }

    auto start = std::chrono::steady_clock::now();
    
    move(speed);    

    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    double s_curve_end_time = seconds - s_curve_duration_sec;
    while (elapsed.count() < s_curve_end_time) {
        elapsed = std::chrono::steady_clock::now() - start;
        pros::delay(10);
    }

    auto ramp_start = std::chrono::steady_clock::now();

    while (true) {
        double ramp_elapsed = (std::chrono::steady_clock::now() - ramp_start).count();
        
        if (ramp_elapsed > s_curve_duration_sec) {
            break;
        }

        double curve = (std::cos(ramp_elapsed / s_curve_duration_sec * M_PI) + 1);
        double final_modifier = std::pow(curve, ramp_strength);
        move(static_cast<int>(speed * final_modifier));

        pros::delay(10);
    }

    move(0);
}

void turn(int speed, int direction){
    speed = speed*direction;
    move(speed, -speed);
}

void move(int leftVelocity, int rightVelocity, double time) {
    leftMotors.move_velocity(leftVelocity);
    rightMotors.move_velocity(rightVelocity);
    pros::delay(1000*time);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}


