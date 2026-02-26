#include "autotuner.hpp"
#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <cstdio>

static void resetState() {
    // Stop motors and let robot settle
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(200); 
}

void AutoTuner::run(Autonomous& auton) {
    printf("AutoTuner Started\n");
    fflush(stdout);
    controller.set_text(0, 0, "Tuner Ready");

    while (true) {
        // Print menu
        printf("AUTO TUNER MENU\n");
        printf("X: Tune Turn PID\n");
        printf("Y: Tune Drive PID\n");
        printf("B: Exit\n");
        fflush(stdout);

        // Wait for selection
        bool selected = false;
        while (!selected) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                printf("ENTERING TUNING MODE...\n");
                fflush(stdout);
                tuneTurn(auton);
                printf("\n");
                fflush(stdout);
                selected = true;
            } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                tuneDrive(auton);
                printf("\n");
                fflush(stdout);
                selected = true;
            } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                return;
            }
            pros::delay(20);
        }
    }
}

void AutoTuner::tuneTurn(Autonomous& auton) {
    printf("Starting Turn Tuning...\n");
    fflush(stdout);
    controller.set_text(0, 0, "Tuning Turn...");
    // Initial Parameters [kP, kD]
    std::vector<double> p = {TURN_KP, TURN_KD};
    std::vector<double> dp = {0.5, 0.01}; // Initial step sizes (kD needs finer tuning)
    
    double best_err = runTurnTest(auton, p[0], p[1]);
    
    // Twiddle Algorithm (Coordinate Ascent)
    for (int i = 0; i < 12; i++) {
        for (int k = 0; k < 2; k++) { // Loop through kP then kD
            // Try increasing parameter
            p[k] += dp[k];
            double err = runTurnTest(auton, p[0], p[1]);

            if (err < best_err) {
                best_err = err;
                dp[k] *= 1.1; // Success, increase step size
            } else {
                // Increasing didn't work, try decreasing (2x because we added 1x before)
                p[k] -= 2 * dp[k];
                err = runTurnTest(auton, p[0], p[1]);

                if (err < best_err) {
                    best_err = err;
                    dp[k] *= 1.1; // Success, increase step size
                } else {
                    // Neither worked, put it back and reduce step size
                    p[k] += dp[k];
                    dp[k] *= 0.9;
                }
            }
            
            // Update status (no clear needed, just overwrite)
            printf("Tuning Turn...\n");
            printf("Best Cost: %.2f\n", best_err);
            printf("P: %.4f\n", p[0]);
            printf("D: %.4f\n", p[1]);
            fflush(stdout);
            
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) return;
        }
    }

    printf("TURN TUNING DONE\n");
    printf("Final P: %.5f\n", p[0]);
    printf("Final D: %.5f\n", p[1]);
    fflush(stdout);
    controller.set_text(0, 0, "Turn Done");
    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) pros::delay(10);
}

double AutoTuner::runTurnTest(Autonomous& auton, double kp, double kd) {
    // Setup test environment
    auton.turnPID.setConstants(kp, 0, kd);
    auton.turnPID.reset();
    
    // Reset sensors
    auton.updatePose();

    double startHeading = auton.getYaw();
    double target = startHeading + 90;
    auton.turnPID.setTarget(target);

    double total_error_sq = 0;
    int time_steps = 0;

    // Run Test Loop (1.5s)
    for (int t = 0; t < 150; t++) {
        double rawHeading = auton.getYaw();
        
        // Calculate squared error (penalizes large errors more)
        double error = target - rawHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        total_error_sq += (error * error);

        double output = auton.turnPID.compute(rawHeading, true);
        
        leftMotors.move_velocity(output);
        rightMotors.move_velocity(-output);

        pros::delay(10);
        time_steps++;
    }

    resetState();

    return total_error_sq / time_steps; // Return average squared error (Cost)
}

void AutoTuner::tuneDrive(Autonomous& auton) {
    printf("Starting Drive Tuning...\n");
    fflush(stdout);
    controller.set_text(0, 0, "Tuning Drive...");
    // Initial Parameters [kP, kD]
    std::vector<double> p = {DISTANCE_KP, DISTANCE_KD};
    std::vector<double> dp = {1.0, 0.1}; // Initial step sizes (kD needs finer tuning)
    
    double best_err = runDriveTest(auton, p[0], p[1]);
    
    for (int i = 0; i < 10; i++) { 
        for (int k = 0; k < 2; k++) {
            // Try increasing parameter
            p[k] += dp[k];
            double err = runDriveTest(auton, p[0], p[1]);

            if (err < best_err) {
                best_err = err;
                dp[k] *= 1.1; // Success
            } else {
                // Try decreasing
                p[k] -= 2 * dp[k];
                err = runDriveTest(auton, p[0], p[1]);

                if (err < best_err) {
                    best_err = err;
                    dp[k] *= 1.1; // Success
                } else {
                    // Revert and reduce step
                    p[k] += dp[k];
                    dp[k] *= 0.9;
                }
            }
            
            // Update status (no clear needed, just overwrite)
            printf("Tuning Drive...\n");
            printf("Best Cost: %.2f\n", best_err);
            printf("P: %.4f\n", p[0]);
            printf("D: %.4f\n", p[1]);
            fflush(stdout);
            
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) return;
        }
    }

    printf("DRIVE TUNING DONE\n");
    printf("Final P: %.5f\n", p[0]);
    printf("Final D: %.5f\n", p[1]);
    printf("WRITE THESE DOWN!\n");
    fflush(stdout);
    controller.set_text(0, 0, "Drive Done");
    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) pros::delay(10);
}

double AutoTuner::runDriveTest(Autonomous& auton, double kp, double kd) {
    // Setup test environment
    auton.distancePID.setConstants(kp, 0, kd);
    auton.distancePID.reset();
    auton.distancePID.setTarget(24);
    
    // Reset position and sensors
    auton.updatePose(); // Flush any pending odometry deltas before reset
    auton.setPose(0, 0);
    leftMotors.tare_position_all();
    rightMotors.tare_position_all();
    
    auton.updatePose();

    double total_error_sq = 0;
    int time_steps = 0;
    double target = 24;

    // Run Test Loop (2.0s)
    for (int t = 0; t < 200; t++) {
        auton.updatePose();
        double currentDist = std::hypot(auton.pos_x, auton.pos_y);
        
        // Calculate squared error
        double error = std::abs(target - currentDist);
        total_error_sq += (error * error);

        double output = auton.distancePID.compute(currentDist);
        
        leftMotors.move_velocity(output);
        rightMotors.move_velocity(output);

        pros::delay(10);
        time_steps++;
    }

    resetState();
    
    // Return to start
    // Use actual distance traveled to return safely, preventing wall crashes if the test failed
    double ticks_per_inch = (360 / (2.75 * M_PI));
    double return_ticks = -std::hypot(auton.pos_x, auton.pos_y) * ticks_per_inch;
    leftMotors.move_relative(return_ticks, 100);
    rightMotors.move_relative(return_ticks, 100);
    pros::delay(3000); // Increased delay to ensure robot returns fully
    resetState();

    return total_error_sq / time_steps; // Return average squared error (Cost)
}