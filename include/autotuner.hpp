#pragma once

#include "autonomous.hpp"

class AutoTuner {
    public:
        // Run the interactive auto-tuning menu
        static void run(Autonomous& auton);

    private:
        static void tuneTurn(Autonomous& auton);
        static void tuneDrive(Autonomous& auton);
        
        // Helper to run a single test and return the "cost" (total error)
        static double runTurnTest(Autonomous& auton, double kp, double kd);
        static double runDriveTest(Autonomous& auton, double kp, double kd);
};