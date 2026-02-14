// #include "autoFunctions.h"
// #include "main.h"
// #include "subsystems.h"
// #include "constants.h"
// #include "chrono"
// #include "odometry.h"

// void move(int leftVelocity, int rightVelocity) {
//     leftMotors.move_velocity(leftVelocity);
//     rightMotors.move_velocity(rightVelocity);
// }

// void move(int speed){
//     move(speed, speed);
// }

// void move_time(int leftVelocity, int rightVelocity, double seconds) {
//     move(leftVelocity, rightVelocity);
//     pros::delay(1000*seconds);
//     move(0);
// }

// void move_time_pid_heading(int speed, double seconds, double heading) {
//     double kp = 2.25, kd = 0.0; // kp 3.4 // kd 0.3
//     double original_heading = get_yaw_quaternion();
//     double pid_res, error, derivative = 0;
//     std::optional<double> previous_error;

//     auto start = std::chrono::steady_clock::now();
//     while (true) {
//         heading = get_yaw_quaternion();
        
//         error = original_heading - heading;
//         if (error > 180) error -= 360;
// 		else if (error < -180) error += 360;
// 		if (error == 180) error = 179.99;

//         if (previous_error.has_value()) {
//             derivative = error - previous_error.value();
//         }

//         pid_res = (error * kp) + (derivative * kd);
        
//         move(speed + pid_res, speed - pid_res);
//         previous_error = error;

//         std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
//         if (elapsed.count() > seconds) {
//             break;
//         }

//         pros::delay(10);
//     }
// }

// void move_time_pid(int speed, double seconds) {
//     double heading = get_yaw_quaternion();
//     move_time_pid_heading(speed, seconds, heading);
// }

// void move_time_pid_stop(int speed, double seconds) {
//     move_time_pid(speed, seconds);
//     move(0);
// }


// void move_time_s(int speed, double seconds, double ramp_duration, int ramp_strength) {
//     if (seconds < ramp_duration) {
//         ramp_duration = seconds;
//     }

//     auto start = std::chrono::steady_clock::now();
    
//     double full_speed_duration = seconds - ramp_duration;
//     move_time_pid(speed, full_speed_duration);
//     rampDown_s(speed, ramp_duration, ramp_strength);
//     move(0);
// }

// void rampDown_s(int speed, double ramp_duration, int ramp_strength) {
//     auto ramp_start = std::chrono::steady_clock::now();

//     while (true) {
//         std::chrono::duration<double> ramp_elapsed = std::chrono::steady_clock::now() - ramp_start;
//         if (ramp_elapsed.count() > ramp_duration) {
//             break;
//         }

//         double curve = (std::cos((ramp_elapsed.count() / ramp_duration) * M_PI) + 1.0) / 2.0;
//         double final_modifier = std::pow(curve, ramp_strength);

//         int move_speed = speed * final_modifier;
//         move(move_speed);

//         pros::delay(10);
//     }

//     move(0);
// }
