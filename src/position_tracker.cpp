#include "okapi/api.hpp"
#include "inertial.hpp"
#include "encoders.hpp"
#include "utils.hpp"

using namespace okapi;

//Link to 5225A github github.com/nickmertin/5225A-2017-2018/blob/master/src/auto.c
//
// The following algorithm come from: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
//
// The following variables are used in this document to represent physical parameters and other known
// values:
// • 𝑠𝐿   is the left-right distance from the tracking center to the left tracking wheel
// • 𝑠𝑅   is the left-right distance from the tracking center to the right tracking wheel
// • 𝑠𝑆   is the forward-backward distance from the tracking center to the back tracking wheel
// • 𝑑0  ⃗⃗⃗⃗ is the previous global position vector
// • 𝜃0   is the previous global orientation
// • 𝜃𝑟   is the global orientation at last reset
//
// Tracking Algorithm
// The algorithm itself consists of a single procedure to perform calculations, which should be called
// frequently (a cycle period of no more than 10 milliseconds). We recommend using a dedicated runtime
// task/thread for this process.
//
// The procedure can be broken down into a few steps:
//
// 1. Store the current encoder values in local variables
//
// 2. Calculate the change in each encoders’ value since the last cycle, and convert to distance of
// wheel travel (for example, if the wheel has a radius 2" and turned 5°, then it travelled
// approximately 0.1745"); call these Δ𝐿, Δ𝑅, and Δ𝑆
//
// 3. Update stored "previous values" of encoders
//
// 4. Calculate the total change in the left and right encoder values since the last reset, and convert
// to distance of wheel travel call these Δ𝐿𝑟 and Δ𝑅𝑟
//
// 5. Calculate new absolute orientation 𝜃1 = 𝜃𝑟 + (Δ𝐿𝑟−Δ𝑅𝑟 / 𝑠𝐿+𝑠𝑅)
// ; please note that the second term will be in radians, regardless of the units of other variables

// Looking at 5225A's code you can see that step 4 + 5 are ignored and Δ𝜃 is calculated as below

//
// 6. Calculate the change in angle Δ𝜃 = (Δ𝐿 - Δ𝑅 / sL + sR)
//
// 7. If Δ𝜃 = 0 (i.e. Δ𝐿 = Δ𝑅), then calculate the local offset Δ𝑑𝑙⃗ = [ Δ𝑆  Δ𝑅 ]
//
// 8. Otherwise, calculate the local offset Δ𝑑⃗⃗ = 2 sin 𝜃 / 2 x [Δ𝑆 / Δ𝜃 + 𝑠𝑆   Δ𝑅 / Δ𝜃 + 𝑠𝑅]
//
// 9. Calculate the average orientation 𝜃𝑚 = 𝜃0 + Δ𝜃 / 2
//
// 10. Calculate global offset Δ𝑑⃗  as Δ𝑑⃗⃗ rotated by −𝜃𝑚; this can be done by converting your existing
// Cartesian coordinates to polar coordinates, changing the angle, then converting back
//
// 11. Calculate new absolute position 𝑑1⃗⃗⃗ = 𝑑0 ⃗⃗ + Δ𝑑⃗

//runs position tracking code
// void position_tracker_task(void* param)
// {
//   //The left-right distance from the tracking center to the left tracking wheel
//   const double sL = 6.5;
//   //The left-right distance from the tracking center to the right tracking wheel
//   const double sR = 6.5;
//   //The forward-backward distance from the tracking center to the back tracking wheel
//   const double sS = 3.0;
//   //Creates a constant for wheel diameter
//   const double wheel_diam = 2.75;
//   //Creates a constant for pi
//   const double drive_pi = 3.14159265359;
//   //Calculates a constant wheel circumference using diameter and pi
//   const double wheel_circ = wheel_diam * drive_pi;
//   //Encoder degrees in circumference
//   const double degrees_per_circ = 360.0;
//   //Encoder degrees per inch
//   const double degrees_per_inch = degrees_per_circ / wheel_circ;
//
//   //Creates variables for storing the previous linear value of the shaft encoders
//   double previous_left_shaft_val = 0.0;
//   double previous_right_shaft_val = 0.0;
//   double previous_middle_shaft_val = 0.0;
//
//   //Creates a variable to keep track of the last calculated angle
//   QAngle theta_0 = 0_deg;
//
//   while(true)
//   {
//     // 1. Store the current encoder values in local variables
//     double left_shaft_val = (shaft_enc_l->get() / degrees_per_inch);
//     double right_shaft_val = (shaft_enc_r->get() / degrees_per_inch);
//     double middle_shaft_val = (shaft_enc_m->get() / degrees_per_inch);
//
//     //2. Calculate the change in each encoders’ value since the last cycle
//     double delta_R = (left_shaft_val - previous_left_shaft_val);
//     double delta_L = (right_shaft_val - previous_right_shaft_val);
//     double delta_S = (middle_shaft_val - previous_middle_shaft_val);
//
//     //3. Update stored "previous values" of encoders
//     double previous_left_shaft_val = left_shaft_val;
//     double previous_right_shaft_val = right_shaft_val;
//     double previous_middle_shaft_val = middle_shaft_val;
//
//     //6. Calculate the change in angle Δ𝜃 = (Δ𝐿 - Δ𝑅 / sL + sR)
//     QAngle delta_theta = (((delta_L - delta_R) / (sL + sR)) * radian);
//
//     //7. If Δ𝜃 = 0 (i.e. Δ𝐿 = Δ𝑅), then calculate the local offset Δ𝑑𝑙⃗ = [ Δ𝑆  Δ𝑅 ]
//     // We need an If statement for driving straight because in step 8 we calculate sinΔ𝜃 (Δ𝜃 = 0 when driving straight) and the sin of 0 is = infinity (this will screw up the code)
//
//     // creates a variable to store the change in
//     double delta_d_1_x = 0.0;
//     double delta_d_1_y = 0.0;
//     double delta_d_x = 0.0;
//     double delta_d_y = 0.0;
//
//     //if (delta_theta == 0.0_deg)// cannot do an equal sign here because floats cannot be compared to zero because it will never be exactly 0 (merci beaucoup mon père)
//     if ((delta_theta > -0.0001_deg) && (delta_theta < 0.0001_deg))
//     {
//       delta_d_1_x = delta_S;
//       delta_d_1_y = delta_R; // Can be R or L because when driving straight R and L will be the Same
//     }
//
//     //8. Otherwise, calculate the local offset Δ𝑑⃗⃗ = 2 sin 𝜃 / 2 x [Δ𝑆 / Δ𝜃 + 𝑠𝑆   Δ𝑅 / Δ𝜃 + 𝑠𝑅]
//     else
//     {
//       // delta_d_x = (((2 * sin) * (inertial_get_value / 2)) * (delta_S / delta_theta + sS));
//       // delta_d_y = (((2 * sin) * (inertial_get_value / 2)) * (delta_R / delta_theta + sR));
//
//       delta_d_x = 2 * sin * inertial_get_value() / 2 * (delta_S / delta_theta + sS);
//       delta_d_y = 2 * sin * inertial_get_value() / 2 * (delta_R / delta_theta + sR);
//     }
//
//     //9. Calculate the average orientation 𝜃𝑚 = 𝜃0 + Δ𝜃 / 2
//     QAngle theta_m = theta_0 + delta_theta / 2;
//
//
//
//     pros::delay(10);
//   }
//
// }
//
// //starts the task that will read the location of the robot
// void tracker_initialize()
// {
//   //uses the built-in pros task creator to start a task
//   pros::Task position_tracker (position_tracker_task, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
//     TASK_STACK_DEPTH_DEFAULT, "Position Tracker Task");
// }
