#include "okapi/api.hpp"
using namespace okapi;

//void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, bool drive_straight = true);
void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed, double target_heading, double drive_straight_kp, double y_drive_kp, double x_drive_kp,  double turn_min_speed, bool drive_straight = true);
//min speed 15
