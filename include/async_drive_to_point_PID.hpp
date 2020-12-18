#include "okapi/api.hpp"
using namespace okapi;

void async_drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed, double target_heading, double drive_straight_kp, double y_drive_kp, double x_drive_kp, double turn_min_speed);
void wait_for_drive_complete_2();
