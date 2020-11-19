#include "okapi/api.hpp"
using namespace okapi;

//void gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, bool drive_straight = true, double kp = 0.013, double ki = 0.0, double kd = 0.0);
void gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, bool drive_straight = true);
//void async_gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, double kp = 0.013, double ki = 0.0, double kd = 0.0);
void async_gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed);
bool drive_is_complete();
void wait_for_drive_complete();
