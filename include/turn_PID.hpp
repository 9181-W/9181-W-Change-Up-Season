#include "okapi/api.hpp"
using namespace okapi;

void gyro_turn(std::shared_ptr<ChassisController> chassis, QAngle angle, double max_speed = 100, double min_speed = 17.5, double kp = 0.013, double ki = 0.0, double kd = 0.0, double epsilon = 2);
void gyro_turn_to(std::shared_ptr<ChassisController> chassis, QAngle angle, double max_speed = 100, double min_speed = 17.5, double kp = 0.014, double ki = 0.0, double kd = 0.0, double = 2);
