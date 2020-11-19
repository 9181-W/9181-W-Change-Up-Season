#include "okapi/api.hpp"
using namespace okapi;

extern std::shared_ptr<ChassisController> chassis;

void time_strafe(double speed);
void time_drive(double speed);
void motor_stop();
void intake_on(double speed);
void intake_off();
void top_and_bottom_spin();
void top_and_bottom_off();
void top_and_bottom_spin_backwards();
void top_and_bottom_eject();
void intake_ball();
