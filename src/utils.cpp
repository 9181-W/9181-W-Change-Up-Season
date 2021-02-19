#include "okapi/api.hpp"
using namespace okapi;
#include "initialize.hpp"
#include "utils.hpp"
#include "ultrasonic.hpp"
#include "inertial.hpp"

std::shared_ptr<ChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors(
      // LEFT_FRONT_WHEEL_PORT, -RIGHT_FRONT_WHEEL_PORT,
      // -RIGHT_REAR_WHEEL_PORT, LEFT_REAR_WHEEL_PORT
      -LEFT_FRONT_WHEEL_PORT, RIGHT_FRONT_WHEEL_PORT,
      RIGHT_REAR_WHEEL_PORT, -LEFT_REAR_WHEEL_PORT
    )
    .withDimensions(AbstractMotor::gearset::green, {{4.0_in, 13_in}, imev5GreenTPR})//4, 13
    .build();

void time_strafe(double speed)
{
  left_rear_mtr.moveVelocity(-speed);
  left_front_mtr.moveVelocity(speed);
  right_rear_mtr.moveVelocity(speed);
  right_front_mtr.moveVelocity(-speed);
}

void time_drive(double speed)
{
  left_rear_mtr.moveVelocity(speed);
  left_front_mtr.moveVelocity(speed);
  right_rear_mtr.moveVelocity(speed);
  right_front_mtr.moveVelocity(speed);
}

void motor_stop()
{
  left_rear_mtr.moveVelocity(0);
  left_front_mtr.moveVelocity(0);
  right_rear_mtr.moveVelocity(0);
  right_front_mtr.moveVelocity(0);
}

void intake_on(double speed)
{
  right_intake_mtr.moveVelocity(-speed);
  left_intake_mtr.moveVelocity(speed);
}

void intake_off()
{
  right_intake_mtr.moveVelocity(0);
  left_intake_mtr.moveVelocity(0);
}

void top_and_bottom_spin()
{
  bottom_mtr.moveVelocity(-280);
  top_mtr.moveVelocity(600);
}

void top_and_bottom_spin_slow()
{
  bottom_mtr.moveVelocity(-100);
  top_mtr.moveVelocity(100);
}

void top_and_bottom_eject()
{
  bottom_mtr.moveVelocity(600);
  top_mtr.moveVelocity(-600);//300
}

void top_and_bottom_spin_backwards()
{
  bottom_mtr.moveVelocity(200);
  top_mtr.moveVelocity(-200);
}

void top_and_bottom_off()
{
  bottom_mtr.moveVelocity(0);
  top_mtr.moveVelocity(0);
}

void intake_ball()
{
  int start_time = pros::millis();
  pros::delay(12);
  while((get_line_sensor_2_value() > 2500) && ((pros::millis() - start_time) < 5000) && (get_line_sensor_1_value() > 2500))
  {
    bottom_mtr.moveVelocity(-225);
    top_mtr.moveVelocity(225);
    pros::delay(33);
  }
  bottom_mtr.moveVelocity(0);
  top_mtr.moveVelocity(0);
}

// void intake_ball()
// {
//     bottom_mtr.moveVelocity(-300);
//     top_mtr.moveVelocity(600);
// }
