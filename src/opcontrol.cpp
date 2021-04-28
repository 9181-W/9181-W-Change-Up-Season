#include "opcontrol.hpp"
#include "ultrasonic.hpp"
#include "okapi/api.hpp"
#include "initialize.hpp"
#include "utils.hpp"

//tank drive controls
void tank_drive(double gearset_rpm = 200)
{
  //creates speed variables and inserts the values from the joysticks
  double left_speed = master_controller.getAnalog(okapi::ControllerAnalog::leftY) * gearset_rpm;
  double right_speed = master_controller.getAnalog(okapi::ControllerAnalog::rightY) * gearset_rpm;

  //sets the speeds of the motors
  left_rear_mtr.moveVelocity(left_speed);
  left_front_mtr.moveVelocity(left_speed);
  right_rear_mtr.moveVelocity(-right_speed);
  right_front_mtr.moveVelocity(-right_speed);
}

// void arcade_drive(double gearset_rpm = 200)
// {
//   // getAnalog returns a float -1,1
//   double leftY = master_controller.getAnalog(okapi::ControllerAnalog::leftY);
//   double rightX = master_controller.getAnalog(okapi::ControllerAnalog::rightX);
//   double leftX = master_controller.getAnalog(okapi::ControllerAnalog::leftX);
//
//   std::shared_ptr<ChassisModel> chassis_model = chassis->getModel();
//   std::shared_ptr<XDriveModel> chassis_x_model = std::dynamic_pointer_cast<XDriveModel>(chassis_model);
//   chassis_x_model->xArcade(leftX, leftY, rightX);
// }

void arcade_drive(double gearset_rpm = 200)
{
  // getAnalog returns a float -1,1
  double leftY = master_controller.getAnalog(okapi::ControllerAnalog::leftY);
  double rightX = master_controller.getAnalog(okapi::ControllerAnalog::rightX);
  double leftX = master_controller.getAnalog(okapi::ControllerAnalog::leftX);

  //calculate individual wheel speeds between -1,1
  // double front_left_speed = (leftY * 0.83) + (rightX * 0.87) + leftX;
  // double front_right_speed = (leftY * 0.83) - (rightX * 0.87) - leftX;
  // double back_left_speed = (leftY * 0.83) + (rightX * 0.87) - leftX;
  // double back_right_speed = (leftY * 0.83) - (rightX * 0.87) + leftX;

  double front_left_speed = (leftY) + (rightX) + leftX;
  double front_right_speed = (leftY) - (rightX) - leftX;
  double back_left_speed = (leftY) + (rightX) - leftX;
  double back_right_speed = (leftY) - (rightX) + leftX;

  //calculate wheel speeds in rpm
  front_left_speed *= gearset_rpm;
  front_right_speed *= gearset_rpm;
  back_left_speed *= gearset_rpm;
  back_right_speed *= gearset_rpm;

  //clamps the wheel speed to a certain value
  front_left_speed = std::clamp(front_left_speed, -gearset_rpm, gearset_rpm);
  front_right_speed = std::clamp(front_right_speed, -gearset_rpm, gearset_rpm);
  back_left_speed = std::clamp(back_left_speed, -gearset_rpm, gearset_rpm);
  back_right_speed = std::clamp(back_right_speed, -gearset_rpm, gearset_rpm);

  //moves the motors at the calculated value
  left_front_mtr.moveVelocity(front_left_speed);
  right_front_mtr.moveVelocity(front_right_speed);
  left_rear_mtr.moveVelocity(back_left_speed);
  right_rear_mtr.moveVelocity(back_right_speed);

  if ((master_controller.getDigital(okapi::ControllerDigital::A)) == true)
  {
    right_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
    left_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
    right_front_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
    left_front_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  }
}

//controls for the front two intakes
void intake_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    left_intake_mtr.moveVelocity(600);
    right_intake_mtr.moveVelocity(-600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::B)) == true)
  {
    left_intake_mtr.moveVelocity(600);
    right_intake_mtr.moveVelocity(-600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    left_intake_mtr.moveVelocity(-600);
    right_intake_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    left_intake_mtr.moveVelocity(-600);
    right_intake_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L2)) == true)
  {
    left_intake_mtr.moveVelocity(-600);
    right_intake_mtr.moveVelocity(600);
  }

  else
  {
    left_intake_mtr.moveVelocity(0);
    right_intake_mtr.moveVelocity(0);
  }
}

//controls the bottom roller motor
void bottom_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    bottom_mtr.moveVelocity(-600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::B)) == true)
  {
    top_mtr.moveVelocity(-25);
  }

  //if the line sensor detects a ball the bottom motor is shut off
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_2_value() < 2875) && (get_line_sensor_1_value() < 2875))
  {
    bottom_mtr.moveVelocity(-0);
  }

  //makes the top motor move slightly backwards if the ball goes slightly past the first line sensor
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_1_value() < 2875))
  {
    top_mtr.moveVelocity(300);
  }

  //if the line sensor detects a ball the bottom motor is shut off
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_2_value() < 2875))
  {
    bottom_mtr.moveVelocity(-0);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    bottom_mtr.moveVelocity(-250);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::X)) == true)
  {
    bottom_mtr.moveVelocity(-300);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    bottom_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    bottom_mtr.moveVelocity(-300);
  }

  else
  {
    bottom_mtr.moveVelocity(0);
  }
}

//controls for the top roller control motor
void top_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    top_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::B)) == true)
  {
    top_mtr.moveVelocity(25);
  }

  //shuts of the top motor if the line sensors detect a ball
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_2_value() < 2875) && (get_line_sensor_1_value() < 2875))
  {
    top_mtr.moveVelocity(0);
  }

  //makes the top motor move slightly backwards if the ball goes slightly past the first line sensor
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_1_value() < 2875))
  {
    top_mtr.moveVelocity(-250);
  }

  //shuts of the top motor if the line sensors detect a ball
  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_line_sensor_2_value() < 2875))
  {
    top_mtr.moveVelocity(0);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    top_mtr.moveVelocity(300);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    top_mtr.moveVelocity(-600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L2)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    top_mtr.moveVelocity(600);
  }

  else
  {
    top_mtr.moveVelocity(0);
  }
}

//runs during the opcontrol function call
void modified_opcontrol()
{
  //sets brake modes
  top_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	bottom_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	right_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	left_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
  right_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	left_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	right_front_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
	left_front_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);

  //runs specific controls
  while (true)
  {
    arcade_drive();
    intake_controls();
    bottom_controls();
    top_controls();

    pros::delay(33);
  }
}
