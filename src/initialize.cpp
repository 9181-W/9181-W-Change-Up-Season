#include "initialize.hpp"
#include "okapi/api.hpp"
#include "inertial.hpp"
#include "ultrasonic.hpp"
#include "encoders.hpp"
using namespace okapi;

Controller master_controller(ControllerId::master);

Motor right_rear_mtr(RIGHT_REAR_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_rear_mtr(LEFT_REAR_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_front_mtr(RIGHT_FRONT_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_front_mtr(LEFT_FRONT_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_intake_mtr(LEFT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_intake_mtr(RIGHT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor bottom_mtr(BOTTOM_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor top_mtr(TOP_PORT, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

void modified_initialize()
{
  top_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	bottom_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	right_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	left_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);

  pros::lcd::initialize();
  pros::lcd::print(0, "initialize");
  //pros::lcd::print(3, "GOOD LUCK");

  encoder_initialize();
  inertial_initialize();
  line_sensor_initialize();

}
