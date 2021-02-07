#include "initialize.hpp"
#include "okapi/api.hpp"
#include "inertial.hpp"
#include "ultrasonic.hpp"
#include "encoders.hpp"
#include "position_tracker.hpp"
using namespace okapi;

//creates the controller
Controller master_controller(ControllerId::master);

//creates the motors
Motor right_rear_mtr(RIGHT_REAR_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_rear_mtr(LEFT_REAR_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_front_mtr(RIGHT_FRONT_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_front_mtr(LEFT_FRONT_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_intake_mtr(LEFT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_intake_mtr(RIGHT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor bottom_mtr(BOTTOM_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor top_mtr(TOP_PORT, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

//runs initializations
void modified_initialize()
{
  //sets the brake types
  top_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	bottom_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	right_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	left_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);

  //inits. the lcd screen
  pros::lcd::initialize();
  //prints init. to the lcd
  pros::lcd::print(0, "initialize");

  //inits. the encoders
  encoder_initialize();
  //inits. the inertials
  inertial_initialize();
  //inits. the line sensors
  line_sensor_initialize();
  //inits. the position tracker
  tracker_initialize();

}
