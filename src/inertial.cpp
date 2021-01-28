#include "okapi/api.hpp"
using namespace okapi;

#define INERTIAL_PORT_1 14
#define INERTIAL_PORT_2 15

pros::Imu* inertial_1 = NULL;
pros::Imu* inertial_2 = NULL;

double inertial_value_1 = 0.0;
double inertial_value_2 = 0.0;

void inertial_reading(void* param)
{
  while (true)
  {
    if (inertial_1->is_calibrating() == true)
    {
      inertial_value_1 = 0;
    }
    else
    {
      inertial_value_1 = inertial_1->get_rotation();
    }

    if (inertial_2->is_calibrating() == true)
    {
      inertial_value_2 = 0;
    }
    else
    {
      inertial_value_2 = inertial_2->get_rotation();
    }

    pros::lcd::print(1,"Inertial Value 1 %5.2f",(inertial_value_1));
    pros::lcd::print(2,"Inertial Value 2 %5.2f",(inertial_value_2));
    pros::delay(33);
  }
}

void inertial_initialize()
{
  inertial_1 = new pros::Imu(INERTIAL_PORT_1);
  inertial_2 = new pros::Imu(INERTIAL_PORT_2);

  inertial_1->reset();
  inertial_2->reset();

  double start_time = pros::c::millis();
  pros::lcd::print(7,"CALIBRATING");
  pros::delay(10);


  do {
    pros::delay(50);

  } while((inertial_1->is_calibrating() == true) && (inertial_2->is_calibrating() == true));

  double end_time = pros::c::millis();
  //pros::lcd::print(5,"Calibration Time %f", end_time - start_time);

  pros::Task inertial_value_task (inertial_reading, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Inertial Value Task");
}

void inertial_reset()
{
  //inertial_1->reset();
}

double inertial_get_value()
{
  //return (inertial_value_2);
  return ((inertial_value_1 + inertial_value_2) / 2);
}
