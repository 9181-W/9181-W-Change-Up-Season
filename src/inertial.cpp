#include "okapi/api.hpp"
using namespace okapi;

//defines which ports the inertial sensors are in
#define INERTIAL_PORT_1 14
#define INERTIAL_PORT_2 15

//creates the inertials as empty objects
pros::Imu* inertial_1 = NULL;
pros::Imu* inertial_2 = NULL;

//sets the initial values of the inertials to zero
double inertial_value_1 = 0.0;
double inertial_value_2 = 0.0;

//task function to read the inertials
void inertial_reading(void* param)
{
  while (true)
  {
    //waits until the inertial has calibrated until it draws the value
    if (inertial_1->is_calibrating() == true)
    {
      inertial_value_1 = 0;
    }
    //draws the inertial values
    else
    {
      inertial_value_1 = inertial_1->get_rotation();
    }
    //waits until the inertial has calibrated until it draws the value
    if (inertial_2->is_calibrating() == true)
    {
      inertial_value_2 = 0;
    }
    //draws the inertial values
    else
    {
      inertial_value_2 = inertial_2->get_rotation();
    }
    //prints the inertial values to the lcd screen
    pros::lcd::print(1,"Inertial Value 1 %5.2f",(inertial_value_1));
    pros::lcd::print(2,"Inertial Value 2 %5.2f",(inertial_value_2));
    //waits so the loop doesnt run too fast
    pros::delay(10);
  }
}

//initializes the inertial sensors
void inertial_initialize()
{
  //creates the inetial sensors
  inertial_1 = new pros::Imu(INERTIAL_PORT_1);
  inertial_2 = new pros::Imu(INERTIAL_PORT_2);

  //resets the inertial sensors
  inertial_1->reset();
  inertial_2->reset();

  //prints calibrating to the lcd
  pros::lcd::print(7,"CALIBRATING");
  pros::delay(10);

  //waits while the inertial sensors are calibrating
  do
  {
    pros::delay(50);
  }
  while((inertial_1->is_calibrating() == true) && (inertial_2->is_calibrating() == true));

  //task for reading the inertials
  pros::Task inertial_value_task (inertial_reading, (void*)"PROSV5", TASK_PRIORITY_MAX,
    TASK_STACK_DEPTH_DEFAULT, "Inertial Value Task");
}

//sets the initial zero location of the inertial sensors to zero
double zero_inertial_1 = 0.0;
double zero_inertial_2 = 0.0;

//creates a new location on the inertials to be zero degrees
void inertial_reset()
{
  //sets the zeroes to be the current inertial values
  zero_inertial_1 = inertial_1->get_rotation();
  zero_inertial_2 = inertial_2->get_rotation();
}

//gets the values of the inertials
double inertial_get_value()
{
  //inertial value is equa to the current value minus the predetermined zero location
  double curr_1 = inertial_value_1 - zero_inertial_1;
  double curr_2 = inertial_value_2 - zero_inertial_2;
  //returns an average of the two inertial values
  return ((curr_1 + curr_2) / 2);
}
