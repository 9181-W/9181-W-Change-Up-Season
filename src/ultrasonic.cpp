#include "okapi/api.hpp"
using namespace okapi;

#define LINE_SENSOR_PORT_1 'A'
#define LINE_SENSOR_PORT_2 'B'

pros::ADILineSensor* line_sensor_1 = NULL;
pros::ADILineSensor* line_sensor_2 = NULL;

double line_sensor_value_1 = 0.0;
double line_sensor_value_2 = 0.0;

void line_sensor_reading(void* param)
{
  while (true)
  {
    line_sensor_value_1 = line_sensor_1->get_value();
    line_sensor_value_2 = line_sensor_2->get_value();
    // pros::lcd::print(6,"Line Sensor 1 Value %f",line_sensor_value_1);
    //pros::lcd::print(7,"Line Sensor 2 Value %f",line_sensor_value_2);
    pros::delay(33);
  }
}

void line_sensor_initialize()
{
  line_sensor_1 = new pros::ADILineSensor(LINE_SENSOR_PORT_1);
  line_sensor_2 = new pros::ADILineSensor(LINE_SENSOR_PORT_2);

  pros::Task line_sensor_task (line_sensor_reading, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Line Sensor Task");
}

double get_line_sensor_1_value()
{
  return line_sensor_value_1;
}

double get_line_sensor_2_value()
{
  return line_sensor_value_2;
}
