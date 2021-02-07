#include "okapi/api.hpp"
#include "inertial.hpp"
#include "position_PID.hpp"
#include "encoders.hpp"
#include "utils.hpp"
#include "position_tracker.hpp"
#include "drive_to_point_PID.hpp"

using namespace okapi;

//creates variables that will have assigned values for the asynchronous function
std::shared_ptr<ChassisController> async_chassis_2;
QLength async_y_distance = 0.0_in;
double async_y_max_speed = 0.0;
double async_y_min_speed = 0.0;
QLength async_x_distance = 0.0_in;
double async_x_max_speed = 0.0;
double async_x_min_speed = 0.0;
double async_target_heading = 0.0;
double async_drive_straight_kp = 0.0;
double async_y_drive_kp = 0.0;
double async_x_drive_kp = 0.0;
double async_turn_min_speed = 0.0;
bool async_complete_2 = true;
pros::Task* drive_task_2 = NULL;

//task that will run the drive to point function with new values
void async_drive_to_point(void* param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_2)
    {
      //gyro_drive(async_chassis_2, async_distance_2, async_max_speed_2);
      drive_to_point(chassis, async_y_distance, async_y_max_speed, async_y_min_speed, async_x_distance, async_x_max_speed, async_x_min_speed, async_target_heading, async_drive_straight_kp, async_y_drive_kp, async_x_drive_kp, async_turn_min_speed, true);
      //sets the drive to a completed state
      async_complete_2 = true;
    }
    pros::delay(10);
  }
}

//manual function to set the drive to a completed state
bool drive_is_complete_2()
{
  return async_complete_2;
}

//waits until the ansychronous drive has completed
void wait_for_drive_complete_2()
{
  while(!async_complete_2)
  {
    pros::delay(10);
  }
}

//uses a new function to assign values to the asynchronous variables
void async_drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed, double target_heading, double drive_straight_kp, double y_drive_kp, double x_drive_kp, double turn_min_speed)
{
  //assigns values to the asynchronous variables
  async_chassis_2 = chassis;
  async_y_distance = y_distance;
  async_y_max_speed = y_max_speed;
  async_y_min_speed = y_min_speed;
  async_x_distance = x_distance;
  async_x_max_speed = x_max_speed;
  async_x_min_speed = x_min_speed;
  async_target_heading = target_heading;
  async_drive_straight_kp = drive_straight_kp;
  async_y_drive_kp = y_drive_kp;
  async_x_drive_kp = x_drive_kp;
  async_turn_min_speed = turn_min_speed;

  //runs the drive to point as an asynchronous task
  if (drive_task_2 == NULL)
  {
    drive_task_2 = new pros::Task(async_drive_to_point, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Drive Task");
  }

  async_complete_2 = false;
}
