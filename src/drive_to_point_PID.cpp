#include "okapi/api.hpp"
#include "inertial.hpp"
#include "position_PID.hpp"
#include "encoders.hpp"
#include "utils.hpp"
#include "position_tracker.hpp"

using namespace okapi;

//Creates a constant for wheel diameter
const double wheel_diam = 2.75;
//Creates a constant for pi
const double drive_pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * drive_pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;
//creates the maximum velocity adj. as a global so it can be adjusted by functions later
double maximum_vel_adj = 0.1;
//creates the epsilons as globals so they can be adjusted by functions later
double drive_straight_epsilon = 1;
double y_epsilon = 0.05;//5
double y_distance_epsilon = 0.5;
double x_epsilon = 0.05;//75
double x_distance_epsilon = 1.0;

//Drive X distance at Y speed
//void gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, bool drive_straight, double kp, double ki, double kd)
void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed, double target_heading, double drive_straight_kp, double y_drive_kp, double x_drive_kp,  double turn_min_speed, bool drive_straight)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    y_max_speed = y_max_speed / 100;
    x_max_speed = x_max_speed / 100;

    y_min_speed = y_min_speed / 100;
    x_min_speed = x_min_speed / 100;

    // if (x_min_speed < 0.3) x_min_speed = 0.3;
    // if (y_min_speed < 0.3) y_min_speed = 0.3;

    //Sets the encoder units to use degrees instead of ticks
    chassis->getModel()->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    //sets the derivative values for both x and y
    const double y_drive_kd = -0.001;
    const double x_drive_kd = -0.001;

    //
    const double zero_speed = 0.0075;

    //Converts Qlength distance to distance_in_inches
    double y_target_coordinate = y_distance.convert(inch);
    double x_target_coordinate = x_distance.convert(inch);
    //Draws starting position from the encoders (found in chassisController.cpp on github)
    double y_start_pos_value = get_y_position();
    double x_start_pos_value = get_x_position();
    //Calculates current position based on start position (found in chassisController.cpp on github)
    double y_current_pos_value = get_y_position() - y_start_pos_value;
    double x_current_pos_value = get_x_position() - x_start_pos_value;
    //
    double y_relative_initial_drive_error = y_target_coordinate - get_y_position();
    double x_relative_initial_drive_error = x_target_coordinate - get_x_position();
    // double x_initial_drive_error = okapi::cos(get_heading()).getValue() * x_relative_initial_drive_error - okapi::sin(get_heading()).getValue() * x_relative_initial_drive_error;
    // double y_initial_drive_error = okapi::cos(get_heading()).getValue() * y_relative_initial_drive_error + okapi::sin(get_heading()).getValue() * y_relative_initial_drive_error;
    double initial_drive_error = sqrt((y_relative_initial_drive_error * y_relative_initial_drive_error) + (x_relative_initial_drive_error * x_relative_initial_drive_error));
    double total_drive_error = initial_drive_error;

    //Sets last error to zero before driving starts
    double y_last_error = 0.0;
    double x_last_error = 0.0;

    //sets previous errors to 9999.9 so robot does not accidentally time-out
    double y_second_last_error = 9999.9;
    double x_second_last_error = 9999.9;
    double y_third_last_error = 9999.9;
    double x_third_last_error = 9999.9;
    double y_fourth_last_error = 9999.9;
    double x_fourth_last_error = 9999.9;

    //Defines the initial drive errors (0.0)
    double y_drive_error = y_target_coordinate - get_y_position();
    double x_drive_error = x_target_coordinate - get_x_position();
    double drive_straight_error = target_heading - inertial_get_value();

    //Creates a variable that contains the initial gyro value (0)
    double initial_drive_gyro_value = -inertial_get_value();

    //Sets the first previous speed to zero
    double y_last_speed = 0.0;
    double x_last_speed = 0.0;

    //sets the total of the last three changes in error to 9999.9 so the robot does not time out
    double y_last_three_derivatives = 9999.9;
    double x_last_three_derivatives = 9999.9;



    //Drive while the robot isn't too slow (checks if the robot is moving fast enough to warrant continuation of driving) or hasnt driven half of its target distance or hasnt finished turning
    while ( ((fabs(y_last_three_derivatives) > y_epsilon) || (fabs(total_drive_error) > fabs(initial_drive_error) / 2.0)) ||
           ((fabs(x_last_three_derivatives) > x_epsilon) || (fabs(total_drive_error) > fabs(initial_drive_error) / 2.0)) ||
           ((fabs(drive_straight_error) > drive_straight_epsilon * 3)) )
    // while(true)
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        printf("tar x: %5.1f  tar y: %5.1f\n", x_target_coordinate, y_target_coordinate);
        printf("cur x: %5.1f  cur y: %5.1f  head: %5.2f\n", get_x_position(), get_y_position(), get_heading().convert(degree));
        printf("x_err: %5.1f  y_err: %5.1f\n",x_drive_error,y_drive_error);

        //converts the global coordinates from the position tracker into locations for the robot to drive to
        //https://gamedev.stackexchange.com/questions/79765/how-do-i-convert-from-the-global-coordinate-space-to-a-local-space
        double relativeX = x_target_coordinate - get_x_position();
        double relativeY = y_target_coordinate - get_y_position();
        x_drive_error = okapi::cos(get_heading()).getValue() * relativeX - okapi::sin(get_heading()).getValue() * relativeY;
        y_drive_error = okapi::cos(get_heading()).getValue() * relativeY + okapi::sin(get_heading()).getValue() * relativeX;
        total_drive_error = sqrt((x_drive_error * x_drive_error) + (y_drive_error * y_drive_error));

        //exit if we have reached the target
        if((fabs(y_drive_error) < y_distance_epsilon) && (fabs(x_drive_error) < x_distance_epsilon) && (fabs(drive_straight_error) < drive_straight_epsilon))
        {
          break;
        }
        pros::lcd::print(4,"xE %5.1f yE %5.1f tE %5.1f", x_drive_error, y_drive_error, drive_straight_error);

        //Calculates the derivatives(change in error)
        double y_derivative = y_last_error - y_drive_error;
        y_fourth_last_error = y_third_last_error;
        y_third_last_error = y_second_last_error;
        y_second_last_error = y_last_error;

        double x_derivative = x_last_error - x_drive_error;
        x_fourth_last_error = x_third_last_error;
        x_third_last_error = x_second_last_error;
        x_second_last_error = x_last_error;

        //Sets a new last error
        y_last_error = y_drive_error;
        x_last_error = x_drive_error;

        //calculates the overall change in errors over the last three frames of the loop
        double y_third_last_derivative = y_fourth_last_error - y_third_last_error;
        double y_second_last_derivative = y_third_last_error - y_second_last_error;
        double y_last_derivative = y_second_last_error - y_last_error;
        y_last_three_derivatives = y_last_derivative + y_second_last_derivative + y_third_last_derivative;

        double x_third_last_derivative = x_fourth_last_error - x_third_last_error;
        double x_second_last_derivative = x_third_last_error - x_second_last_error;
        double x_last_derivative = x_second_last_error - x_last_error;
        x_last_three_derivatives = x_last_derivative + x_second_last_derivative + x_third_last_derivative;

        //Calculate speed to be driven at using kp,ki,kd
        double y_speed = y_drive_error * y_drive_kp + y_derivative * y_drive_kd;
        double x_speed = x_drive_error * x_drive_kp + x_derivative * x_drive_kd;

        //prints the x speed and y speed to the terminal
        printf("req x_vel: %5.2f  req y_vel: %5.2f\n",x_speed,y_speed);

        //Removes impossible speeds by setting the speed down to a possible one
        if(y_speed > y_max_speed)
        {
            y_speed = y_max_speed;
        }

        if(y_speed < y_max_speed * -1)
        {
            y_speed = y_max_speed * -1;
        }

        if(x_speed > x_max_speed)
        {
            x_speed = x_max_speed;
        }

        if(x_speed < x_max_speed * -1)
        {
            x_speed = x_max_speed * -1;
        }


        //If the speed is lower than a certain value set the speeed to zero otherwise if the robot is slower than the minimum speed set the robot to the minimum speed
        if(fabs(y_speed) < zero_speed)
        {
          y_speed = 0.0;
        }
        else if(fabs(y_speed) < y_min_speed)
        {
            if(y_speed > 0)
            {
                y_speed = y_min_speed;
            }

            else if(y_speed < 0)
            {
                y_speed = y_min_speed * -1;
            }
        }

        //If the speed is lower than a certain value set the speeed to zero otherwise if the robot is slower than the minimum speed set the robot to the minimum speed
        if(fabs(x_speed) < zero_speed)
        {
          x_speed = 0.0;
        }
        else if(fabs(x_speed) < x_min_speed)
        {
            if(x_speed > 0)
            {
                x_speed = x_min_speed;
            }

            else if(x_speed < 0)
            {
                x_speed = x_min_speed * -1;
            }
        }


        // ************************************************************************************************* //
        // Set maximum accelleration to prevent the robot from jerking right or left at the start of driving //
        // ************************************************************************************************* //

        double velocity_adj = y_speed - y_last_speed;

        if(velocity_adj > maximum_vel_adj)
        {
            y_speed = y_last_speed + maximum_vel_adj;
        }

        if(velocity_adj < -maximum_vel_adj)
        {
            y_speed = y_last_speed + -maximum_vel_adj;
        }

        double x_velocity_adj = x_speed - x_last_speed;

        if(x_velocity_adj > maximum_vel_adj)
        {
            x_speed = x_last_speed + maximum_vel_adj;
        }

        if(x_velocity_adj < -maximum_vel_adj)
        {
            x_speed = x_last_speed + -maximum_vel_adj;
        }

        //assigns the last speeds our current speeds
        y_last_speed = y_speed;
        x_last_speed = x_speed;
        //prints the actual velocities to the terminal
        printf("act x_vel: %5.2f  act y_vel: %5.2f\n",x_speed,y_speed);

        //sets the initial turn speed to zero
        double turn_speed = 0.0;
        //if drive straight has been turned on (always)
        if(drive_straight == true)
        {
          //Gets the gyro's current value
          double drive_gyro_value = inertial_get_value();

          //Calculates the amount that the robot is off of its heading
          drive_straight_error = target_heading - drive_gyro_value;

          //Creates a turn speed for the robot using a proportional value
          turn_speed = drive_straight_error * drive_straight_kp;

          //If the speed is lower than a certain value set the speeed to zero otherwise if the robot is slower than the minimum speed set the robot to the minimum speed
          if(fabs(turn_speed) < zero_speed)
          {
            turn_speed = 0.0;
          }
          else if(fabs(turn_speed) < turn_min_speed)
          {
              if(turn_speed > 0)
              {
                  turn_speed = turn_min_speed;
              }

              else if(turn_speed < 0)
              {
                  turn_speed = turn_min_speed * -1;
              }
          }

          //prints the gyro value and turn speed to the terminal
          printf("Gyro: %5.1f  Turn Speed: %5.1f\n",drive_gyro_value,turn_speed);
          pros::lcd::print(5,"xS %5.1f yS %5.1f tS %5.1f", x_speed, y_speed, turn_speed);
        }

        // ******************************************* //
        // Set final speed and calculate the new error //
        // ******************************************* //

        //
        std::shared_ptr<ChassisModel> chassis_model = chassis->getModel();
        std::shared_ptr<XDriveModel> chassis_x_model = std::dynamic_pointer_cast<XDriveModel>(chassis_model);
        chassis_x_model->xArcade(x_speed, y_speed, turn_speed);
        pros::delay(33);

        //Calculates current position based on start position after small movement
        y_current_pos_value = get_y_position() - y_start_pos_value;
        x_current_pos_value = get_x_position() - x_start_pos_value;
    }

    //prints the end of the function to the terminal
    printf("end!\n");

    //Stops the robot from moving after the robot has reached its target distance
    chassis->getModel()->stop();
}

//function to change the maximum velocity adj.
double maximum_vel_adj_up()
{
  maximum_vel_adj = 1.0;
}

//function to set the maximum velocity adj back to what it originally was
double maximum_vel_adj_back()
{
  maximum_vel_adj = 0.1;
}

//makes the epsilon for turning higher
double allowable_errors_up()
{
  drive_straight_epsilon = 2.0;
}

//sets the epsilon for turning back to what it originally was
double allowable_errors_back()
{
  drive_straight_epsilon = 1.0;
}

//makes the epsilon for turning even higher
double allowable_errors_up_1()
{
  drive_straight_epsilon = 3.0;
}

//sets the epsilon for turning back to what it originally was
double allowable_errors_back_1()
{
  drive_straight_epsilon = 1.0;
}

//makes the epsilons
double allowable_errors_up_adj(double d_s_e, double x_e, double y_e)
{
  drive_straight_epsilon = d_s_e;
  x_distance_epsilon = x_e;
  y_distance_epsilon = y_e;
}

//sets the epsilons back to what it originally was
double allowable_errors_back_3()
{
  drive_straight_epsilon = 1.0;
  x_distance_epsilon = 1.0;
  y_distance_epsilon = 0.5;
}
