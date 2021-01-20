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

double maximum_vel_adj = 0.1;

double drive_straight_epsilon = 1;

double y_epsilon = 0.05;//5
double y_distance_epsilon = 0.5;
double x_epsilon = 0.05;//75
double x_distance_epsilon = 1.0;

//Drive X distance at Y speed
//void drive(double distance_in_inches, double max_speed)
//void gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, bool drive_straight, double kp, double ki, double kd)
void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed, double target_heading, double drive_straight_kp, double y_drive_kp, double x_drive_kp,  double turn_min_speed, bool drive_straight)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    y_max_speed = y_max_speed / 100;
    x_max_speed = x_max_speed / 100;

    y_min_speed = y_min_speed / 100;
    x_min_speed = x_min_speed / 100;

    //Sets the encoder units to use degrees instead of ticks
    chassis->getModel()->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    //const double y_drive_kp = 0.02;
    //const double y_drive_kp = 0.03;
    const double y_drive_kd = -0.001;
    //const double x_drive_kp = 0.04;
    //const double x_drive_kp = 0.05;
    const double x_drive_kd = -0.001;

    // const double y_epsilon = 0.05;//5
    // const double y_distance_epsilon = 0.5;
    // const double x_epsilon = 0.05;//75
    // const double x_distance_epsilon = 1.0;

    //const double drive_straight_kp = 0.010;
    // const double drive_straight_epsilon = 1;

    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double x_maximum_vel_adj = 0.2;

    const double zero_speed = 0.0075;
    //const double turn_min_speed = 0.12;

    //Converts Qlength distance to distance_in_inches
    double y_target_coordinate = y_distance.convert(inch);
    double x_target_coordinate = x_distance.convert(inch);
    //Draws starting position from the encoders (found in chassisController.cpp on github)
    double y_start_pos_value = get_y_position();
    double x_start_pos_value = get_x_position();
    //Calculates current position based on start position (found in chassisController.cpp on github)
    double y_current_pos_value = get_y_position() - y_start_pos_value;
    double x_current_pos_value = get_x_position() - x_start_pos_value;

    double y_relative_initial_drive_error = y_target_coordinate - get_y_position();
    double x_relative_initial_drive_error = x_target_coordinate - get_x_position();
    // double x_initial_drive_error = okapi::cos(get_heading()).getValue() * x_relative_initial_drive_error - okapi::sin(get_heading()).getValue() * x_relative_initial_drive_error;
    // double y_initial_drive_error = okapi::cos(get_heading()).getValue() * y_relative_initial_drive_error + okapi::sin(get_heading()).getValue() * y_relative_initial_drive_error;
    double initial_drive_error = sqrt((y_relative_initial_drive_error * y_relative_initial_drive_error) + (x_relative_initial_drive_error * x_relative_initial_drive_error));
    double total_drive_error = initial_drive_error;

    //Sets last error to zero before driving starts
    double y_last_error = 0.0;
    double x_last_error = 0.0;

    double y_second_last_error = 9999.9;
    double x_second_last_error = 9999.9;

    double y_third_last_error = 9999.9;
    double x_third_last_error = 9999.9;

    double y_fourth_last_error = 9999.9;
    double x_fourth_last_error = 9999.9;

    //Defines the initial drive error (found in chassisController.cpp on github)
    double y_drive_error = y_target_coordinate - get_y_position();
    double x_drive_error = x_target_coordinate - get_x_position();
    double drive_straight_error = target_heading - inertial_get_value();

    //Creates a variable that contains the initial gyro value (0)
    //inertial_reset();
    double initial_drive_gyro_value = -inertial_get_value();

    //Sets the first speed to zero
    double y_last_speed = 0.0;
    double x_last_speed = 0.0;

    double y_last_three_derivatives = 9999.9;
    double x_last_three_derivatives = 9999.9;



    //Drive while the robot hasn't reached its target distance
    // while ( ((fabs(y_last_three_derivatives) > y_epsilon) || (fabs(y_drive_error) > fabs(y_initial_drive_error) / 2.0)) ||
    //         ((fabs(x_last_three_derivatives) > x_epsilon) || (fabs(x_drive_error) > fabs(x_initial_drive_error) / 2.0)) ||
    //         ((fabs(drive_straight_error) > drive_straight_epsilon) || (fabs(drive_straight_error) > fabs(target_heading) / 2.0)) )

    while ( ((fabs(y_last_three_derivatives) > y_epsilon) || (fabs(total_drive_error) > fabs(initial_drive_error) / 2.0)) ||
            ((fabs(x_last_three_derivatives) > x_epsilon) || (fabs(total_drive_error) > fabs(initial_drive_error) / 2.0)) ||
            ((fabs(drive_straight_error) > drive_straight_epsilon * 3)) )

    //while (fabs(last_three_derivatives) > epsilon)
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        printf("tar x: %5.1f  tar y: %5.1f\n", x_target_coordinate, y_target_coordinate);
        printf("cur x: %5.1f  cur y: %5.1f  head: %5.2f\n", get_x_position(), get_y_position(), get_heading().convert(degree));
        printf("x_err: %5.1f  y_err: %5.1f\n",x_drive_error,y_drive_error);

        // //Calculate distance left to drive
        // y_drive_error = y_distance_in_inches - get_y_position();
        // x_drive_error = x_distance_in_inches - get_x_position();

        //https://gamedev.stackexchange.com/questions/79765/how-do-i-convert-from-the-global-coordinate-space-to-a-local-space
        double relativeX = x_target_coordinate - get_x_position();
        double relativeY = y_target_coordinate - get_y_position();
        x_drive_error = okapi::cos(get_heading()).getValue() * relativeX - okapi::sin(get_heading()).getValue() * relativeY;
        y_drive_error = okapi::cos(get_heading()).getValue() * relativeY + okapi::sin(get_heading()).getValue() * relativeX;
        total_drive_error = sqrt((x_drive_error * x_drive_error) + (y_drive_error * y_drive_error));

        //exit if we have reached the target
        if((fabs(y_drive_error) < y_distance_epsilon) && (fabs(x_drive_error) < x_distance_epsilon) && (fabs(drive_straight_error) < drive_straight_epsilon))
        {
          printf("Exit x and y drive error too small!\n");
          break;
        }


        //Calculates the derivative
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
        //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);
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



        //Removes impossible speeds by setting the speed down to a possible one
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
        //printf("rq2 x_vel: %5.2f  rq2 y_vel: %5.2f\n",x_speed,y_speed);



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

        if(x_velocity_adj > x_maximum_vel_adj)
        {
            x_speed = x_last_speed + x_maximum_vel_adj;
        }

        if(x_velocity_adj < -x_maximum_vel_adj)
        {
            x_speed = x_last_speed + -x_maximum_vel_adj;
        }

        y_last_speed = y_speed;
        x_last_speed = x_speed;
        printf("act x_vel: %5.2f  act y_vel: %5.2f\n",x_speed,y_speed);

        double turn_speed = 0.0;
        if(drive_straight == true)
        {
          //Gets the gyro's current value
          double drive_gyro_value = inertial_get_value();

          //Calculates the amount that the robot is off of its heading
          drive_straight_error = target_heading - drive_gyro_value;

          //Creates a turn speed so that different sides can be slowed down
          turn_speed = drive_straight_error * drive_straight_kp;

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

          printf("Gyro: %5.1f  Turn Speed: %5.1f\n",drive_gyro_value,turn_speed);
        }

        // ******************************************* //
        // Set final speed and calculate the new error //
        // ******************************************* //

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        // chassis->getModel()->arcade(speed, turn_speed);
        std::shared_ptr<ChassisModel> chassis_model = chassis->getModel();
        std::shared_ptr<XDriveModel> chassis_x_model = std::dynamic_pointer_cast<XDriveModel>(chassis_model);
        //chassis_x_model->xArcade(strafe_speed, speed, turn_speed);
        chassis_x_model->xArcade(x_speed, y_speed, turn_speed);
        pros::delay(10);

        //Calculates current position based on start position after small movement
        y_current_pos_value = get_y_position() - y_start_pos_value;
        x_current_pos_value = get_x_position() - x_start_pos_value;


        printf("ltd: %f epsilon: %f\n",y_last_three_derivatives,y_epsilon);
        printf("x_ltd: %f x_epsilon: %f\n",x_last_three_derivatives,x_epsilon);

    }

    //printf("end_y_pos: %f\n", get_y_position());
    printf("end_x_pos: %f\n", get_x_position());
    printf("end!\n");


    //Stops the robot from moving after the robot has reached its target distance
    chassis->getModel()->stop();
}

double allowable_errors_up()
{
  // x_distance_epsilon = 1.5;
  // y_distance_epsilon = 1.0;
  drive_straight_epsilon = 2.0;
}

double maximum_vel_adj_up()
{
  maximum_vel_adj = 1.0;
}

double maximum_vel_adj_back()
{
  maximum_vel_adj = 0.1;
}

double allowable_errors_back()
{
  // x_distance_epsilon = 1.0;
  // y_distance_epsilon = 0.5;
  drive_straight_epsilon = 1.0;
}
