#include "okapi/api.hpp"
#include "inertial.hpp"
#include "encoders.hpp"
#include "utils.hpp"

using namespace okapi;


void tracker_initialize();
double get_x_position();
double get_y_position();
QAngle get_heading();
double reset_pos_x_first();
double reset_pos_y_first();
double reset_pos_x_second();
double reset_pos_y_second();
double reset_pos_x_third();
