#include "okapi/api.hpp"

#define RIGHT_REAR_WHEEL_PORT 20
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define LEFT_FRONT_WHEEL_PORT 1
#define LEFT_INTAKE_PORT 17
#define RIGHT_INTAKE_PORT 18
#define BOTTOM_PORT 19
#define TOP_PORT 21

extern void modified_initialize();

extern okapi::Controller master_controller;
extern okapi::Motor right_rear_mtr;
extern okapi::Motor left_rear_mtr;
extern okapi::Motor right_front_mtr;
extern okapi::Motor left_front_mtr;
extern okapi::Motor right_intake_mtr;
extern okapi::Motor left_intake_mtr;
extern okapi::Motor bottom_mtr;
extern okapi::Motor top_mtr;
