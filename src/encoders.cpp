#include "okapi/api.hpp"
#include "encoders.hpp"
using namespace okapi;

ADIEncoder* shaft_enc_l = NULL;
ADIEncoder* shaft_enc_r = NULL;
ADIEncoder* shaft_enc_m = NULL;

void encoder_display_task(void* param)
{
  while(true)
  {
    pros::lcd::print(2,"Sensors Initialized");
    pros::lcd::print(3,"l %5.1f r %5.1f m %5.1f",shaft_enc_l->get(), shaft_enc_r->get(), shaft_enc_m->get());
    pros::delay(33);
  }
}

void encoder_initialize()
{
  //home
  // shaft_enc_l = new ADIEncoder(OPTICAL_SHAFT_ENCODER_LEFT_TOP, OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM, true);
  // shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, false);
  // shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, false);

  //school
  shaft_enc_l = new ADIEncoder(OPTICAL_SHAFT_ENCODER_LEFT_TOP, OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM, false);
  shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, true);
  //shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, true);
  shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, false);

  pros::Task encoder_display (encoder_display_task, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Encoder Display Task");

}
