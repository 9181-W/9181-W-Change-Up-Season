#include "okapi/api.hpp"
#include "encoders.hpp"
using namespace okapi;

//creates the shaft encoders as empty objects
ADIEncoder* shaft_enc_l = NULL;
ADIEncoder* shaft_enc_r = NULL;
ADIEncoder* shaft_enc_m = NULL;

//creates a task to display the encoder values
void encoder_display_task(void* param)
{
  while(true)
  {
    pros::delay(10);

    //prints the data from the shaft encoders to the lcd screen
    printf("raw_left: %5.1f\n", shaft_enc_l->get());
    printf("raw_right: %5.1f\n", shaft_enc_r->get());
    printf("raw_middle: %5.1f\n", shaft_enc_m->get());
  }
}

//function to initialize the shaft encoders
void encoder_initialize()
{
  //creates the shaft encoders as objects
  shaft_enc_l = new ADIEncoder(OPTICAL_SHAFT_ENCODER_LEFT_TOP, OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM, false);
  shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, true);
  shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, false);

  //creates a task to display encoder values and make them continually drawable
  pros::Task encoder_display (encoder_display_task, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Encoder Display Task");

}
