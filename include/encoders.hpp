#include "okapi/api.hpp"
using namespace okapi;

//School
#define OPTICAL_SHAFT_ENCODER_LEFT_TOP 'G'
#define OPTICAL_SHAFT_ENCODER_LEFT_BOTTOM 'H'
#define OPTICAL_SHAFT_ENCODER_RIGHT_TOP 'E'
#define OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM 'F'
#define OPTICAL_SHAFT_ENCODER_MIDDLE_TOP 'C'
#define OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM 'D'

extern ADIEncoder* shaft_enc_l;
extern ADIEncoder* shaft_enc_r;
extern ADIEncoder* shaft_enc_m;

void encoder_initialize();
