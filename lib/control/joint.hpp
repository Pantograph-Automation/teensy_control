#pragma once

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder, ClockInterface* clock)
      : _stepper(stepper), _encoder(encoder), _clock(clock) {}


    

  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;

    int rotations;

};
