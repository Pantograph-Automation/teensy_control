#pragma once

#include "encoder_interface.hpp"
#include "stepper_interface.hpp"

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder)
      : _stepper(stepper), _encoder(encoder) {}


  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
};
