#pragma once

#include "hardware_interfaces.hpp"

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder)
      : _stepper(stepper), _encoder(encoder) {}


  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
};
