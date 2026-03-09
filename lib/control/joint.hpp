#pragma once

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"
#include "lifecycle.hpp"

#define TOLERANCE 0.02f
#define RAD_PER_STEP 0.003926991f
#define JOINT_VEL 0.2f
#define PULSE_WIDTH_US 40UL

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder, ClockInterface* clock)
      : _stepper(stepper), _encoder(encoder), _clock(clock) {}

    inline Status pulse_if_required(float target_angle) {

      float error = target_angle - _read_position();

      if (error >= TOLERANCE) {
        _stepper->set_direction_forward();
      }
      else if (error <= -TOLERANCE)
      {
        _stepper->set_direction_backward();
      }
      else { return Status::COMPLETE; }

      unsigned long dt = _clock->microseconds() - last_pulse_time;

      if (JOINT_VEL <= RAD_PER_STEP / (dt * 1e-6))
      {
        _stepper->set_low();
        _clock->sleep(PULSE_WIDTH_US);
        _stepper->set_high();
      }

      return Status::ACTIVE;
    }
    
    inline float _read_position() {
      return _encoder->read_angle();
    }

  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;

    int rotations;
    bool pulse;
    unsigned long last_pulse_time;

    
};
