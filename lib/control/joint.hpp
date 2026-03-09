#pragma once

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"
#include "lifecycle.hpp"

#define DEADBAND 0.02f
#define RAD_PER_STEP 0.003926991f
#define JOINT_VEL 0.2f
#define PULSE_WIDTH_US 40UL

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder, ClockInterface* clock)
      : _stepper(stepper), _encoder(encoder), _clock(clock) {}

    inline Status pull_high(float target_position) {

      float dt = _clock->microseconds() - last_edge_time;

      if (dt > PULSE_WIDTH_US) { _stepper->set_high(); }

      float error = _read_position() - target_position;
      
      if (error > DEADBAND && !pulse) {
        _stepper->set_direction_forward();
        pulse = true;
        last_edge_time = _clock->microseconds();
      }
      else if (error < -DEADBAND && !pulse) {
        _stepper->set_direction_backward();
        pulse = true;
        last_edge_time = _clock->microseconds();
      }
      else {
        return Status::COMPLETE;
      }
      return Status::ACTIVE;
    };

    inline void pull_low() {

      unsigned long dt = _clock->microseconds() - last_edge_time;
      float velocity = RAD_PER_STEP / (dt * 1e6);

      if (pulse && velocity <= JOINT_VEL && dt >= PULSE_WIDTH_US) {
        _stepper->set_low();
        last_edge_time = _clock->microseconds();
      }

    };
    

  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;

    int rotations;
    bool pulse;
    unsigned long last_edge_time;

    inline float _read_position() {
      return _encoder->read_angle();
    }
};
