#pragma once

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"
#include "lifecycle.hpp"


#define RAD_PER_STEP 0.003926991f
#define PULSE_WIDTH_US 20UL

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder, ClockInterface* clock)
      : _stepper(stepper), _encoder(encoder), _clock(clock) {}

    inline void begin() {
      _encoder->begin();
      _stepper->set_high();
      last_pulse_time = _clock->microseconds();
    }

    inline void bad_calibrate() {

      last_encoder_reading = 0.5*_encoder->read_angle() + 0.5*_encoder->read_angle();
   
      rotations = 1;
      offset = last_encoder_reading - (0.5*PI);

    }

    inline Status pulse_if_required(float target_angle, float tolerance, float joint_vel) {

      float error = target_angle - _read_position();

      if (error >= tolerance) {
        _stepper->set_direction_backward();
      }
      else if (error <= -tolerance)
      {
        _stepper->set_direction_forward();
      }
      else { return Status::COMPLETE; }

      unsigned long dt = _clock->microseconds() - last_pulse_time;

      unsigned long required_period_us = (unsigned long)((RAD_PER_STEP / joint_vel) * 1000000.0f);

      if (dt >= required_period_us)
      {
        _stepper->set_low();
        _clock->sleep(PULSE_WIDTH_US);
        _stepper->set_high();
        last_pulse_time = _clock->microseconds();
      }

      return Status::ACTIVE;
    }
    
    inline float _read_position() {
      
      float current_encoder_reading = _encoder->read_angle();
      float delta = current_encoder_reading - last_encoder_reading;

      if (delta < -PI) { rotations += 1; }
      else if (delta > PI) { rotations -= 1; }

      last_encoder_reading = current_encoder_reading;

      float total_motor_angle = (rotations * 2 * PI) + current_encoder_reading - offset;
      return total_motor_angle / 5;
    }

  private:
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;


    int rotations = 0;
    bool pulse = false;
    unsigned long last_pulse_time;
    float last_encoder_reading;
    float offset;

    
};
