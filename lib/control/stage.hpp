#pragma once

#include "stepper_interface.hpp"
#include "clock_interface.hpp"
#include "lifecycle.hpp"

#define STEPS_PER_METER 100000
#define Z_TOLERANCE 0.0002f
#define Z_VELOCITY 0.1f // meters per second
#define Z_MIN_PULSE_WIDTH 10UL 

class Stage {
  public:
    Stage(StepperInterface* stepper, ClockInterface* clock) : _stepper(stepper), _clock(clock) {};

    inline void bad_calibrate(float position) {
      last_position = position; //m
      current_position = position;
    }

    inline void transfer_edge() {
      unsigned long t = _clock->microseconds();

      if(is_high) {
        _stepper->set_low();
        last_falling_edge = t;
        is_high = false;   
      } else {
        _stepper->set_high();
        last_rising_edge = t;
        is_high = true;
      }
    }

    /**
     * @brief Translate the linear stage by a positive (up) or negative (down) distance
     * @param distance The distance to travel, in m
     */
    inline Status pulse_edge(float position) {

      // Check if edge transfer allowed
      unsigned long t = _clock->microseconds();
      unsigned long dt = is_high
        ? t - last_rising_edge
        : t - last_falling_edge;
      if(dt < Z_MIN_PULSE_WIDTH) { 
        return Status::ACTIVE;
      }

      // Check if within tolerance
      float error = position - current_position;
      if(abs(error) <= Z_TOLERANCE) { 
        return Status::COMPLETE;
      }
      
      // Check if past allowed velocity
      float dt_s = dt*1e-6;
      float current_velocity = abs((current_position - last_position)/ dt_s) / 2.0f;
      if(current_velocity >= Z_VELOCITY) {
        return Status::ACTIVE;
      }

      // Set direction and increment position counter
      last_position = current_position;
      const float k_meters_per_step = 1.0f / static_cast<float>(STEPS_PER_METER);
      if (error < 0.0) {
        _stepper->set_direction_backward();
        current_position -= k_meters_per_step/2.0f;
      } else {
        _stepper->set_direction_forward();
        current_position += k_meters_per_step/2.0f;
      }

      // Transfer edge
      transfer_edge();

      return Status::ACTIVE;
    }

    inline void pulse_up_once() {
      _stepper->set_direction_forward();

      _stepper->set_high();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
      _stepper->set_low();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
    }

    inline void pulse_down_once() {
      _stepper->set_direction_backward();

      _stepper->set_high();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
      _stepper->set_low();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
    }

    /**
     * @brief Returns the current position of the stage
     */
    inline float get_position() {
      return current_position;
    }

  private:
    StepperInterface* _stepper;
    ClockInterface* _clock;

    // Position tracking
    float last_position;
    float current_position;

    // Edge tracking
    bool is_high = true;
    float last_encoder_reading;
    unsigned long last_rising_edge = 0;
    unsigned long last_falling_edge = 0;

    inline void _pulse() {
      _stepper->set_high();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
      _stepper->set_low();
      _clock->sleep(Z_MIN_PULSE_WIDTH);
    }

};
