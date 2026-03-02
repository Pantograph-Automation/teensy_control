#pragma once

#include "Arduino.h"
#include "ring_buffer.hpp"

class Stepper
{
  public:

    /**
     * @param pulse_pin Pin used to pulse the stepper motor
     * @param enable_pin Pin used to enable the stepper motor
     * @param direction_pin Pin used to control the direction of the motor
     */
    Stepper(int pulse_pin, int enable_pin, int direction_pin) 
    {
      this->pulse_pin = pulse_pin;
      this->enable_pin = enable_pin;
      this->direction_pin = direction_pin;

      pinMode(pulse_pin, OUTPUT);
      pinMode(enable_pin, OUTPUT);
      pinMode(direction_pin, OUTPUT);

      time_buffer.fill(0UL);
      position_buffer.fill(0.0);
      velocity_buffer.fill(0.0);
      acceleration_buffer.fill(0.0);
    };

    // Basic utility methods
    void enable() {
      digitalWrite(enable_pin, HIGH);
    }

    void disable() {
      digitalWrite(enable_pin, LOW);
    }
    void set_direction_forward()
    {
      digitalWrite(direction_pin, HIGH);
    }

    void set_direction_backward()
    {
      digitalWrite(direction_pin, LOW);
    }

    inline unsigned long get_time(int index = 0) { return get_(time_buffer, index); }

    inline float get_position(int index = 0) { return get_(position_buffer, index); }

    inline float get_velocity(int index = 0) { return get_(velocity_buffer, index); }

    inline float get_acceleration(int index = 0) { return get_(acceleration_buffer, index); }

    /**
     * @brief Update state information with new position
     * @param new_position The current position of the stepper (read from encoder)
     */
    inline void update_state(const float new_position, const unsigned long time)
    {
      update_(position_buffer, new_position);
      update_(time_buffer, time);
      unsigned long time_delta = get_time_delta();

      // Update velocity
      float velocity = (get_position() - get_position(1)) / (float)(time_delta * 1e-6);
      update_(velocity_buffer, velocity);

      // Update acceleration
      float acceleration = (get_velocity() - get_velocity(1)) / (float)(time_delta * 1e-6);
      update_(acceleration_buffer, acceleration);
    }

    /**
     * @brief Compute the current time delta from last updated state
     */
    inline unsigned long get_time_delta()
    {
      return get_time() - get_time(1); // t - (t-1)
    }

    /**
     * @brief Checks if a step is required based on the estimated and commanded position
     * @param position_cmd The commanded stepper position
     * @param position_est The estimated stepper position
     * @details Place this method at the beginning of the control loop
     */
    inline void check_step_required(const float position_cmd, const float rad_per_step, float position_est)
    {
      float delta = position_cmd - position_est;

      if (!pulse_active) {
        if (delta >= rad_per_step) {
          set_direction_forward();
          digitalWrite(pulse_pin, HIGH);
          pulse_start = get_time();
          pulse_active = true;
          position_est += rad_per_step;
        } else if (delta <= -rad_per_step) {
          set_direction_backward();
          digitalWrite(pulse_pin, HIGH);
          pulse_start = get_time();
          pulse_active = true;
          position_est -= rad_per_step;
        }
      }
    };

    /**
     * @brief Pulses if a step is required AND enough time has passed to pulse
     * @param pulse_width The time between writing HIGH and LOW for a pulse
     * @details Place this method at the beginning of the control loop
     */
    inline void pulse_if_required(const unsigned long pulse_width)
    {
      if (pulse_active && (get_time() - pulse_start >= pulse_width)) {
        digitalWrite(pulse_pin, LOW);
        pulse_active = false;
      }
    }

  private:

    // pin information
    int pulse_pin;
    int enable_pin;
    int direction_pin;

    // Buffers that remember the states at t, t-1, and t-2
    Buffer<float> position_buffer{3};
    Buffer<float> velocity_buffer{3};
    Buffer<float> acceleration_buffer{3};
    Buffer<unsigned long> time_buffer{3}; // t, t-1, and t-2

    // Pulse housekeeping
    bool pulse_active;
    unsigned long pulse_start;

    /**
     * @brief Generic getter
     */
    template<typename T>
    inline T get_(Buffer<T>& buffer, int index)
    {
      return buffer.get(index);
    }

    /**
     * @brief Generic setter
     */
    template<typename T>
    inline void update_(Buffer<T>& buffer, const T& value)
    {
      buffer.pushOverwrite(value);
    }

};