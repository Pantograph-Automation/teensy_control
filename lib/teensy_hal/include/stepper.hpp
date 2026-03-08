#pragma once

#include "Arduino.h"
#include "hardware_interfaces.hpp"

class Stepper : public StepperInterface
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

    };

    /**
     * @brief Set the stepper direction forward
     */
    inline void set_direction_forward() override
    {
      digitalWrite(direction_pin, HIGH);
    }

    /**
     * @brief Set the stepper direction backward
     */
    inline void set_direction_backward() override
    {
      digitalWrite(direction_pin, LOW);
    }

    /**
     * @brief Execute a blocking (non-realtime) pulse
     * @param pulse_width The width of the pulse, in microseconds
     */
    inline void blocking_pulse(const unsigned long pulse_width) override
    {
      digitalWrite(pulse_pin, HIGH);
      delayMicroseconds(pulse_width);
      digitalWrite(pulse_pin, LOW);
      delayMicroseconds(pulse_width);
    }

  private:

    // pin information
    int pulse_pin;
    int enable_pin;
    int direction_pin;

};