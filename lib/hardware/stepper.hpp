#pragma once

#include "Arduino.h"
#include "stepper_interface.hpp"

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
     * @brief Write the pulse pin to high
     */
    inline void set_high() override
    {
      digitalWrite(pulse_pin, HIGH);
    }

    /**
     * @brief Write the pulse pin to low
     */
    inline void set_low() override
    {
      digitalWrite(pulse_pin, LOW);
    }

  private:

    // pin information
    int pulse_pin;
    int enable_pin;
    int direction_pin;

};