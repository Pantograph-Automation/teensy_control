#pragma once

#include <math.h>

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"
#include "lifecycle.hpp"


constexpr float k_rad_per_step = 0.003926991f;
constexpr unsigned long k_min_pulse_width = 20UL;
constexpr float k_pi = 3.14159265358979323846f;
constexpr float k_joint_gear_ratio = 5.0f;
constexpr float k_joint_rad_per_step = k_rad_per_step / k_joint_gear_ratio;

class Joint {
  public:

    Joint(StepperInterface* stepper, EncoderInterface* encoder, ClockInterface* clock)
      : _stepper(stepper), _encoder(encoder), _clock(clock) {}

    /**
     * @brief Initialize the joint
     */
    inline void begin() {
      _encoder->begin();
      _stepper->set_high();

      unsigned long current_time = _clock->microseconds();
      last_falling_edge = current_time;
      last_rising_edge = current_time;
    }

    /**
     * @brief Calibrate the joint at a specified angle
     * @param rotations The number of rollovers the encoder has experienced at the calibration point
     * @param position The absolute position of the joint (in radians) at the calibration point
     */
    inline void calibrate(int rotations, float position) {

      last_encoder_reading = _encoder->sample(50);
      this->rotations = rotations;

      offset = 2*k_pi*rotations - last_encoder_reading*k_joint_gear_ratio - position;

    }

    /**
     * @brief Pulse the joint stepper motor once
     */
    inline void pulse_once() {
      _stepper->set_high();
      _clock->sleep(k_min_pulse_width);
      _stepper->set_low();
      _clock->sleep(k_min_pulse_width);
    }

    /**
     * @brief Transfer the pulse edge from high to low (or vice versa) if needed
     * @param angle The joint target angle
     * @param velocity The target joint velocity
     * @param tolerance The deadband tolerance for the joint
     * @details The stepper pulse pin should be normally high
     */
    inline Status pulse_edge(float position, float velocity, float tolerance) {

      // Check if edge transfer allowed
      unsigned long t = _clock->microseconds();
      unsigned long dt = is_high
        ? t - last_falling_edge
        : t - last_rising_edge;
      if(dt < k_min_pulse_width) { 
        return Status::ACTIVE;
      }
            
      // Check if within tolerance
      float current_position = read_position();
      if(abs(position - current_position) <= tolerance) { 
        return Status::COMPLETE;
      }
      
      // Check if past allowed velocity
      float current_velocity = abs((current_position - last_position) / dt);
      if(current_velocity >= velocity) {
        return Status::ACTIVE;
      }

      // Transfer edge
      if(is_high) {
        _stepper->set_low();
        last_falling_edge = t;      
      } else {
        _stepper->set_high();
        last_rising_edge = t;
      }

      return Status::ACTIVE;
    }
    
    inline float read_position() {
      
      float current_encoder_reading = _encoder->read_angle();

      float delta = current_encoder_reading - last_encoder_reading;

      if (delta < -k_pi) { rotations += 1; }
      else if (delta > k_pi) { rotations -= 1; }

      last_encoder_reading = current_encoder_reading;

      const float total_motor_angle =
        (rotations * 2.0f * k_pi) + current_encoder_reading  offset;
      return total_motor_angle / k_joint_gear_ratio;
    }

  private:
    // Hardware interfaces
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;

    // Positioning variables set at calibration
    int rotations = 0;
    float offset;
   
    // Edge tracking information
    bool is_high = true;
    float last_encoder_reading;
    float last_position;
    unsigned long last_rising_edge = 0;
    unsigned long last_falling_edge = 0;

};
