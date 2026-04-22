#pragma once

#include <math.h>

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"
#include "lifecycle.hpp"

constexpr float k_rad_per_step = 0.0078539816339745;
constexpr unsigned long k_min_pulse_width = 10UL;
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
     * @param rollovers The number of rollovers the encoder has experienced at the calibration point
     * @param position The absolute position of the joint (in radians) at the calibration point
     */
    inline void calibrate(int rollovers, float position) {

      last_encoder_reading = _encoder->sample(50);
      this->rollovers = rollovers;

      float measured_position = (2.0*k_pi*rollovers + last_encoder_reading) / k_joint_gear_ratio;

      offset = position - measured_position;

    }

    /**
     * @brief Pulse the joint stepper motor once
     */
    inline void pulse_once() {
      read_position(); // ensure that the rollovers are updated
      _stepper->set_high();
      _clock->sleep(k_min_pulse_width);
      _stepper->set_low();
      _clock->sleep(k_min_pulse_width);
    }

    /**
     * @brief Transfer the stepper pulse edge
     */
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
        ? t - last_rising_edge
        : t - last_falling_edge;
      if(dt < k_min_pulse_width) { 
        return Status::ACTIVE;
      }

      // Check if within tolerance
      float current_position = read_position();
      float error = position - current_position;
      if(abs(error) <= tolerance) { 
        return Status::COMPLETE;
      } 
      
      // Set direction
      if (error < 0.0) {
        _stepper->set_direction_backward();
      } else {
        _stepper->set_direction_forward();
      }
      
      // Check if past allowed velocity
      float dt_s = dt*1e-6;
      float current_velocity = abs(k_joint_rad_per_step / dt_s) / 2.0f;
      if(current_velocity >= velocity) {
        return Status::ACTIVE;
      }

      transfer_edge();

      return Status::ACTIVE;
    }

    
    inline float read_position() {

      float current_encoder_reading = _encoder->read_angle();
      float delta = current_encoder_reading - last_encoder_reading;

      if (delta < -k_pi) { rollovers += 1; }
      else if (delta > k_pi) { rollovers -= 1; }

      last_encoder_reading = current_encoder_reading;
      const float encoder_angle = 2.0*k_pi*rollovers + current_encoder_reading;
      
      return (encoder_angle / k_joint_gear_ratio) + offset;
    }


  private:
    // Hardware interfaces
    StepperInterface* _stepper;
    EncoderInterface* _encoder;
    ClockInterface* _clock;

    /** @brief The number of rollovers the encoder has experienced */
    int rollovers = 0;
    /** @brief The joint-space calibration offset */
    float offset;
   
    // Edge tracking
    bool is_high = true;
    float last_encoder_reading;
    float last_position;
    unsigned long last_rising_edge = 0;
    unsigned long last_falling_edge = 0;

};
