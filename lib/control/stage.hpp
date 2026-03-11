#include "stepper_interface.hpp"
#include "clock_interface.hpp"
#include "lifecycle.hpp"

#define Z_PULSE_WIDTH_US 50UL
#define STEPS_PER_METER 400000
#define Z_TOLERANCE 0.0005f

class Stage {
  public:
    Stage(StepperInterface* stepper, ClockInterface* clock) : _stepper(stepper), _clock(clock) {};

    inline void bad_calibrate() {

      current_position = 0.2; //m

    }

    /**
     * @brief Translate the linear stage by a positive (up) or negative (down) distance
     * @param distance The distance to travel, in m
     */
    inline Status pulse_if_required(float height) {

      float error = height - current_position;

      if (error >= Z_TOLERANCE) {
        _stepper->set_direction_backward();
        current_position += 1.0 / (float) STEPS_PER_METER;
      }
      else if (error <= -Z_TOLERANCE)
      {
        _stepper->set_direction_forward();
        current_position -= 1.0 / (float) STEPS_PER_METER;
      }
      else { return Status::COMPLETE; }

      _pulse();

      return Status::ACTIVE;
      
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
    float current_position = 0.0; // m

    inline void _pulse() {
      _stepper->set_high();
      _clock->sleep(Z_PULSE_WIDTH_US);
      _stepper->set_low();
      _clock->sleep(Z_PULSE_WIDTH_US);
    }

};