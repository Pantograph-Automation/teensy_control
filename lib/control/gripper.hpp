#pragma once

#if defined(ARDUINO)
#include <Servo.h>

constexpr int k_servo_pin = 20;

class Gripper
{
  public:
    
    Gripper() = default;

    /**
     * @brief Width configuration for the gripper
     */
    enum class Width : uint16_t{
      UNKNOWN = 0,
      OPEN = 20,
      LID = 30,
      DISH = 35,
      CLOSE = 38,
    };

    /**
     * @brief Initialize the gripper
     * @param pin The servo pin
     */
    inline void begin(const int pin = k_servo_pin) {
      servo.attach(pin);
      open();
    }

    /**
     * @brief Move the gripper to a set configuration
     * @param width The with of the gripper
     */
    inline void grip(Width width) {
      
      // Ensure that the servo does not go to 0
      if (width == Width::UNKNOWN) { return; }

      // Write the servo position
      servo.write(static_cast<int>(width));

      // Update the current state
      position = width;
    };

    /**
     * @brief Open the gripper
     */
    inline void open() {
      grip(Width::OPEN);
    }

    /** @brief Read the current gripper width */
    inline Width get_position() const {
      return position;
    }

  private:

    /** @brief Hardware-dependant servo object */
    Servo servo;

    /** @brief Current position of the gripper */
    Width position = Width::UNKNOWN;

};

#endif