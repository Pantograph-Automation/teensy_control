// usbipd list
// usbipd attach --wsl --busid 2-1
#if defined(ARDUINO)

#include "Arduino.h"
#include "lifecycle.hpp"
#include "system.hpp"
#include "gripper.hpp"
#include <cstring>
#include <cstdio>

System system;
Setpoint* setpoint = nullptr;
Gripper gripper;
bool calibrated = false;

/**
 * @brief Calibrates the system
 * @warning This is a blocking function!!
 */
void calibrate() {

  // gripper.begin();

  // unsigned long k_calibration_delay = 800UL;
  // unsigned long k_z_calibration_delay = 800UL;

  // // Calibrate linear stage
  // while(digitalRead(11) != HIGH) {
  //   linear_stage.pulse_up_once();
  //   hw_clock.sleep(k_z_calibration_delay);
  // }
  // linear_stage.bad_calibrate(0.272);

  // // Calibrate joint 1
  // hw_stepper1.set_direction_backward();
  // hw_stepper2.set_direction_backward();
  // while(digitalRead(14) != HIGH) {
  //   joint1.pulse_once();
  //   joint2.pulse_once();
  //   hw_clock.sleep(k_calibration_delay);
  // }
  // joint1.calibrate(-1, -0.2617);
  // hw_clock.sleep(10000UL);

  // // Calibrate joint 2
  // hw_stepper1.set_direction_forward();
  // hw_stepper2.set_direction_forward();
  // while(digitalRead(10) != HIGH) {
  //   joint1.pulse_once();
  //   joint2.pulse_once();
  //   hw_clock.sleep(k_calibration_delay);
  // }
  // float joint2_pos = k_pi + 0.2617;
  // joint2.calibrate(3, joint2_pos);
  // hw_clock.sleep(10000UL);

  // // Set home setpoint
  // replace_setpoint(HOME_J1, HOME_J2, HOME_Z, k_tolerance, k_joint_velocity);

  return;
}

/**
 * @brief Deactivates the system
 * @details Placeholder for now
 */
void deactivate() {};

/**
 * @brief Parse an incoming serial message
 * @param message The incoming message to parse
 */
inline Error parse_serial(const char * message)
{
  if (std::strncmp(message, "ACTIVATE", 8) == 0) {
    calibrate();
    calibrated = true;
    return Error::OK;
  }

  if (std::strncmp(message, "DEACTIVATE", 10) == 0) {
    delete setpoint;
    deactivate();
    calibrated = false;
    return Error::OK;
  }

  if (std::strncmp(message, "GRIPPER OPEN", 12) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }
    gripper.open();
    return Error::OK;
  }

  if (std::strncmp(message, "GRIPPER CLOSE", 13) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }
    gripper.grip(Gripper::Width::CLOSE);
    return Error::OK;
  }

  if (std::strncmp(message, "SETPOINT", 8) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }

    float q1;
    float q2;
    float z;
    if (std::sscanf(message, "SETPOINT %f %f %f", &q1, &q2, &z) == 3) {
      setpoint = new Setpoint(q1, q2, z);
      system.move(setpoint);
      return Error::OK;
    }
    return Error::INVALID_SETPOINT;
  }
  return Error::INVALID_SERIAL;
}

inline void get_serial()
{
  const int k_buffer_size = 64;
  int serial_buffer_idx = 0;
  char serial_buffer_[k_buffer_size];

  while (Serial.available() > 0) {
    const char incoming = Serial.read();

    if (incoming == '\n') {
      serial_buffer_[serial_buffer_idx] = '\0';
      auto error = parse_serial(serial_buffer_);
      respond_serial(error);
      serial_buffer_idx = 0;
    } else if (serial_buffer_idx < (k_buffer_size - 1)) {
      serial_buffer_[serial_buffer_idx++] = incoming;
    }
  }
}

void setup()
{

  Serial.begin(k_serial_baud_rate);
  while(!Serial);

  system.begin();

}

void loop()
{
  // unsigned long start_time = micros();

  if (Serial.available()) {
    get_serial();
  }

}

#else

int main(int argc, char **argv) {
    // Ghost entry point
    return 0;
}

#endif
