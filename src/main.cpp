// usbipd list
// usbipd attach --wsl --busid 2-1
#if defined(ARDUINO)

#include "Arduino.h"
#include "lifecycle.hpp"
#include "pantograph.hpp"
#include "gripper.hpp"
#include <cstring>
#include <cstdio>

// Limit switch pins
static constexpr int k_switch_pin_1 = 14;
static constexpr int k_switch_pin_2 = 10;
static constexpr int k_switch_pin_z = 11;

static constexpr float k_joint_calibration_velocity = k_pi / 2.0f; // rad per s
static constexpr float k_z_calibration_velocity = 0.05f; // meters per s

// Calibration values
static constexpr float k_j1_calibration_pos = -0.2617f;
static constexpr float k_j2_calibration_pos = k_pi + 0.2617f;
static constexpr float k_z_calibration_pos = 0.272f;

// Home position
const Setpoint home = Setpoint(0.5f * k_pi, 1.5f * k_pi, 0.15f);

Pantograph pantograph;
Setpoint* setpoint = nullptr;
Gripper gripper;
bool calibrated = false;

/**
 * @brief Calibrates the pantograph
 * @warning This is a blocking function!!
 */
void calibrate() {

  // Initialize the gripper
  gripper.begin();

  Serial.println("Rotating linear stage!");
  // Calibrate linear stage
  pantograph.rotate(
    0.0f,
    0.0f,
    k_z_calibration_velocity
  );
  while(digitalRead(k_switch_pin_z) != HIGH);
  pantograph.stop();
  pantograph.set_pos_z(k_z_calibration_pos);

  // Calibrate joint 1
  pantograph.rotate(
    -k_joint_calibration_velocity,
    -k_joint_calibration_velocity,
    0.0f
  );
  while(digitalRead(k_switch_pin_1) != HIGH);
  pantograph.stop();
  pantograph.set_pos_j1(k_j1_calibration_pos);

  // Calibrate joint 1
  pantograph.rotate(
    k_joint_calibration_velocity,
    k_joint_calibration_velocity,
    0.0f
  );
  while(digitalRead(k_switch_pin_2) != HIGH);
  pantograph.stop();
  pantograph.set_pos_j2(k_j2_calibration_pos);

  // Set home setpoint
  delete setpoint;
  setpoint = new Setpoint(home);

  return;
}

/**
 * @brief Deactivates the pantograph
 * @details Placeholder for now
 */
void deactivate() {};

/**
 * @brief Parse an incoming serial message
 * @param message The incoming message to parse
 */
inline Error parse_serial(const char * message)
{
  Serial.println("Parsing serial");
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

  if (std::strncmp(message, "GRIPPER LID", 11) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }
    gripper.grip(Gripper::Width::LID);
    return Error::OK;
  }

  if (std::strncmp(message, "GRIPPER DISH", 12) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }
    gripper.grip(Gripper::Width::DISH);
    return Error::OK;
  }

  if (std::strncmp(message, "SETPOINT", 8) == 0) {
    if (!calibrated) { return Error::INVALID_TRANSITION; }

    float q1;
    float q2;
    float z;
    if (std::sscanf(message, "SETPOINT %f %f %f", &q1, &q2, &z) == 3) {
      setpoint = new Setpoint(q1, q2, z);
      pantograph.move(setpoint);
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
  
  pinMode(k_switch_pin_1, INPUT_PULLUP);
  pinMode(k_switch_pin_2, INPUT_PULLUP);
  pinMode(k_switch_pin_z, INPUT_PULLUP);
  pantograph.begin();

  while(!Serial);
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
