#pragma once

#include <cstdio>
#include <cstring>

#include "clock_interface.hpp"
#include "lifecycle.hpp"
#include "serial_interface.hpp"

class SerialCommandHandler
{
public:
  SerialCommandHandler(
    State * state,
    ClockInterface * clock,
    bool * commanded_gripper_state,
    StatusCallback inactive_control,
    StatusCallback calibrate_control,
    float tolerance,
    float joint_velocity,
    float joint_jerk)
  : state_(state),
    clock_(clock),
    commanded_gripper_state_(commanded_gripper_state),
    inactive_control_(inactive_control),
    calibrate_control_(calibrate_control),
    tolerance_(tolerance),
    joint_velocity_(joint_velocity),
    joint_jerk_(joint_jerk)
  {
    serial_buffer_[0] = '\0';
  }

  inline Error parse_serial(const char * message)
  {
    if (std::strncmp(message, "ACTIVATE", 8) == 0) {
      state_->reset(calibrate_control_);
      return Error::OK;
    }

    if (std::strncmp(message, "DEACTIVATE", 10) == 0) {
      state_->reset(inactive_control_);
      return Error::OK;
    }

    if (std::strncmp(message, "GRIPPER OPEN", 12) == 0) {
      if (is_transition_invalid()) {
        return Error::INVALID_TRANSITION;
      }

      *commanded_gripper_state_ = false;
      return Error::OK;
    }

    if (std::strncmp(message, "GRIPPER CLOSE", 13) == 0) {
      if (is_transition_invalid()) {
        return Error::INVALID_TRANSITION;
      }

      *commanded_gripper_state_ = true;
      return Error::OK;
    }

    if (std::strncmp(message, "SETPOINT", 8) == 0) {
      if (is_transition_invalid()) {
        return Error::INVALID_TRANSITION;
      }

      float q1;
      float q2;
      float z;
      if (std::sscanf(message, "SETPOINT %f %f %f", &q1, &q2, &z) == 3) {
        const unsigned long now_us = clock_->microseconds();
        state_->retarget_joints(q1, q2, joint_velocity_, joint_jerk_, now_us);
        replace_setpoint(q1, q2, z, tolerance_, joint_velocity_);
        return Error::OK;
      }

      return Error::INVALID_SETPOINT;
    }

    return Error::INVALID_SERIAL;
  }

  inline void get_serial(SerialInterface * serial)
  {
    while (serial->available() > 0) {
      const char incoming = serial->read();

      if (incoming == '\n') {
        serial_buffer_[serial_buffer_index_] = '\0';
        state_->error = parse_serial(serial_buffer_);
        serial_buffer_index_ = 0;
      } else if (serial_buffer_index_ < (k_buffer_size - 1)) {
        serial_buffer_[serial_buffer_index_++] = incoming;
      }
    }
  }

  inline void respond_serial(SerialInterface * serial, Status status) const
  {
    if (state_->error == Error::OK) {
      serial->println(processStatus(status));
      return;
    }

    serial->println(processError(state_->error));
  }

private:
  static constexpr int k_buffer_size = 64;

  inline bool is_transition_invalid() const
  {
    return state_->callback == inactive_control_ || state_->callback == calibrate_control_;
  }

  inline void replace_setpoint(
    float q1,
    float q2,
    float z,
    float tolerance,
    float velocity)
  {
    delete state_->setpoint;
    state_->setpoint = new Setpoint(q1, q2, z, tolerance, velocity);
  }

  State * state_;
  ClockInterface * clock_;
  bool * commanded_gripper_state_;
  StatusCallback inactive_control_;
  StatusCallback calibrate_control_;
  float tolerance_;
  float joint_velocity_;
  float joint_jerk_;
  char serial_buffer_[k_buffer_size];
  int serial_buffer_index_ = 0;
};
