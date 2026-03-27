#pragma once

#include "joint_trajectory.hpp"

#define SERIAL_BAUD_RATE 115200

enum class Status
{
  ERROR,
  ACTIVE,
  COMPLETE
};
using StatusCallback = Status (*)();

enum class Error
{
  OK,
  INVALID_SERIAL,
  INVALID_SETPOINT,
  CONTROL_TIMEOUT,
  INVALID_TRANSITION
};


struct Setpoint
{
  Setpoint(
    float q1,
    float q2,
    float z,
    float tolerance,
    float velocity
  ) : q1(q1), q2(q2), z(z), tolerance(tolerance), velocity(velocity) {};

  float q1;
  float q2;
  float z;
  float tolerance;
  float velocity;
};

class State
{
  public:
    State() {};

    /** @brief The currently active control loop callback */
    StatusCallback callback = []() { return Status::ERROR; };

    /** @brief The currently active system setpoint */
    Setpoint* setpoint = nullptr;

    /** @brief Active trajectory generators for the rotary joints */
    JointTrajectory joint1_trajectory;
    JointTrajectory joint2_trajectory;

    /** @brief The currently active error, if any */
    Error error = Error::OK;

    /** @brief Whether or not a response is due to the serial interface */
    bool response_due = false;

    inline void reset(StatusCallback callback) {
      delete setpoint;
      setpoint = nullptr;
      error = Error::OK;
      this->callback = callback;
    }

    inline void initialize_joint_trajectories(
      const float q1,
      const float q2,
      const float velocity,
      const float acceleration,
      const unsigned long now_us)
    {
      joint1_trajectory.initialize(q1, 0.0f, q1, velocity, acceleration, now_us);
      joint2_trajectory.initialize(q2, 0.0f, q2, velocity, acceleration, now_us);
    }

    inline void retarget_joints(
      const float q1,
      const float q2,
      const float velocity,
      const float acceleration,
      const unsigned long now_us)
    {
      const float current_q1 = joint1_trajectory.sample_position(now_us);
      const float current_q2 = joint2_trajectory.sample_position(now_us);
      const float current_v1 = joint1_trajectory.sample_velocity(now_us);
      const float current_v2 = joint2_trajectory.sample_velocity(now_us);

      joint1_trajectory.initialize(current_q1, current_v1, q1, velocity, acceleration, now_us);
      joint2_trajectory.initialize(current_q2, current_v2, q2, velocity, acceleration, now_us);
    }

    inline float commanded_q1(const unsigned long now_us) const
    {
      return joint1_trajectory.sample_position(now_us);
    }

    inline float commanded_q2(const unsigned long now_us) const
    {
      return joint2_trajectory.sample_position(now_us);
    }

};

const char* processError(Error error)
{
  switch (error)
  {
    case Error::INVALID_SERIAL:
      return "ERROR Invalid serial message received.";
    case Error::INVALID_SETPOINT:
      return "ERROR Invalid setpoint received.";
    case Error::CONTROL_TIMEOUT:
      return "ERROR Control timed out. Collision likely!";
    case Error::INVALID_TRANSITION:
      return "ERROR Invalid transition. Recalibrate.";
    default:
      return "ERROR Unknown error occurred.";
  }
}

const char* processStatus(Status status) {
  switch (status)
  {
    case Status::ACTIVE:
      return "OK ACTIVE";
    case Status::COMPLETE:
      return "OK COMPLETE";
    case Status::ERROR:
      return "ERROR Received status in error state";
    default:
      return "ERROR Unknown status";
  }
}
