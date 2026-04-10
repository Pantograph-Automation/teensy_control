#pragma once

#include "trajectory.hpp"

constexpr unsigned long k_serial_baud_rate = 115200UL;

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

struct RotaryWaypoint
{
  float q1{0.0f};
  float q2{0.0f};
  float v1{0.0f};
  float v2{0.0f};
  float a1{0.0f};
  float a2{0.0f};
};

class State
{
  public:
    State() {};

    /** @brief The currently active control loop callback */
    StatusCallback callback = []() { return Status::ERROR; };

    /** @brief The currently active system setpoint */
    Setpoint* setpoint = nullptr;

    /** @brief Whether the active setpoint needs a new rotary trajectory plan */
    bool setpoint_dirty = false;

    /** @brief Active synchronized rotary plan */
    SynchronizedTrapezoids2D rotary_trajectory;

    /** @brief Start time of the active rotary plan */
    unsigned long trajectory_start_us = 0UL;

    /** @brief The currently active error, if any */
    Error error = Error::OK;

    /** @brief Whether or not a response is due to the serial interface */
    bool response_due = false;

    /** @brief The status payload to send when the current command succeeds */
    Status pending_status = Status::COMPLETE;

    inline void reset(StatusCallback callback) {
      delete setpoint;
      setpoint = nullptr;
      setpoint_dirty = false;
      rotary_trajectory = {};
      trajectory_start_us = 0UL;
      error = Error::OK;
      response_due = false;
      pending_status = Status::COMPLETE;
      this->callback = callback;
    }

    inline void replace_setpoint(
      const float q1,
      const float q2,
      const float z,
      const float tolerance,
      const float velocity)
    {
      delete setpoint;
      setpoint = new Setpoint(q1, q2, z, tolerance, velocity);
      setpoint_dirty = true;
    }

    inline void plan_rotary_trajectory(
      const float measured_q1,
      const float measured_q2,
      const float velocity,
      const float acceleration,
      const unsigned long now_us)
    {
      if (setpoint == nullptr) {
        rotary_trajectory = {};
        trajectory_start_us = now_us;
        setpoint_dirty = false;
        return;
      }

      rotary_trajectory = compute_synchronized_trapezoids_2d(
        measured_q1,
        setpoint->q1,
        measured_q2,
        setpoint->q2,
        {
          static_cast<double>(velocity),
          static_cast<double>(acceleration)});
      trajectory_start_us = now_us;
      setpoint_dirty = false;
    }

    inline RotaryWaypoint sample_rotary_waypoint(const unsigned long now_us) const
    {
      const double elapsed_time_s =
        (now_us <= trajectory_start_us) ?
        0.0 :
        static_cast<double>(now_us - trajectory_start_us) / 1000000.0;
      const SynchronizedTrajectorySample2D sample =
        sample_synchronized_trapezoids_2d(rotary_trajectory, elapsed_time_s);

      return {
        static_cast<float>(sample.axis1.q),
        static_cast<float>(sample.axis2.q),
        static_cast<float>(sample.axis1.v),
        static_cast<float>(sample.axis2.v),
        static_cast<float>(sample.axis1.a),
        static_cast<float>(sample.axis2.a)};
    }

    inline bool rotary_trajectory_complete(const unsigned long now_us) const
    {
      if (setpoint_dirty) {
        return false;
      }

      if (now_us <= trajectory_start_us) {
        return rotary_trajectory.total_time_s <= 0.0;
      }

      const double elapsed_time_s =
        static_cast<double>(now_us - trajectory_start_us) / 1000000.0;
      return elapsed_time_s >= rotary_trajectory.total_time_s;
    }

};

inline const char* processError(Error error)
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

inline const char* processStatus(Status status) {
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
