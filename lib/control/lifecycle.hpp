#pragma once

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

class State
{
  public:
    State() {};

    /** @brief The currently active control loop callback */
    StatusCallback callback = []() { return Status::ERROR; };

    /** @brief The currently active system setpoint */
    Setpoint* setpoint = nullptr;

    /** @brief The currently active error, if any */
    Error error = Error::OK;

    /** @brief Whether or not a response is due to the serial interface */
    bool response_due = false;

    /** @brief The status payload to send when the current command succeeds */
    Status pending_status = Status::COMPLETE;

    inline void reset(StatusCallback callback) {
      delete setpoint;
      setpoint = nullptr;
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
