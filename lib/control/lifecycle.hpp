#pragma once

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
    Setpoint* setpoint;

    /** @brief The currently active error, if any */
    Error error = Error::OK;

    /** @brief Whether or not a response is due to the serial interface */
    bool response_due = false;

    inline void reset(StatusCallback callback) {
      setpoint = nullptr;
      error = Error::OK;
      this->callback = callback;
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