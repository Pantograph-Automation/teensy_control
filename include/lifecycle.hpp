#pragma once

#define SERIAL_BAUD_RATE 115200

enum class Status
{
  INACTIVE,
  CALIBRATING,
  CALIBRATED,
  TRACKING,
  FINISHED,
  ERROR
};
using StatusCallback = Status (*)();

enum class Error
{
  INVALID_SERIAL,
  INVALID_SETPOINT,
  CONTROL_TIMEOUT
};
using ErrorCallback = Error (*)();

enum class Transition
{
  NONE,
  ACTIVATE,
  DEACTIVATE
};
using TransitionCallback = Transition (*)();

struct Setpoint
{
  Setpoint(){};

  Setpoint(
    float x,
    float y,
    float tolerance,
    float velocity,
    unsigned long int timeout
  ) : x(x), y(y), tolerance(tolerance), velocity(velocity), timeout(timeout) {};

  float x;
  float y;
  float tolerance;
  float velocity;
  int timeout;
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
    default:
      return "ERROR Unknown error occurred.";
  }
}

const char* processStatus(Status status) {
    switch (status)
    {
    case Status::INACTIVE:
      return "OK INACTIVE";
    case Status::CALIBRATED:
      return "OK ACTIVE";
    case Status::FINISHED:
      return "OK FINISHED";
    default:
      return "ERROR Controller tried to respond while still in a transition state.";
    }
}