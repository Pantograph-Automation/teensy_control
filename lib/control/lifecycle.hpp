#pragma once
#if defined(ARDUINO)
#include "Arduino.h"

constexpr unsigned long k_serial_baud_rate = 115200UL;

/** @brief Error state of the overall system */
enum class Error
{
  OK,
  INVALID_SERIAL,
  INVALID_SETPOINT,
  CONTROL_TIMEOUT,
  INVALID_TRANSITION
};

/** @brief The currently tracked setpoint read fromt the serial interface */
struct Setpoint
{
  Setpoint(
    float q1,
    float q2,
    float z
  ) : q1(q1), q2(q2), z(z) {};

  /** @brief Joint 1 position */
  float q1;

  /** @brief Joint 2 position */
  float q2;

  /** @brief Z stage position */
  float z;
};

/**
 * @brief Process an error into a serial string
 * @param error The error to process
 */
inline const char* process_error(Error error)
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

/**
 * @brief Responds to the serial interface
 * @details Only called once motion is completed, so returns COMPLETE by default
 */
inline void respond_serial(Error error)
{
  if (error == Error::OK) {
    Serial.println("COMPLETE");
    return;
  }

  Serial.println(process_error(error));
}

#endif