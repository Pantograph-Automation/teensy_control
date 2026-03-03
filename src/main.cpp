#include <AS5600.h>
#include "config.h"

// usbipd list
// usbipd attach --wsl --busid 2-1

#define SERIAL_BAUD_RATE 115200


enum class Status
{
  INACTIVE,
  CALIBRATING,
  CALIBRATED,
  TRACKING,
  FINISHED
};
using Callback = Status (*)();

enum class Error
{
  OK,
  INVALID_SERIAL,
  INVALID_SETPOINT,
  CONTROL_TIMEOUT
};

enum class Transition
{
  NONE,
  ACTIVATE,
  DEACTIVATE
};

class Setpoint
{
  public:
    Setpoint::Setpoint(){};

    Setpoint::Setpoint(
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
    unsigned long int timeout;

};
auto home = Setpoint(0.0, 0.15, 0.01, 0.1, micros() + 30000);

// Lifecycle variables
Status status = Status::INACTIVE;
Transition transition = Transition::NONE;
Error error = Error::OK;
bool response_due = false;

// IO buffer
static char serial_buffer[64];

// Actively tracked setpoint
Setpoint setpoint;

// Active controller callback
Callback callback;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

  callback = inactiveControl;

}

void loop()
{
  
  readSerial();

  switch (transition)
  {
  case Transition::NONE:
    break;
  case Transition::ACTIVATE:
    callback = calibrateControl;
    break;
  case Transition::DEACTIVATE:
    callback = inactiveControl;
  }

  status = callback();

  if(response_due) { respondSerial(); }

}

//
// Serial interface functions
//

/**
 * @brief Parse the serial buffer iff a new message is available
 */
void readSerial()
{
  serial_buffer[0] = '\0';
  int index = 0;

  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n') { // Message is complete
        serial_buffer[index] = '\0'; // Null-terminate the string
        parseSerial(serial_buffer); 
        index = 0; // Reset for next message
    } 
    else if (index < 63) { // Avoid buffer overflow
        serial_buffer[index++] = incoming;
    }
    }
}

/**
 * @brief Parse an incoming serial message
 * @param message The incoming char buffer
 */
void parseSerial(const char* message)
{
  response_due = true;

  if (strncmp(message, "ACTIVATE", 8) == 0) {
    transition = Transition::ACTIVATE;
  }
  else if (strncmp(message, "DEACTIVATE", 10) == 0) {
    transition = Transition::DEACTIVATE;
    setpoint = Setpoint();
  }
  else if (strncmp(message, "SETPOINT", 8) == 0) {
    float x, y;
    if (sscanf(message, "SETPOINT %f %f", &x, &y) == 2) {
      setpoint.x = x; // Update the current setpoint
      setpoint.y = y;
    } else {
      error = Error::INVALID_SETPOINT;
    }
  } else { error = Error::INVALID_SERIAL; }
}

/**
 * @brief Send a response to the serial buffer based on the current state
 */
void respondSerial()
{
  if (error == Error::OK) {
    switch (status)
    {
    case Status::INACTIVE:
      Serial.println("OK INACTIVE");
      break;
    case Status::CALIBRATED:
      Serial.println("OK ACTIVE");
      break;
    case Status::FINISHED:
      Serial.println("OK FINISHED");
      break;
    default:
      Serial.println("ERROR Controller tried to respond while still in a transition state.");
      break;
    }
  }
  else {
    switch (error)
    {
    case Error::INVALID_SERIAL:
      Serial.println("ERROR Invalid serial message received.");
      break;
    case Error::INVALID_SETPOINT:
      Serial.println("ERROR Invalid setpoint received.");
      break;
    case Error::CONTROL_TIMEOUT:
      Serial.println("ERROR Control timed out. Collision likely!");
      break;
    }
  }

  response_due = false;
}

//  
// Control loop callbacks 
//

Status activeControl() {
  delay(500);
  response_due = true;
  return Status::FINISHED;
}

Status calibrateControl() {
  delay(500);
  response_due = true;
  setpoint = home;
  return Status::CALIBRATED;
}

Status inactiveControl() {
  delay(100);
  response_due = true;
  return Status::INACTIVE;
}