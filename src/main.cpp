// usbipd list
// usbipd attach --wsl --busid 2-1

#include "lifecycle.hpp"

// Lifecycle variables
Status status = Status::INACTIVE;
Transition transition = Transition::NONE;
bool response_due = false;
static char serial_buffer[64]; // IO buffer
Setpoint setpoint; // Actively tracked setpoint
StatusCallback callback; // Active controller callback

#if defined(ARDUINO)

#include "Arduino.h"

Status activeControl() {
  delay(500);
  return Status::FINISHED;
}

Status calibrateControl() {
  delay(500);
  setpoint = home;
  return Status::CALIBRATED;
}

Status inactiveControl() {
  delay(100);
  return Status::INACTIVE;
}

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

#else

int main(int argc, char **argv) {
    // Ghost entry point
    return 0;
}

#endif