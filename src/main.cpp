// usbipd list
// usbipd attach --wsl --busid 2-1

#include "lifecycle.hpp"

State state;

#if defined(ARDUINO)

#include "Arduino.h"

Status activeControl() {
  delay(500);
  return Status::COMPLETE;
}

Status calibrateControl() {
  delay(500);
  return Status::COMPLETE;
}

Status inactiveControl() {
  delay(100);
  return Status::COMPLETE;
}

/**
 * @brief Parse an incoming serial message
 * @param message The incoming char buffer
 */
Error parseSerial(const char* message)
{

  if (strncmp(message, "ACTIVATE", 8) == 0) {
    state.reset(calibrateControl);
    return Error::OK;
  }

  if (strncmp(message, "DEACTIVATE", 10) == 0) {
    state.reset(inactiveControl);
    return Error::OK;
  }

  if (strncmp(message, "SETPOINT", 8) == 0) {

    if (state.callback == inactiveControl 
     || state.callback == calibrateControl ) { return Error::INVALID_TRANSITION; }

    float x, y, tolerance, velocity;
    int timeout;
    if (sscanf(message, "SETPOINT %f %f %f %f %d", 
      &x, &x, &tolerance, &velocity, &timeout) == 2) {
        state.setpoint = &Setpoint(x, y, tolerance, velocity, timeout);
        return Error::OK;
    } else {
      return Error::INVALID_SETPOINT;
    }
  }

  return Error::INVALID_SERIAL;
}


/**
 * @brief Get the serial buffer, assuming one is available
 */
void getSerial()
{
  static char serial_buffer[64];
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
 * @brief Respond with the appropriate serial message
 */
void respondSerial(Status status) {
  if (nullptr == state.error || *state.error == Error::OK)
  {
    Serial.println(processStatus(status));
  } else {
    Serial.println(processError(*state.error));
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

}

void loop()
{
  if (Serial.available()) {
    getSerial();
    state.response_due = true;
  }

  auto status = state.callback();
    

  if(state.response_due) {
    respondSerial(status);
    state.response_due = false;
  }

}

#else

int main(int argc, char **argv) {
    // Ghost entry point
    return 0;
}

#endif