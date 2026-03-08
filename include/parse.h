#include "Arduino.h"

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
    if (sscanf(message, "SETPOINT %f %f %f %f %d", 
      &setpoint.x, 
      &setpoint.x, 
      &setpoint.tolerance, 
      &setpoint.velocity,
      &setpoint.timeout) == 2) {
    } else {
      error = Error::INVALID_SETPOINT;
    }
  } else { error = Error::INVALID_SERIAL; }
}

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
