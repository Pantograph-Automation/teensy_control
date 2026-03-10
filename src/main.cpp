// usbipd list
// usbipd attach --wsl --busid 2-1
#if defined(ARDUINO)

#include "Arduino.h"
#include "lifecycle.hpp"
#include "clock.hpp"
#include "encoder.hpp"
#include "stepper.hpp"
#include "joint.hpp"

#define PULSE1 4
#define DIR1 5
Clock hw_clock;

Encoder hw_encoder1(&Wire1);
Stepper hw_stepper1(PULSE1, DIR1);
Joint joint1(&hw_stepper1, &hw_encoder1, &hw_clock);

#define PULSE2 6
#define DIR2 7
Encoder hw_encoder2(&Wire);
Stepper hw_stepper2(PULSE2, DIR2);
Joint joint2(&hw_stepper2, &hw_encoder2, &hw_clock);

State state;

Status activeControl() {
  
  Serial.print(joint1._read_position());
  Serial.print("  ");
  delay(5);
  Serial.println(joint2._read_position());
  delay(80);
  return Status::COMPLETE;
}

Status calibrateControl() {
  joint1.bad_calibrate();
  joint2.bad_calibrate();
  state.callback = activeControl;
  return Status::COMPLETE;
}

Status inactiveControl() {
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

    float q1, q2, tolerance, velocity;
    int timeout;
    if (sscanf(message, "SETPOINT %f %f %f %f %d", 
      &q1, &q2, &tolerance, &velocity, &timeout) == 2) {
        delete state.setpoint;
        state.setpoint = new Setpoint(q1, q2, tolerance, velocity, timeout);
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
        state.error = parseSerial(serial_buffer); 
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
  if (state.error == Error::OK)
  {
    Serial.println(processStatus(status));
  } else {
    Serial.println(processError(state.error));
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

  joint1.begin();
  joint2.begin();

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