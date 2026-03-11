// usbipd list
// usbipd attach --wsl --busid 2-1
#if defined(ARDUINO)

#include "Arduino.h"
#include "Servo.h"

#include "lifecycle.hpp"
#include "clock.hpp"
#include "encoder.hpp"
#include "stepper.hpp"
#include "joint.hpp"
#include "stage.hpp"

#define TOLERANCE 0.005f
#define JOINT_VEL 1.0f

#define PULSE1 4
#define DIR1 5
#define HOME_J1 2.08f
Clock hw_clock;

Encoder hw_encoder1(&Wire);
Stepper hw_stepper1(PULSE1, DIR1);
Joint joint1(&hw_stepper1, &hw_encoder1, &hw_clock);

#define PULSE2 22 // 2 IS PULSE LINEAR RAIL
#define DIR2 21 // 3 IS DIR LINEAR RAIL
#define HOME_J2 1.10f
Encoder hw_encoder2(&Wire1);
Stepper hw_stepper2(PULSE2, DIR2);
Joint joint2(&hw_stepper2, &hw_encoder2, &hw_clock);

#define PULSE3 2
#define DIR3 3
#define HOME_Z 0.05f
Stepper hw_stepper3(PULSE3, DIR3);
Stage linear_stage(&hw_stepper3, &hw_clock);

#define SERVO_PIN 20
Servo servo;
bool current_gripper_state = false; // HACK: Tracking gripper with two bools??
bool commanded_gripper_state = false; // false is open, true is closed
void close_gripper () {
  servo.write(32);
  delay(100);
}

void open_gripper () {
  servo.write(10);
  delay(100);
}

State state;

Status activeControl() {
  
  joint1.pulse_if_required(state.setpoint->q1, state.setpoint->tolerance, state.setpoint->velocity);
  joint2.pulse_if_required(state.setpoint->q2, state.setpoint->tolerance, state.setpoint->velocity);
  linear_stage.pulse_if_required(state.setpoint->z);

  if (current_gripper_state != commanded_gripper_state) {
    if (commanded_gripper_state == false) { open_gripper(); }
    else { close_gripper(); }

    current_gripper_state = commanded_gripper_state;
  }

  return Status::ACTIVE;
}

Status calibrateControl() {
  joint1.bad_calibrate();
  joint2.bad_calibrate();
  linear_stage.bad_calibrate();
  delete state.setpoint;
  state.setpoint = new Setpoint(HOME_J1, HOME_J2, HOME_Z, TOLERANCE, JOINT_VEL);
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

  if (strncmp(message, "GRIPPER OPEN", 12) == 0) {

    if (state.callback == inactiveControl 
     || state.callback == calibrateControl ) { return Error::INVALID_TRANSITION; }


    commanded_gripper_state = false;
    return Error::OK;
  }

  if (strncmp(message, "GRIPPER CLOSE", 13) == 0) {

    if (state.callback == inactiveControl 
     || state.callback == calibrateControl ) { return Error::INVALID_TRANSITION; }

    commanded_gripper_state = true;
    return Error::OK;
  }

  if (strncmp(message, "SETPOINT", 8) == 0) {

    if (state.callback == inactiveControl 
     || state.callback == calibrateControl ) { return Error::INVALID_TRANSITION; }

    float q1, q2, z;
    if (sscanf(message, "SETPOINT %f %f %f", 
      &q1, &q2, &z) == 3) {

        delete state.setpoint;
        state.setpoint = new Setpoint(q1, q2, z, TOLERANCE, JOINT_VEL);
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

  servo.attach(SERVO_PIN);


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