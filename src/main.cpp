// usbipd list
// usbipd attach --wsl --busid 2-1
#if defined(ARDUINO)

#include "Arduino.h"
#include "Servo.h"

#include "lifecycle.hpp"
#include "clock.hpp"
#include "encoder.hpp"
#include "serial.hpp"
#include "serial_command_handler.hpp"
#include "stepper.hpp"
#include "joint.hpp"
#include "stage.hpp"

#define TOLERANCE 0.005f
#define JOINT_VEL 1.0f
#define JOINT_ACCEL 2.0f

#define PULSE1 4
#define DIR1 5
#define HOME_J1 2.08f
Clock hw_clock;
HardwareSerialAdapter hw_serial;

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
  servo.write(45);
  delay(100);
}

void open_gripper () {
  servo.write(5);
  delay(100);
}

State state;

inline void replace_setpoint(
  const float q1,
  const float q2,
  const float z,
  const float tolerance,
  const float velocity)
{
  delete state.setpoint;
  state.setpoint = new Setpoint(q1, q2, z, tolerance, velocity);
}

Status activeControl() {
  const unsigned long now_us = hw_clock.microseconds();
  const float commanded_q1 = state.commanded_q1(now_us);
  const float commanded_q2 = state.commanded_q2(now_us);

  joint1.pulse_if_required(commanded_q1, state.setpoint->tolerance, state.setpoint->velocity);
  joint2.pulse_if_required(commanded_q2, state.setpoint->tolerance, state.setpoint->velocity);
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
  replace_setpoint(HOME_J1, HOME_J2, HOME_Z, TOLERANCE, JOINT_VEL);
  state.initialize_joint_trajectories(HOME_J1, HOME_J2, JOINT_VEL, JOINT_ACCEL, hw_clock.microseconds());
  state.callback = activeControl;
  return Status::COMPLETE;
}

Status inactiveControl() {
  return Status::COMPLETE;
}

SerialCommandHandler serial_command_handler(
  &state,
  &hw_clock,
  &commanded_gripper_state,
  inactiveControl,
  calibrateControl,
  TOLERANCE,
  JOINT_VEL,
  JOINT_ACCEL);

/**
 * @brief Parse an incoming serial message
 * @param message The incoming char buffer
 */
Error parseSerial(const char* message)
{
  return serial_command_handler.parse_serial(message);
}

/**
 * @brief Get the serial buffer, assuming one is available
 */
void getSerial()
{
  serial_command_handler.get_serial(&hw_serial);
}

/**
 * @brief Respond with the appropriate serial message
 */
void respondSerial(Status status) {
  serial_command_handler.respond_serial(&hw_serial, status);
}

void setup()
{
  hw_serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

  joint1.begin();
  joint2.begin();

  servo.attach(SERVO_PIN);

}

void loop()
{
  if (hw_serial.available()) {
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
