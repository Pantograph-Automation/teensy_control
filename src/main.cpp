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

constexpr float k_tolerance = 0.01f;
constexpr float k_joint_velocity = 3.5f;
constexpr float k_joint_acceleration = 100.0f;

Clock hw_clock;

#define PULSE1 22
#define DIR1 21
#define HOME_J1 1.10f
Encoder hw_encoder1(&Wire1);
Stepper hw_stepper1(PULSE1, DIR1);
Joint joint1(&hw_stepper1, &hw_encoder1, &hw_clock);

#define PULSE2 4
#define DIR2 5
#define HOME_J2 2.08f
HardwareSerialAdapter hw_serial;

Encoder hw_encoder2(&Wire);
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
  servo.write(15);
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
  state.replace_setpoint(q1, q2, z, tolerance, velocity);
}

Status activeControl() {
  const unsigned long now_us = hw_clock.microseconds();

  // if (state.setpoint == nullptr) {
  //   if (current_gripper_state != commanded_gripper_state) {
  //     if (commanded_gripper_state == false) { open_gripper(); }
  //     else { close_gripper(); }

  //     current_gripper_state = commanded_gripper_state;
  //   }

  //   return Status::COMPLETE;
  // }

  // const RotaryWaypoint waypoint = state.sample_rotary_waypoint(now_us);
  // const Status joint1_status =
  //   joint1.pulse_edge(waypoint.q1, fabs(waypoint.v1), state.setpoint->tolerance);
  // const Status joint2_status =
  //   joint2.pulse_edge(waypoint.q2, fabs(waypoint.v2), state.setpoint->tolerance);
  // const Status stage_status = linear_stage.pulse_if_required(state.setpoint->z);

  Serial.print(joint1.read_position(), 3);
  Serial.print("  ");
  Serial.print(joint2.read_position(), 3);
  Serial.println();

  if (current_gripper_state != commanded_gripper_state) {
    if (commanded_gripper_state == false) { open_gripper(); }
    else { close_gripper(); }

    current_gripper_state = commanded_gripper_state;
  }

  // const bool complete =
  //   state.rotary_trajectory_complete(now_us) &&
  //   joint1_status == Status::COMPLETE &&
  //   joint2_status == Status::COMPLETE &&
  //   stage_status == Status::COMPLETE;

  // return complete ? Status::COMPLETE : Status::ACTIVE;
  return Status::ACTIVE;
}

Status calibrateControl() {
  open_gripper();

  int CALIB_SPEED = 1500;

  while(true) {
  Serial.print(hw_encoder1.read_angle(), 3);
  Serial.print("  ");
  Serial.print(hw_encoder2.read_angle(), 3);
  Serial.println();
  }

  // Calibrate first arm
  while(digitalRead(19) == 0) {
    linear_stage.pulse_up_once();
    hw_clock.sleep((int)(CALIB_SPEED / 5));
  }

  hw_stepper1.set_direction_forward();
  hw_stepper2.set_direction_forward();
  while(digitalRead(14) == 0) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(CALIB_SPEED);
  }
  hw_stepper1.set_direction_backward();
  hw_stepper2.set_direction_backward();

  for(int i = 0; i < 739; i++) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(CALIB_SPEED);
  }
  joint2.bad_calibrate();
  hw_clock.sleep(CALIB_SPEED*10);
  Serial.print("Calibrated J2 at: ");
  Serial.println(joint2.read_position());

  // Now second arm
  while(digitalRead(10) == 0) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(CALIB_SPEED);
  }
  hw_stepper1.set_direction_forward();
  hw_stepper2.set_direction_forward();
  for(int i = 0; i < 739; i++) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(CALIB_SPEED);
  }
  joint1.bad_calibrate();
  hw_clock.sleep(CALIB_SPEED*10);
  Serial.print("Calibrated J1 at: ");
  Serial.println(joint1.read_position());
  

  replace_setpoint(HOME_J1, HOME_J2, HOME_Z, k_tolerance, k_joint_velocity);
  state.callback = activeControl;
  return Status::ACTIVE;
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
  k_tolerance,
  k_joint_velocity,
  k_joint_acceleration);

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
  hw_serial.begin(k_serial_baud_rate);
  while(!Serial);

  joint1.begin();
  joint2.begin();

  servo.attach(SERVO_PIN);

  open_gripper();

}

void loop()
{
  if (hw_serial.available()) {
    getSerial();
  }

  if (state.response_due) {
    respondSerial(state.pending_status);
    state.response_due = false;
    return;
  }

  auto status = state.callback();
  (void)status;
}

#else

int main(int argc, char **argv) {
    // Ghost entry point
    return 0;
}

#endif
