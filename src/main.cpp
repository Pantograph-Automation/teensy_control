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

constexpr float k_tolerance = 0.035f;
constexpr float k_joint_velocity = 3.0 * k_pi / 4.0f;

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
#define HOME_Z 0.22f
Stepper hw_stepper3(PULSE3, DIR3);
Stage linear_stage(&hw_stepper3, &hw_clock);

#define SERVO_PIN 20
Servo servo;
bool current_gripper_state = false; // HACK: Tracking gripper with two bools??
bool commanded_gripper_state = false; // false is open, true is closed

void close_gripper () {
  servo.attach(SERVO_PIN);
  delay(10);
  servo.write(38);
  delay(100);
  current_gripper_state = true;
}

void open_gripper () {
  servo.write(20);
  delay(100);
  current_gripper_state = false;
  servo.detach();
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
  
  Status status1 = joint1.pulse_edge(state.setpoint->q1, state.setpoint->velocity, state.setpoint->tolerance);
  Status status2 = joint2.pulse_edge(state.setpoint->q2, state.setpoint->velocity, state.setpoint->tolerance);
  Status status3 = linear_stage.pulse_edge(state.setpoint->z);

  // Check gripper state, move if neccessary
  if (current_gripper_state != commanded_gripper_state) {
    if (commanded_gripper_state == false) { open_gripper(); }
    else { close_gripper(); }
  }

  // Return the status
  return (status1 == Status::ERROR || status2 == Status::ERROR || status3 == Status::ERROR)
  ? Status::ERROR
  : ((status1 == Status::ACTIVE || status2 == Status::ACTIVE || status3 == Status::ACTIVE)
    ? Status::ACTIVE
    : Status::COMPLETE);
}

Status calibrateControl() {

  servo.attach(SERVO_PIN);
  open_gripper();

  unsigned long k_calibration_delay = 800UL;
  unsigned long k_z_calibration_delay = 800UL;

  // Calibrate linear stage
  while(digitalRead(11) != HIGH) {
    linear_stage.pulse_up_once();
    hw_clock.sleep(k_z_calibration_delay);
  }
  linear_stage.bad_calibrate(0.272);

  // Calibrate joint 1
  hw_stepper1.set_direction_backward();
  hw_stepper2.set_direction_backward();
  while(digitalRead(14) != HIGH) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(k_calibration_delay);
  }
  joint1.calibrate(-1, -0.2617);
  hw_clock.sleep(10000UL);

  // Calibrate joint 2
  hw_stepper1.set_direction_forward();
  hw_stepper2.set_direction_forward();
  while(digitalRead(10) != HIGH) {
    joint1.pulse_once();
    joint2.pulse_once();
    hw_clock.sleep(k_calibration_delay);
  }
  float joint2_pos = k_pi + 0.2617;
  joint2.calibrate(3, joint2_pos);
  hw_clock.sleep(10000UL);

  // Set home setpoint
  replace_setpoint(HOME_J1, HOME_J2, HOME_Z, k_tolerance, k_joint_velocity);

  // Activate the control loop
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
  k_tolerance,
  k_joint_velocity);

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

  state.reset(inactiveControl);

  joint1.begin();
  joint2.begin();

}

void loop()
{
  // unsigned long start_time = micros();

  if (hw_serial.available()) {
    getSerial();
  }

  if (state.response_due && state.pending_status != Status::ACTIVE) {
    respondSerial(state.pending_status);
    state.response_due = false;
    return;
  }

  auto status = state.callback();
  
  state.pending_status = status;

}

#else

int main(int argc, char **argv) {
    // Ghost entry point
    return 0;
}

#endif
