## Repository Purpose

This repository contains firmware and host-side tests for a Teensy 4.0 based motion
controller. The system:

- Receives motion and gripper setpoints over a serial interface.
- Executes those setpoints in a synchronized manner
- Drives two rotary joints and one linear stage
- Reports status and error responses back over serial.

The current firmware entry point is
[`src/main.cpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/src/main.cpp).

## Architecture Overview

### Runtime flow

1. `setup()` initializes serial and the stepper system
2. `loop()` checks for serial input, parses commands, and runs the neccessary open loop control command
3. Simultaneous motion is executed using the TeensyStep4 library

### Serial commands

The firmware currently recognizes commands such as:

- `ACTIVATE`
- `DEACTIVATE`
- `GRIPPER OPEN`
- `GRIPPER CLOSE`
- `GRIPPER LID`
- `GRIPPER DISH`
- `SETPOINT <q1> <q2> <z>`

Serial parsing and response handling live in
[`src/main.cpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/src/main.cpp).
