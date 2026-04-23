# AGENTS.md

## Repository Purpose

This repository contains firmware and host-side tests for a Teensy 4.0 based motion
controller. The system:

- Receives motion and gripper setpoints over a serial interface.
- Executes those setpoints in a near-realtime control loop.
- Drives two rotary joints and one linear stage.
- Reports status and error responses back over serial.

The current firmware entry point is
[`src/main.cpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/src/main.cpp).

## Architecture Overview

### Runtime flow

1. `setup()` initializes serial, joints, and the gripper servo.
2. `loop()` checks for serial input, parses commands, and runs the active control
   callback every iteration.
3. The active callback is stored in `State::callback` and transitions between
   inactive, calibration, and active control behaviors.

### Serial commands

The firmware currently recognizes commands such as:

- `ACTIVATE`
- `DEACTIVATE`
- `GRIPPER OPEN`
- `GRIPPER CLOSE`
- `SETPOINT <q1> <q2> <z>`

Serial parsing and response handling live in
[`src/main.cpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/src/main.cpp).

## Repository Map

- [`src/main.cpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/src/main.cpp): Teensy firmware entry point, serial protocol handling, lifecycle transitions, and top-level control loop.
- [`lib/control/joint.hpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/lib/control/joint.hpp): Rotary joint control logic using encoder feedback and pulse timing.
- [`lib/control/stage.hpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/lib/control/stage.hpp): Linear stage pulse generation and position tracking.
- [`lib/control/lifecycle.hpp`](/home/dmalexa5/callus-transfer-group-c/teensy_control/lib/control/lifecycle.hpp): Shared state, status/error enums, setpoint model, and serial response strings.
- [`lib/interfaces/`](/home/dmalexa5/callus-transfer-group-c/teensy_control/lib/interfaces): Hardware abstraction interfaces for clock, encoder, and stepper implementations.
- [`lib/hardware/`](/home/dmalexa5/callus-transfer-group-c/teensy_control/lib/hardware): Concrete Teensy/Arduino-facing hardware adapters.
- [`test/`](/home/dmalexa5/callus-transfer-group-c/teensy_control/test): Native GoogleTest coverage for control logic.
- [`platformio.ini`](/home/dmalexa5/callus-transfer-group-c/teensy_control/platformio.ini): PlatformIO environments for native tests and Teensy firmware builds.

## Development Guidance

### Control-loop expectations

- Treat the main loop as near-realtime code.
- Prefer bounded work per iteration.
- Avoid blocking calls in the active control path unless hardware timing requires
  them for pulse generation.
- Keep serial parsing simple and deterministic.
- Be careful with heap allocation in loop-driven code paths. If changing setpoint
  ownership or lifecycle handling, favor predictable memory behavior.

### Safe change areas

- Adding or refining serial commands.
- Tightening state-machine transitions.
- Improving validation and error reporting.
- Expanding native test coverage around control behavior.
- Refactoring toward clearer interfaces while preserving timing-sensitive behavior.

### Areas that deserve extra caution

- Pulse timing constants and direction logic.
- Encoder wraparound and calibration logic.
- Any new delays, blocking I/O, or dynamic allocation inside hot paths.
- Changes that alter command semantics or status/error strings used by upstream
  tooling.

## Style Requirements

All C++ changes in this repository should follow ROS 2 style requirements as
closely as practical for embedded code. In particular:

- Follow ROS 2 naming and layout conventions for types, functions, files, and
  namespaces.
- Prefer `snake_case` for functions and variables, and use class/type naming that
  matches ROS 2 conventions used by the surrounding codebase.
- Keep headers self-contained with `#pragma once`.
- Prefer `constexpr`, typed constants, and enums over new preprocessor macros
  where practical.
- Prefer small, single-purpose functions and explicit, readable control flow.
- Add brief, high-signal comments only where behavior, timing, or hardware
  interaction is not obvious.

When touching older code that does not yet match these conventions, improve it
incrementally without causing unrelated formatting churn.

## Build and Test

Do not design, plan, build, or run tests. This is a prototyping projecto ONLY. Instead, provide reccomendations for immediate next steps for building and testing.

The `teensy40` environment builds the Arduino/Teensy firmware.

## Expectations for Future Agents

- Favor changes that improve determinism, safety, and testability.
- Verify behavior with native tests when logic changes are made.
- Keep docs and comments aligned with the actual serial protocol and lifecycle
  behavior.
