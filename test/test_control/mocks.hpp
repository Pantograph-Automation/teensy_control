#pragma once

#include <gmock/gmock.h>

#include "clock_interface.hpp"
#include "encoder_interface.hpp"
#include "serial_interface.hpp"
#include "stepper_interface.hpp"

class MockStepper : public StepperInterface {
public:
  MOCK_METHOD(void, set_direction_forward, (), (override));
  MOCK_METHOD(void, set_direction_backward, (), (override));
  MOCK_METHOD(void, set_high, (), (override));
  MOCK_METHOD(void, set_low, (), (override));
};

class MockEncoder : public EncoderInterface {
public:
    MOCK_METHOD(void, begin, (), (override));
    MOCK_METHOD(float, read_angle, (), (override));
};

class MockClock : public ClockInterface {
  public:
    MOCK_METHOD(unsigned long, microseconds, (), (override));
    MOCK_METHOD(unsigned long, milliseconds, (), (override));
    MOCK_METHOD(void, sleep, (unsigned long), (override));
};

class MockSerial : public SerialInterface {
public:
  MOCK_METHOD(void, begin, (unsigned long), (override));
  MOCK_METHOD(int, available, (), (override));
  MOCK_METHOD(char, read, (), (override));
  MOCK_METHOD(void, println, (const char *), (override));
};
