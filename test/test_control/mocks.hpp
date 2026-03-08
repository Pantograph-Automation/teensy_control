// test/test_control/mocks.hpp
#include <gmock/gmock.h>
#include "hardware_interfaces.hpp"

class MockStepper : public IStepper {
public:
    MOCK_METHOD(void, set_direction_forward, (), (override));
    MOCK_METHOD(void, set_direction_backward, (), (override));
    MOCK_METHOD(void, blocking_pulse, (unsigned long pulse_width), (override));
};

class MockEncoder : public IEncoder {
public:
    MOCK_METHOD(void, begin, (unsigned long timeout), (override));
    MOCK_METHOD(float, read_angle, (), (override));
};