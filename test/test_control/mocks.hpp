// test/test_control/mocks.hpp
#include <gmock/gmock.h>
#include "encoder_interface.hpp"
#include "stepper_interface.hpp"

class MockStepper : public StepperInterface {
public:
    MOCK_METHOD(void, set_direction_forward, (), (override));
    MOCK_METHOD(void, set_direction_backward, (), (override));
    MOCK_METHOD(void, blocking_pulse, (unsigned long pulse_width), (override));
};

class MockEncoder : public EncoderInterface {
public:
    MOCK_METHOD(void, begin, (), (override));
    MOCK_METHOD(float, read_angle, (), (override));
};