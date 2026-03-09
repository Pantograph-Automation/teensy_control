#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "joint.hpp"
#include "mocks.hpp"

using ::testing::Return;
using ::testing::NiceMock;

class JointTest : public ::testing::Test {
  protected:
    NiceMock<MockStepper> mockStepper;
    NiceMock<MockEncoder> mockEncoder;
    NiceMock<MockClock>   mockClock;
    
    Joint* joint;

    void SetUp() override {
        joint = new Joint(&mockStepper, &mockEncoder, &mockClock);
    }

    void TearDown() override {
        delete joint;
    }
};

TEST_F(JointTest, TestPulseRequired) {
  EXPECT_CALL(mockEncoder, read_angle()).WillOnce(Return(10.0f));
  float target = 5.0f;
  
  EXPECT_CALL(mockStepper, set_direction_backward());
  EXPECT_CALL(mockClock, microseconds()).WillOnce(Return(1000));
  EXPECT_CALL(mockStepper, set_low());
  EXPECT_CALL(mockStepper, set_high());

  Status status = joint->pulse_if_required(target);
  
  EXPECT_EQ(status, Status::ACTIVE);
}