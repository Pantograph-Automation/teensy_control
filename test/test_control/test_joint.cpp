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
  EXPECT_CALL(mockEncoder, begin());
  EXPECT_CALL(mockClock, microseconds()).WillOnce(Return(0));
  joint->begin();

  EXPECT_CALL(mockEncoder, read_angle())
    .WillOnce(Return(0.5f))
    .WillOnce(Return(0.5f))
    .WillOnce(Return(0.5f));
  joint->bad_calibrate();

  EXPECT_CALL(mockStepper, set_direction_backward());
  EXPECT_CALL(mockClock, microseconds())
    .WillOnce(Return(1000000))
    .WillOnce(Return(1000020));
  EXPECT_CALL(mockStepper, set_low());
  EXPECT_CALL(mockStepper, set_high());
  EXPECT_CALL(mockClock, sleep(PULSE_WIDTH_US));

  Status status = joint->pulse_if_required(2.0f, 0.01f, 1.0f);
  
  EXPECT_EQ(status, Status::ACTIVE);
}
