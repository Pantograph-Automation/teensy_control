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

TEST_F(JointTest, PullHighLargeError) {
  EXPECT_CALL(mockEncoder, read_angle()).WillOnce(Return(10.0f));
  float target = 5.0f;
  
  EXPECT_CALL(mockStepper, set_direction_forward());
  
  Status status = joint->pull_high(target);
  
  EXPECT_EQ(status, Status::ACTIVE);
}

TEST_F(JointTest, PullLowSetsStepperLowAfterPulseWidth) {
   
  EXPECT_CALL(mockClock, microseconds()).WillRepeatedly(Return(1000));
  EXPECT_CALL(mockEncoder, read_angle()).WillRepeatedly(Return(10.0f));

  joint->pull_high(5.0f); 

  EXPECT_CALL(mockClock, microseconds()).WillRepeatedly(Return(2000));

  EXPECT_CALL(mockStepper, set_low());

  joint->pull_low();
}