#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "joint.hpp"
#include "mocks.hpp"

using ::testing::NiceMock;
using ::testing::Return;

class JointTest : public ::testing::Test
{
protected:
  NiceMock<MockStepper> mock_stepper;
  NiceMock<MockEncoder> mock_encoder;
  NiceMock<MockClock> mock_clock;
  Joint joint{&mock_stepper, &mock_encoder, &mock_clock};

  void begin_joint(unsigned long start_time_us = 0UL)
  {
    EXPECT_CALL(mock_encoder, begin());
    EXPECT_CALL(mock_stepper, set_high());
    EXPECT_CALL(mock_clock, microseconds()).WillOnce(Return(start_time_us));
    joint.begin();
  }

  void calibrate_joint(float encoder_angle)
  {
    EXPECT_CALL(mock_encoder, read_angle())
      .WillOnce(Return(encoder_angle))
      .WillOnce(Return(encoder_angle));
    joint.bad_calibrate();
  }
};

// Verifies startup primes the encoder and pulse line so later control-loop pulses start from a known state.
TEST_F(JointTest, BeginInitializesEncoderAndPulseTiming)
{
  begin_joint(42UL);
}

// Verifies calibration converts encoder readings into the expected home-aligned joint position.
TEST_F(JointTest, CalibrationSeedsHomeReferencedPosition)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));

  EXPECT_NEAR(joint._read_position(), 0.5f * k_pi, 1e-5f);
}

// Verifies positive position error commands the backward direction and emits a pulse once the period has elapsed.
TEST_F(JointTest, PulseIfRequiredCommandsBackwardStepWhenTargetIsAhead)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));
  EXPECT_CALL(mock_stepper, set_direction_backward());
  EXPECT_CALL(mock_clock, microseconds())
    .WillOnce(Return(1000000UL))
    .WillOnce(Return(1000020UL));
  EXPECT_CALL(mock_stepper, set_low());
  EXPECT_CALL(mock_clock, sleep(PULSE_WIDTH_US));
  EXPECT_CALL(mock_stepper, set_high());

  EXPECT_EQ(joint.pulse_if_required(2.0f, 0.01f, 1.0f), Status::ACTIVE);
}

// Verifies pulse timing uses joint-space radians per step after gearbox reduction.
TEST_F(JointTest, PulseIfRequiredUsesJointSpaceStepSizeForVelocityTiming)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));
  EXPECT_CALL(mock_stepper, set_direction_backward());
  EXPECT_CALL(mock_clock, microseconds())
    .WillOnce(Return(1000UL))
    .WillOnce(Return(1020UL));
  EXPECT_CALL(mock_stepper, set_low());
  EXPECT_CALL(mock_clock, sleep(PULSE_WIDTH_US));
  EXPECT_CALL(mock_stepper, set_high());

  EXPECT_EQ(joint.pulse_if_required(2.0f, 0.01f, 1.0f), Status::ACTIVE);
}

// Verifies negative position error flips the stepper direction without forcing an early pulse.
TEST_F(JointTest, PulseIfRequiredCommandsForwardStepWhenTargetIsBehind)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));
  EXPECT_CALL(mock_stepper, set_direction_forward());
  EXPECT_CALL(mock_clock, microseconds()).WillOnce(Return(100UL));

  EXPECT_EQ(joint.pulse_if_required(1.0f, 0.01f, 1.0f), Status::ACTIVE);
}

// Verifies zero commanded velocity falls back to a small positive rate so feedback correction can still pulse.
TEST_F(JointTest, PulseIfRequiredFallsBackWhenVelocityIsZero)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));
  EXPECT_CALL(mock_stepper, set_direction_backward());
  EXPECT_CALL(mock_clock, microseconds())
    .WillOnce(Return(5000UL))
    .WillOnce(Return(5020UL));
  EXPECT_CALL(mock_stepper, set_low());
  EXPECT_CALL(mock_clock, sleep(PULSE_WIDTH_US));
  EXPECT_CALL(mock_stepper, set_high());

  EXPECT_EQ(joint.pulse_if_required(2.0f, 0.01f, 0.0f), Status::ACTIVE);
}

// Verifies in-tolerance targets complete immediately so the control loop does not issue unnecessary step pulses.
TEST_F(JointTest, PulseIfRequiredCompletesWhenWithinTolerance)
{
  begin_joint();
  calibrate_joint(0.5f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.5f));

  EXPECT_EQ(
    joint.pulse_if_required((0.5f * k_pi) + 0.001f, 0.01f, 1.0f),
    Status::COMPLETE);
}

// Verifies encoder wraparound increments the inferred revolution count instead of producing a large discontinuity.
TEST_F(JointTest, ReadPositionTracksEncoderWraparoundAcrossRotations)
{
  begin_joint();
  calibrate_joint(6.0f);

  EXPECT_CALL(mock_encoder, read_angle()).WillOnce(Return(0.1f));

  const float expected_position = ((4.0f * k_pi) + 0.1f - (6.0f - (0.5f * k_pi))) / 5.0f;
  EXPECT_NEAR(joint._read_position(), expected_position, 1e-5f);
}
