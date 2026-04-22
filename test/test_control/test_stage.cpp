#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks.hpp"
#include "stage.hpp"

using ::testing::NiceMock;
using ::testing::Return;

class StageTest : public ::testing::Test
{
protected:
  NiceMock<MockStepper> mock_stepper;
  NiceMock<MockClock> mock_clock;
  Stage stage{&mock_stepper, &mock_clock};
};

// Verifies stage calibration restores the known home height used by the rest of the control flow.
TEST_F(StageTest, BadCalibrateSetsExpectedHomePosition)
{
  stage.bad_calibrate(0.02f);

  EXPECT_FLOAT_EQ(stage.get_position(), 0.02f);
}

// Verifies a positive height error commands the backward direction, pulses once, and advances by one step.
TEST_F(StageTest, PulseIfRequiredMovesStageUpwardByOneStep)
{
  stage.bad_calibrate(0.02f);

  EXPECT_CALL(mock_clock, microseconds()).WillOnce(Return(0UL));
  EXPECT_CALL(mock_stepper, set_direction_backward());
  EXPECT_CALL(mock_stepper, set_high());
  EXPECT_CALL(mock_clock, sleep(Z_MIN_PULSE_WIDTH)).Times(2);
  EXPECT_CALL(mock_stepper, set_low());

  EXPECT_EQ(stage.pulse_if_required(0.021f), Status::ACTIVE);
  EXPECT_NEAR(stage.get_position(), 0.02f + (1.0f / static_cast<float>(STEPS_PER_METER)), 1e-8f);
}

// Verifies a negative height error commands the forward direction and decrements the tracked stage position.
TEST_F(StageTest, PulseIfRequiredMovesStageDownwardByOneStep)
{
  stage.bad_calibrate(0.02f);

  EXPECT_CALL(mock_clock, microseconds()).WillOnce(Return(0UL));
  EXPECT_CALL(mock_stepper, set_direction_forward());
  EXPECT_CALL(mock_stepper, set_high());
  EXPECT_CALL(mock_clock, sleep(Z_MIN_PULSE_WIDTH)).Times(2);
  EXPECT_CALL(mock_stepper, set_low());

  EXPECT_EQ(stage.pulse_if_required(0.019f), Status::ACTIVE);
  EXPECT_NEAR(stage.get_position(), 0.02f - (1.0f / static_cast<float>(STEPS_PER_METER)), 1e-8f);
}

// Verifies requests inside the stage tolerance window complete without issuing a pulse.
TEST_F(StageTest, PulseIfRequiredCompletesWhenHeightIsWithinTolerance)
{
  stage.bad_calibrate(0.02f);

  EXPECT_EQ(stage.pulse_if_required(0.0202f), Status::COMPLETE);
  EXPECT_FLOAT_EQ(stage.get_position(), 0.02f);
}

// Verifies the stage does not emit a second step until the hardcoded Z velocity period has elapsed.
TEST_F(StageTest, PulseIfRequiredWaitsForVelocityLimitedStepPeriod)
{
  stage.bad_calibrate(0.02f);

  EXPECT_CALL(mock_clock, microseconds())
    .WillOnce(Return(0UL))
    .WillOnce(Return(k_stage_min_step_period_us - 1UL));
  EXPECT_CALL(mock_stepper, set_direction_backward()).Times(1);
  EXPECT_CALL(mock_stepper, set_high()).Times(1);
  EXPECT_CALL(mock_clock, sleep(Z_MIN_PULSE_WIDTH)).Times(2);
  EXPECT_CALL(mock_stepper, set_low()).Times(1);

  EXPECT_EQ(stage.pulse_if_required(0.03f), Status::ACTIVE);
  EXPECT_EQ(stage.pulse_if_required(0.03f), Status::ACTIVE);
  EXPECT_NEAR(stage.get_position(), 0.02f + (1.0f / static_cast<float>(STEPS_PER_METER)), 1e-8f);
}
