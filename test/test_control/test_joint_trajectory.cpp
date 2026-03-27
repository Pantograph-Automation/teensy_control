#include <gtest/gtest.h>

#include "joint_trajectory.hpp"
#include "lifecycle.hpp"

// Verifies long moves use the full trapezoidal profile and reach the goal monotonically.
TEST(JointTrajectoryTest, GeneratesTrapezoidalProfileForLongMove)
{
  JointTrajectory trajectory;
  trajectory.initialize(0.0f, 0.0f, 1.0f, 0.5f, 1.0f, 0);

  EXPECT_NEAR(trajectory.acceleration_time_s(), 0.5f, 1e-5f);
  EXPECT_NEAR(trajectory.cruise_time_s(), 1.5f, 1e-5f);
  EXPECT_NEAR(trajectory.deceleration_time_s(), 0.5f, 1e-5f);
  EXPECT_NEAR(trajectory.peak_velocity(), 0.5f, 1e-5f);

  const float p0 = trajectory.sample_position(0);
  const float p1 = trajectory.sample_position(250000);
  const float p2 = trajectory.sample_position(1000000);
  const float p3 = trajectory.sample_position(2250000);
  const float p4 = trajectory.sample_position(2500000);

  EXPECT_LE(p0, p1);
  EXPECT_LE(p1, p2);
  EXPECT_LE(p2, p3);
  EXPECT_LE(p3, p4);

  EXPECT_NEAR(trajectory.sample_velocity(250000), 0.25f, 1e-4f);
  EXPECT_NEAR(trajectory.sample_velocity(1000000), 0.5f, 1e-4f);
  EXPECT_NEAR(trajectory.sample_velocity(2250000), 0.25f, 1e-4f);
  EXPECT_NEAR(p4, 1.0f, 1e-4f);
  EXPECT_TRUE(trajectory.is_complete(2500000));
}

// Verifies short moves skip cruise time and fall back to a triangular velocity profile.
TEST(JointTrajectoryTest, FallsBackToTriangularProfileForShortMove)
{
  JointTrajectory trajectory;
  trajectory.initialize(0.0f, 0.0f, 0.1f, 1.0f, 1.0f, 0);

  EXPECT_NEAR(trajectory.cruise_time_s(), 0.0f, 1e-5f);
  EXPECT_LT(trajectory.peak_velocity(), 1.0f);
  EXPECT_GT(trajectory.acceleration_time_s(), 0.0f);
  EXPECT_GT(trajectory.deceleration_time_s(), 0.0f);
  EXPECT_NEAR(trajectory.sample_position(1000000), 0.1f, 1e-4f);
}

// Verifies retargeting starts exactly from the current commanded position instead of jumping to a new one.
TEST(JointTrajectoryTest, RetargetingPreservesPositionContinuity)
{
  JointTrajectory first_trajectory;
  first_trajectory.initialize(0.0f, 0.0f, 1.0f, 0.5f, 1.0f, 0);

  const unsigned long retarget_time_us = 500000;
  const float retarget_position = first_trajectory.sample_position(retarget_time_us);
  const float retarget_velocity = first_trajectory.sample_velocity(retarget_time_us);

  JointTrajectory second_trajectory;
  second_trajectory.initialize(
    retarget_position,
    retarget_velocity,
    0.3f,
    0.5f,
    1.0f,
    retarget_time_us);

  EXPECT_NEAR(
    second_trajectory.sample_position(retarget_time_us),
    retarget_position,
    1e-5f);
}

// Verifies state-level joint retargeting produces gradual commanded motion rather than a direct setpoint jump.
TEST(StateTrajectoryTest, CommandedTargetsMoveGraduallyAfterRetarget)
{
  State state;
  state.initialize_joint_trajectories(0.0f, 0.0f, 1.0f, 2.0f, 0);
  state.setpoint = new Setpoint(1.0f, -0.5f, 0.2f, 0.005f, 1.0f);

  state.retarget_joints(1.0f, -0.5f, 1.0f, 2.0f, 0);

  EXPECT_NEAR(state.commanded_q1(0), 0.0f, 1e-5f);
  EXPECT_NEAR(state.commanded_q2(0), 0.0f, 1e-5f);

  const float commanded_q1 = state.commanded_q1(100000);
  const float commanded_q2 = state.commanded_q2(100000);

  EXPECT_GT(commanded_q1, 0.0f);
  EXPECT_LT(commanded_q1, 1.0f);
  EXPECT_LT(commanded_q2, 0.0f);
  EXPECT_GT(commanded_q2, -0.5f);
  EXPECT_FLOAT_EQ(state.setpoint->z, 0.2f);

  state.reset([]() { return Status::COMPLETE; });
}

// Verifies commanded trajectories settle at their requested goals after the motion profile finishes.
TEST(StateTrajectoryTest, CommandedTargetsReachGoalAfterTrajectoryCompletion)
{
  State state;
  state.initialize_joint_trajectories(0.0f, 0.0f, 1.0f, 2.0f, 0);
  state.retarget_joints(1.0f, -0.5f, 1.0f, 2.0f, 0);

  EXPECT_NEAR(state.commanded_q1(2000000), 1.0f, 1e-4f);
  EXPECT_NEAR(state.commanded_q2(2000000), -0.5f, 1e-4f);
}
