#include <math.h>

#include <gtest/gtest.h>

#include "joint_trajectory.hpp"
#include "lifecycle.hpp"

namespace
{
TrajectoryState propagate_segment(
  const TrajectorySegment & segment,
  const float local_time_s)
{
  const float acceleration = segment.a0 + (segment.jerk * local_time_s);
  const float velocity =
    segment.v0 +
    (segment.a0 * local_time_s) +
    (0.5f * segment.jerk * local_time_s * local_time_s);
  const float position =
    segment.x0 +
    (segment.v0 * local_time_s) +
    (0.5f * segment.a0 * local_time_s * local_time_s) +
    ((1.0f / 6.0f) * segment.jerk * local_time_s * local_time_s * local_time_s);

  return {position, velocity, acceleration};
}

unsigned long seconds_to_microseconds(const float seconds)
{
  return static_cast<unsigned long>((seconds * 1000000.0f) + 0.5f);
}
}  // namespace

// Verifies long moves use the full five-segment jerk-limited profile and settle at the goal.
TEST(JointTrajectoryTest, GeneratesCruiseProfileWhenTargetVelocityIsFeasible)
{
  JointTrajectory trajectory;
  trajectory.initialize(0.0f, 1.0f, 0.5f, 1.0f, 0UL);

  ASSERT_EQ(trajectory.segment_count(), 5U);
  EXPECT_NEAR(trajectory.peak_velocity(), 0.5f, 1e-5f);
  EXPECT_GT(trajectory.cruise_time_s(), 0.0f);
  EXPECT_NEAR(trajectory.acceleration_time_s(), 2.0f * sqrtf(0.5f), 1e-5f);
  EXPECT_NEAR(trajectory.deceleration_time_s(), 2.0f * sqrtf(0.5f), 1e-5f);

  float previous_position = trajectory.sample_position(0UL);
  for (int step = 1; step <= 20; ++step) {
    const unsigned long sample_time_us = seconds_to_microseconds(
      (trajectory.total_time_s() * static_cast<float>(step)) / 20.0f);
    const float current_position = trajectory.sample_position(sample_time_us);
    EXPECT_LE(previous_position, current_position);
    previous_position = current_position;
  }

  const TrajectoryState final_state = trajectory.evaluate(
    seconds_to_microseconds(trajectory.total_time_s() + 0.1f));
  EXPECT_NEAR(final_state.x, 1.0f, 1e-4f);
  EXPECT_NEAR(final_state.v, 0.0f, 1e-5f);
  EXPECT_NEAR(final_state.a, 0.0f, 1e-5f);
  EXPECT_TRUE(trajectory.is_complete(seconds_to_microseconds(trajectory.total_time_s() + 0.1f)));
}

// Verifies the exact feasibility boundary produces the four-segment no-cruise profile.
TEST(JointTrajectoryTest, UsesNoCruiseProfileAtExactFeasibilityBoundary)
{
  const float target_velocity = 0.5f;
  const float max_jerk = 1.0f;
  const float displacement =
    (2.0f * target_velocity * sqrtf(target_velocity)) / sqrtf(max_jerk);

  JointTrajectory trajectory;
  trajectory.initialize(0.0f, displacement, target_velocity, max_jerk, 0UL);

  ASSERT_EQ(trajectory.segment_count(), 4U);
  EXPECT_NEAR(trajectory.cruise_time_s(), 0.0f, 1e-6f);
  EXPECT_NEAR(trajectory.peak_velocity(), target_velocity, 1e-5f);

  const TrajectoryState final_state = trajectory.evaluate(
    seconds_to_microseconds(trajectory.total_time_s() + 0.05f));
  EXPECT_NEAR(final_state.x, displacement, 1e-4f);
  EXPECT_NEAR(final_state.v, 0.0f, 1e-5f);
  EXPECT_NEAR(final_state.a, 0.0f, 1e-5f);
}

// Verifies short moves fall back to the reachable peak velocity instead of forcing the target velocity.
TEST(JointTrajectoryTest, FallsBackToReachablePeakVelocityWhenMoveIsTooShort)
{
  const float displacement = 0.1f;
  const float max_jerk = 1.0f;
  const float expected_peak_velocity = powf(
    ((displacement * sqrtf(max_jerk)) * 0.5f),
    2.0f / 3.0f);

  JointTrajectory trajectory;
  trajectory.initialize(0.0f, displacement, 1.0f, max_jerk, 0UL);

  ASSERT_EQ(trajectory.segment_count(), 4U);
  EXPECT_NEAR(trajectory.cruise_time_s(), 0.0f, 1e-6f);
  EXPECT_LT(trajectory.peak_velocity(), 1.0f);
  EXPECT_NEAR(trajectory.peak_velocity(), expected_peak_velocity, 1e-5f);

  const TrajectoryState final_state = trajectory.evaluate(
    seconds_to_microseconds(trajectory.total_time_s() + 0.05f));
  EXPECT_NEAR(final_state.x, displacement, 1e-4f);
  EXPECT_NEAR(final_state.v, 0.0f, 1e-5f);
  EXPECT_NEAR(final_state.a, 0.0f, 1e-5f);
}

// Verifies every segment boundary is C2-continuous in position, velocity, and acceleration.
TEST(JointTrajectoryTest, SegmentBoundariesPreservePositionVelocityAndAcceleration)
{
  JointTrajectory trajectory;
  trajectory.initialize(0.0f, 1.0f, 0.5f, 1.0f, 0UL);

  ASSERT_GT(trajectory.segment_count(), 1U);

  for (std::size_t index = 0U; index + 1U < trajectory.segment_count(); ++index) {
    const TrajectorySegment & current_segment = trajectory.segment(index);
    const TrajectorySegment & next_segment = trajectory.segment(index + 1U);
    const TrajectoryState end_state = propagate_segment(
      current_segment,
      current_segment.duration_s);

    EXPECT_NEAR(end_state.x, next_segment.x0, 1e-5f);
    EXPECT_NEAR(end_state.v, next_segment.v0, 1e-5f);
    EXPECT_NEAR(end_state.a, next_segment.a0, 1e-5f);

    const TrajectoryState boundary_state = trajectory.evaluate(
      seconds_to_microseconds(next_segment.start_time_s));
    EXPECT_NEAR(boundary_state.x, next_segment.x0, 1e-5f);
    EXPECT_NEAR(boundary_state.v, next_segment.v0, 1e-5f);
    EXPECT_NEAR(boundary_state.a, next_segment.a0, 1e-5f);
  }
}

// Verifies evaluation clamps cleanly before start time and after the end of the motion.
TEST(JointTrajectoryTest, EvaluateClampsToInitialAndFinalStates)
{
  JointTrajectory trajectory;
  trajectory.initialize(0.2f, 0.8f, 0.5f, 1.0f, 1000000UL);

  const TrajectoryState before_start = trajectory.evaluate(0UL);
  EXPECT_NEAR(before_start.x, 0.2f, 1e-5f);
  EXPECT_NEAR(before_start.v, 0.0f, 1e-5f);
  EXPECT_NEAR(before_start.a, 0.0f, 1e-5f);

  const unsigned long final_time_us =
    1000000UL + seconds_to_microseconds(trajectory.total_time_s() + 0.05f);
  const TrajectoryState after_end = trajectory.evaluate(final_time_us);
  EXPECT_NEAR(after_end.x, 0.8f, 1e-4f);
  EXPECT_NEAR(after_end.v, 0.0f, 1e-5f);
  EXPECT_NEAR(after_end.a, 0.0f, 1e-5f);
}

// Verifies a retargeted move restarts from the current commanded position without a position jump.
TEST(JointTrajectoryTest, RetargetingPreservesPositionContinuity)
{
  JointTrajectory first_trajectory;
  first_trajectory.initialize(0.0f, 1.0f, 0.5f, 1.0f, 0UL);

  const unsigned long retarget_time_us = 600000UL;
  const float retarget_position = first_trajectory.sample_position(retarget_time_us);

  JointTrajectory second_trajectory;
  second_trajectory.initialize(retarget_position, 0.3f, 0.5f, 1.0f, retarget_time_us);

  const TrajectoryState retarget_state = second_trajectory.evaluate(retarget_time_us);
  EXPECT_NEAR(retarget_state.x, retarget_position, 1e-5f);
  EXPECT_NEAR(retarget_state.v, 0.0f, 1e-5f);
  EXPECT_NEAR(retarget_state.a, 0.0f, 1e-5f);

  const TrajectoryState final_state = second_trajectory.evaluate(
    retarget_time_us + seconds_to_microseconds(second_trajectory.total_time_s() + 0.05f));
  EXPECT_NEAR(final_state.x, 0.3f, 1e-4f);
  EXPECT_NEAR(final_state.v, 0.0f, 1e-5f);
  EXPECT_NEAR(final_state.a, 0.0f, 1e-5f);
}

// Verifies state-level joint retargeting produces gradual commanded motion rather than a direct setpoint jump.
TEST(StateTrajectoryTest, CommandedTargetsMoveGraduallyAfterRetarget)
{
  State state;
  state.initialize_joint_trajectories(0.0f, 0.0f, 1.0f, 2.0f, 0UL);
  state.setpoint = new Setpoint(1.0f, -0.5f, 0.2f, 0.005f, 1.0f);

  state.retarget_joints(1.0f, -0.5f, 1.0f, 2.0f, 0UL);

  EXPECT_NEAR(state.commanded_q1(0UL), 0.0f, 1e-5f);
  EXPECT_NEAR(state.commanded_q2(0UL), 0.0f, 1e-5f);

  const float commanded_q1 = state.commanded_q1(100000UL);
  const float commanded_q2 = state.commanded_q2(100000UL);

  EXPECT_GT(commanded_q1, 0.0f);
  EXPECT_LT(commanded_q1, 1.0f);
  EXPECT_LT(commanded_q2, 0.0f);
  EXPECT_GT(commanded_q2, -0.5f);
  EXPECT_FLOAT_EQ(state.setpoint->z, 0.2f);

  state.reset([]() { return Status::COMPLETE; });
}

// Verifies commanded trajectories settle at their requested goals after the jerk-limited motion completes.
TEST(StateTrajectoryTest, CommandedTargetsReachGoalAfterTrajectoryCompletion)
{
  State state;
  state.initialize_joint_trajectories(0.0f, 0.0f, 1.0f, 2.0f, 0UL);
  state.retarget_joints(1.0f, -0.5f, 1.0f, 2.0f, 0UL);

  const float completion_time_s = fmaxf(
    state.joint1_trajectory.total_time_s(),
    state.joint2_trajectory.total_time_s()) + 0.1f;
  const unsigned long completion_time_us = seconds_to_microseconds(completion_time_s);

  EXPECT_NEAR(state.commanded_q1(completion_time_us), 1.0f, 1e-4f);
  EXPECT_NEAR(state.commanded_q2(completion_time_us), -0.5f, 1e-4f);
}

// Verifies trajectories can be initialized directly from calibrated joint positions to home targets.
TEST(StateTrajectoryTest, DirectTrajectoryInitializationStartsAtCurrentPositionAndConvergesHome)
{
  State state;
  state.setpoint = new Setpoint(2.08f, 1.10f, 0.05f, 0.005f, 1.0f);

  state.joint1_trajectory.initialize(1.6f, 2.08f, 1.0f, 2.0f, 0UL);
  state.joint2_trajectory.initialize(0.7f, 1.10f, 1.0f, 2.0f, 0UL);

  EXPECT_NEAR(state.commanded_q1(0UL), 1.6f, 1e-5f);
  EXPECT_NEAR(state.commanded_q2(0UL), 0.7f, 1e-5f);

  const float in_flight_q1 = state.commanded_q1(100000UL);
  const float in_flight_q2 = state.commanded_q2(100000UL);
  EXPECT_GT(in_flight_q1, 1.6f);
  EXPECT_LT(in_flight_q1, 2.08f);
  EXPECT_GT(in_flight_q2, 0.7f);
  EXPECT_LT(in_flight_q2, 1.10f);

  const float completion_time_s = fmaxf(
    state.joint1_trajectory.total_time_s(),
    state.joint2_trajectory.total_time_s()) + 0.1f;
  const unsigned long completion_time_us = seconds_to_microseconds(completion_time_s);

  EXPECT_NEAR(state.commanded_q1(completion_time_us), 2.08f, 1e-4f);
  EXPECT_NEAR(state.commanded_q2(completion_time_us), 1.10f, 1e-4f);

  state.reset([]() { return Status::COMPLETE; });
}
