#include <gtest/gtest.h>

#include "lifecycle.hpp"
#include "trajectory.hpp"

namespace
{
unsigned long seconds_to_microseconds(const float seconds)
{
  return static_cast<unsigned long>((seconds * 1000000.0f) + 0.5f);
}
}  // namespace

// Verifies short moves produce a triangular profile that still settles at the target.
TEST(TrajectoryTest, ComputeMinimumTimeProfileUsesTriangularMoveForShortDistance)
{
  const TrapezoidProfile profile = compute_minimum_time_profile(
    0.0,
    0.1,
    {1.0, 2.0});

  EXPECT_TRUE(profile.triangular);
  EXPECT_DOUBLE_EQ(profile.t_cruise, 0.0);

  const TrajectorySample final_sample = sample_profile(profile, profile.t_total + 0.1);
  EXPECT_NEAR(final_sample.q, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(final_sample.v, 0.0);
  EXPECT_DOUBLE_EQ(final_sample.a, 0.0);
}

// Verifies longer moves use a full trapezoid with a non-zero cruise segment.
TEST(TrajectoryTest, ComputeMinimumTimeProfileUsesCruiseWhenDistanceIsLargeEnough)
{
  const TrapezoidProfile profile = compute_minimum_time_profile(
    0.0,
    1.0,
    {0.5, 1.0});

  EXPECT_FALSE(profile.triangular);
  EXPECT_GT(profile.t_cruise, 0.0);
  EXPECT_NEAR(profile.v_peak, 0.5, 1e-6);
}

// Verifies synchronized planning gives both rotary joints the same total duration.
TEST(TrajectoryTest, ComputeSynchronizedTrapezoidsAlignsAxisCompletionTimes)
{
  const SynchronizedTrapezoids2D synchronized_profiles = compute_synchronized_trapezoids_2d(
    0.0,
    1.0,
    0.0,
    0.2,
    {0.75, 1.5});

  EXPECT_NEAR(
    synchronized_profiles.axis1.t_total,
    synchronized_profiles.axis2.t_total,
    1e-9);
  EXPECT_NEAR(
    synchronized_profiles.total_time_s,
    synchronized_profiles.axis1.t_total,
    1e-9);
}

// Verifies sampling clamps at the start and end of the synchronized rotary move.
TEST(TrajectoryTest, SampleSynchronizedTrapezoidsClampsBeforeStartAndAfterEnd)
{
  const SynchronizedTrapezoids2D synchronized_profiles = compute_synchronized_trapezoids_2d(
    0.2,
    0.8,
    -0.4,
    0.1,
    {0.75, 1.5});

  const SynchronizedTrajectorySample2D before_start =
    sample_synchronized_trapezoids_2d(synchronized_profiles, -0.5);
  EXPECT_NEAR(before_start.axis1.q, 0.2, 1e-6);
  EXPECT_NEAR(before_start.axis2.q, -0.4, 1e-6);

  const SynchronizedTrajectorySample2D after_end =
    sample_synchronized_trapezoids_2d(
      synchronized_profiles,
      synchronized_profiles.total_time_s + 0.5);
  EXPECT_NEAR(after_end.axis1.q, 0.8, 1e-6);
  EXPECT_NEAR(after_end.axis2.q, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(after_end.axis1.v, 0.0);
  EXPECT_DOUBLE_EQ(after_end.axis2.v, 0.0);
}

// Verifies state replanning uses the current measured pose and clears the dirty flag.
TEST(StateTrajectoryTest, PlanningFromMeasuredPoseCreatesSynchronizedRotaryMotion)
{
  State state;
  state.replace_setpoint(1.0f, -0.5f, 0.2f, 0.005f, 1.0f);

  ASSERT_TRUE(state.setpoint_dirty);

  state.plan_rotary_trajectory(0.25f, -0.1f, 1.0f, 2.0f, 500000UL);

  EXPECT_FALSE(state.setpoint_dirty);
  EXPECT_EQ(state.trajectory_start_us, 500000UL);

  const RotaryWaypoint start_waypoint = state.sample_rotary_waypoint(500000UL);
  EXPECT_NEAR(start_waypoint.q1, 0.25f, 1e-5f);
  EXPECT_NEAR(start_waypoint.q2, -0.1f, 1e-5f);
  EXPECT_FLOAT_EQ(start_waypoint.v1, 0.0f);
  EXPECT_FLOAT_EQ(start_waypoint.v2, 0.0f);

  const RotaryWaypoint in_flight_waypoint = state.sample_rotary_waypoint(600000UL);
  EXPECT_GT(in_flight_waypoint.q1, 0.25f);
  EXPECT_LT(in_flight_waypoint.q1, 1.0f);
  EXPECT_LT(in_flight_waypoint.q2, -0.1f);
  EXPECT_GT(in_flight_waypoint.q2, -0.5f);

  const unsigned long completion_time_us =
    500000UL + seconds_to_microseconds(
    static_cast<float>(state.rotary_trajectory.total_time_s + 0.1));
  const RotaryWaypoint final_waypoint = state.sample_rotary_waypoint(completion_time_us);
  EXPECT_NEAR(final_waypoint.q1, 1.0f, 1e-4f);
  EXPECT_NEAR(final_waypoint.q2, -0.5f, 1e-4f);
  EXPECT_TRUE(state.rotary_trajectory_complete(completion_time_us));

  state.reset([]() { return Status::COMPLETE; });
}

// Verifies replacing a setpoint only marks the motion dirty until active control plans it.
TEST(StateTrajectoryTest, ReplaceSetpointMarksTrajectoryDirtyWithoutPlanningImmediately)
{
  State state;
  state.replace_setpoint(2.08f, 1.10f, 0.05f, 0.01f, 3.5f);

  ASSERT_NE(state.setpoint, nullptr);
  EXPECT_TRUE(state.setpoint_dirty);
  EXPECT_FLOAT_EQ(state.setpoint->q1, 2.08f);
  EXPECT_FLOAT_EQ(state.setpoint->q2, 1.10f);
  EXPECT_FLOAT_EQ(state.setpoint->z, 0.05f);
  EXPECT_DOUBLE_EQ(state.rotary_trajectory.total_time_s, 0.0);

  state.reset([]() { return Status::COMPLETE; });
}
