#pragma once

#include <array>
#include <cstddef>
#include <math.h>

struct TrajectoryState
{
  float x = 0.0f;
  float v = 0.0f;
  float a = 0.0f;
};

struct TrajectorySegment
{
  float jerk = 0.0f;
  float duration_s = 0.0f;
  float start_time_s = 0.0f;
  float x0 = 0.0f;
  float v0 = 0.0f;
  float a0 = 0.0f;
};

class JointTrajectory
{
public:
  static constexpr std::size_t k_max_segments = 5U;

  /**
   * Build a symmetric jerk-limited rest-to-rest move over the requested
   * displacement.
   *
   * A requested target velocity v1 is feasible iff:
   *   dx >= 2 * v1^(3/2) / sqrt(J)
   *
   * When the displacement is too short, the reachable peak velocity falls back
   * to:
   *   vp = ((dx * sqrt(J)) / 2)^(2/3)
   */
  inline void initialize(
    const float start_position,
    const float goal_position,
    const float target_velocity,
    const float max_jerk,
    const unsigned long start_time_us)
  {
    goal_position_ = goal_position;
    start_time_us_ = start_time_us;
    peak_velocity_ = 0.0f;
    acceleration_time_s_ = 0.0f;
    cruise_time_s_ = 0.0f;
    deceleration_time_s_ = 0.0f;
    total_time_s_ = 0.0f;
    segment_count_ = 0U;

    const float displacement = goal_position - start_position;
    const float distance = fabsf(displacement);
    if (distance <= 0.0f || target_velocity <= 0.0f || max_jerk <= 0.0f) {
      clear_to_goal(goal_position);
      return;
    }

    const float direction = displacement >= 0.0f ? 1.0f : -1.0f;
    const float sqrt_jerk = sqrtf(max_jerk);
    const float x_min_target = (2.0f * target_velocity * sqrtf(target_velocity)) / sqrt_jerk;
    const float compare_tolerance =
      k_float_tolerance * fmaxf(1.0f, fmaxf(distance, x_min_target));

    float peak_velocity = target_velocity;
    float cruise_time_s = 0.0f;
    const bool has_cruise = distance > (x_min_target + compare_tolerance);

    if (has_cruise) {
      const float cruise_distance = distance - x_min_target;
      cruise_time_s = cruise_distance / peak_velocity;
    } else if (fabsf(distance - x_min_target) > compare_tolerance) {
      const float peak_velocity_base = (distance * sqrt_jerk) * 0.5f;
      peak_velocity = powf(peak_velocity_base, 2.0f / 3.0f);
    }

    const float tau = sqrtf(peak_velocity / max_jerk);
    peak_velocity_ = peak_velocity;
    acceleration_time_s_ = 2.0f * tau;
    cruise_time_s_ = cruise_time_s;
    deceleration_time_s_ = 2.0f * tau;
    initial_state_ = {start_position, 0.0f, 0.0f};
    final_state_ = initial_state_;

    float segment_start_time_s = 0.0f;
    TrajectoryState state = initial_state_;
    const float signed_jerk = direction * max_jerk;

    append_segment(signed_jerk, tau, segment_start_time_s, state);
    append_segment(-signed_jerk, tau, segment_start_time_s, state);
    append_segment(0.0f, cruise_time_s_, segment_start_time_s, state);
    append_segment(-signed_jerk, tau, segment_start_time_s, state);
    append_segment(signed_jerk, tau, segment_start_time_s, state);

    total_time_s_ = segment_start_time_s;
    final_state_ = state;
    final_state_.x = goal_position_;
    final_state_.v = 0.0f;
    final_state_.a = 0.0f;
  }

  inline TrajectoryState evaluate(const unsigned long now_us) const
  {
    if (segment_count_ == 0U) {
      return final_state_;
    }

    const float elapsed_s = seconds_since_start(now_us);
    if (elapsed_s <= 0.0f) {
      return initial_state_;
    }

    if (elapsed_s >= total_time_s_) {
      return final_state_;
    }

    for (std::size_t index = 0U; index < segment_count_; ++index) {
      const TrajectorySegment & segment = segments_[index];
      const float segment_end_time_s = segment.start_time_s + segment.duration_s;
      if (elapsed_s < segment_end_time_s || index == (segment_count_ - 1U)) {
        float local_time_s = elapsed_s - segment.start_time_s;
        if (local_time_s < 0.0f) {
          local_time_s = 0.0f;
        } else if (local_time_s > segment.duration_s) {
          local_time_s = segment.duration_s;
        }

        return propagate(
          {segment.x0, segment.v0, segment.a0},
          segment.jerk,
          local_time_s);
      }
    }

    return final_state_;
  }

  inline float sample_position(const unsigned long now_us) const
  {
    return evaluate(now_us).x;
  }

  inline float sample_velocity(const unsigned long now_us) const
  {
    return evaluate(now_us).v;
  }

  inline float sample_acceleration(const unsigned long now_us) const
  {
    return evaluate(now_us).a;
  }

  inline bool is_complete(const unsigned long now_us) const
  {
    return total_time_s_ <= 0.0f || seconds_since_start(now_us) >= total_time_s_;
  }

  inline float goal_position() const { return goal_position_; }
  inline float acceleration_time_s() const { return acceleration_time_s_; }
  inline float cruise_time_s() const { return cruise_time_s_; }
  inline float deceleration_time_s() const { return deceleration_time_s_; }
  inline float peak_velocity() const { return peak_velocity_; }
  inline float total_time_s() const { return total_time_s_; }
  inline std::size_t segment_count() const { return segment_count_; }
  inline const TrajectorySegment & segment(const std::size_t index) const { return segments_[index]; }

private:
  static constexpr float k_float_tolerance = 1e-5f;

  inline static TrajectoryState propagate(
    const TrajectoryState & start_state,
    const float jerk,
    const float time_s)
  {
    const float acceleration = start_state.a + (jerk * time_s);
    const float velocity =
      start_state.v +
      (start_state.a * time_s) +
      (0.5f * jerk * time_s * time_s);
    const float position =
      start_state.x +
      (start_state.v * time_s) +
      (0.5f * start_state.a * time_s * time_s) +
      ((1.0f / 6.0f) * jerk * time_s * time_s * time_s);

    return {position, velocity, acceleration};
  }

  inline void clear_to_goal(const float goal_position)
  {
    initial_state_ = {goal_position, 0.0f, 0.0f};
    final_state_ = initial_state_;
  }

  inline void append_segment(
    const float jerk,
    const float duration_s,
    float & segment_start_time_s,
    TrajectoryState & state)
  {
    if (duration_s <= 0.0f || segment_count_ >= k_max_segments) {
      return;
    }

    TrajectorySegment & segment = segments_[segment_count_];
    segment.jerk = jerk;
    segment.duration_s = duration_s;
    segment.start_time_s = segment_start_time_s;
    segment.x0 = state.x;
    segment.v0 = state.v;
    segment.a0 = state.a;

    state = propagate(state, jerk, duration_s);
    if (fabsf(state.v) < k_float_tolerance) {
      state.v = 0.0f;
    }
    if (fabsf(state.a) < k_float_tolerance) {
      state.a = 0.0f;
    }
    segment_start_time_s += duration_s;
    ++segment_count_;
  }

  inline float seconds_since_start(const unsigned long now_us) const
  {
    if (now_us <= start_time_us_) {
      return 0.0f;
    }

    return static_cast<float>(now_us - start_time_us_) / 1000000.0f;
  }

  std::array<TrajectorySegment, k_max_segments> segments_{};
  TrajectoryState initial_state_{};
  TrajectoryState final_state_{};
  float goal_position_ = 0.0f;
  float peak_velocity_ = 0.0f;
  float acceleration_time_s_ = 0.0f;
  float cruise_time_s_ = 0.0f;
  float deceleration_time_s_ = 0.0f;
  float total_time_s_ = 0.0f;
  unsigned long start_time_us_ = 0;
  std::size_t segment_count_ = 0U;
};
