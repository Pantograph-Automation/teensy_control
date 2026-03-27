#pragma once

#include <math.h>

class JointTrajectory
{
public:
  inline void initialize(
    const float start_position,
    const float start_velocity,
    const float goal_position,
    const float max_velocity,
    const float max_acceleration,
    const unsigned long start_time_us)
  {
    start_position_ = start_position;
    goal_position_ = goal_position;
    max_velocity_ = max_velocity;
    max_acceleration_ = max_acceleration;
    start_time_us_ = start_time_us;
    direction_ = 1.0f;
    active_ = false;

    if (goal_position_ < start_position_) {
      direction_ = -1.0f;
    }

    const float distance = fabsf(goal_position_ - start_position_);
    if (distance <= 0.0f || max_velocity_ <= 0.0f || max_acceleration_ <= 0.0f) {
      start_velocity_ = 0.0f;
      peak_velocity_ = 0.0f;
      acceleration_time_s_ = 0.0f;
      cruise_time_s_ = 0.0f;
      deceleration_time_s_ = 0.0f;
      acceleration_distance_ = 0.0f;
      cruise_distance_ = 0.0f;
      deceleration_distance_ = 0.0f;
      return;
    }

    float directed_start_velocity = start_velocity * direction_;
    if (directed_start_velocity < 0.0f) {
      directed_start_velocity = 0.0f;
    }
    if (directed_start_velocity > max_velocity_) {
      directed_start_velocity = max_velocity_;
    }

    start_velocity_ = directed_start_velocity;

    const float accel_distance =
      ((max_velocity_ * max_velocity_) - (start_velocity_ * start_velocity_)) /
      (2.0f * max_acceleration_);
    const float decel_distance =
      (max_velocity_ * max_velocity_) / (2.0f * max_acceleration_);

    if ((accel_distance + decel_distance) <= distance) {
      peak_velocity_ = max_velocity_;
      acceleration_distance_ = accel_distance;
      deceleration_distance_ = decel_distance;
      cruise_distance_ = distance - acceleration_distance_ - deceleration_distance_;
    } else {
      peak_velocity_ = sqrtf(
        (max_acceleration_ * distance) + (0.5f * start_velocity_ * start_velocity_));
      if (peak_velocity_ > max_velocity_) {
        peak_velocity_ = max_velocity_;
      }

      acceleration_distance_ =
        ((peak_velocity_ * peak_velocity_) - (start_velocity_ * start_velocity_)) /
        (2.0f * max_acceleration_);
      cruise_distance_ = 0.0f;
      deceleration_distance_ =
        (peak_velocity_ * peak_velocity_) / (2.0f * max_acceleration_);
    }

    acceleration_time_s_ = (peak_velocity_ - start_velocity_) / max_acceleration_;
    if (acceleration_time_s_ < 0.0f) {
      acceleration_time_s_ = 0.0f;
    }
    cruise_time_s_ = cruise_distance_ / peak_velocity_;
    deceleration_time_s_ = peak_velocity_ / max_acceleration_;
    active_ = true;
  }

  inline float sample_position(const unsigned long now_us) const
  {
    if (!active_) {
      return goal_position_;
    }

    const float elapsed_s = seconds_since_start(now_us);
    if (elapsed_s <= acceleration_time_s_) {
      return start_position_ + direction_ * (
        (start_velocity_ * elapsed_s) +
        (0.5f * max_acceleration_ * elapsed_s * elapsed_s));
    }

    const float cruise_start_s = acceleration_time_s_;
    const float cruise_end_s = cruise_start_s + cruise_time_s_;
    if (elapsed_s <= cruise_end_s) {
      const float cruise_elapsed_s = elapsed_s - cruise_start_s;
      return start_position_ + direction_ * (
        acceleration_distance_ + (peak_velocity_ * cruise_elapsed_s));
    }

    const float decel_end_s = cruise_end_s + deceleration_time_s_;
    if (elapsed_s <= decel_end_s) {
      const float decel_elapsed_s = elapsed_s - cruise_end_s;
      return start_position_ + direction_ * (
        acceleration_distance_ +
        cruise_distance_ +
        (peak_velocity_ * decel_elapsed_s) -
        (0.5f * max_acceleration_ * decel_elapsed_s * decel_elapsed_s));
    }

    return goal_position_;
  }

  inline float sample_velocity(const unsigned long now_us) const
  {
    if (!active_) {
      return 0.0f;
    }

    const float elapsed_s = seconds_since_start(now_us);
    if (elapsed_s <= acceleration_time_s_) {
      return direction_ * (start_velocity_ + (max_acceleration_ * elapsed_s));
    }

    const float cruise_end_s = acceleration_time_s_ + cruise_time_s_;
    if (elapsed_s <= cruise_end_s) {
      return direction_ * peak_velocity_;
    }

    const float decel_end_s = cruise_end_s + deceleration_time_s_;
    if (elapsed_s <= decel_end_s) {
      return direction_ * (peak_velocity_ - (max_acceleration_ * (elapsed_s - cruise_end_s)));
    }

    return 0.0f;
  }

  inline bool is_complete(const unsigned long now_us) const
  {
    if (!active_) {
      return true;
    }

    return seconds_since_start(now_us) >= (
      acceleration_time_s_ + cruise_time_s_ + deceleration_time_s_);
  }

  inline float goal_position() const { return goal_position_; }
  inline float acceleration_time_s() const { return acceleration_time_s_; }
  inline float cruise_time_s() const { return cruise_time_s_; }
  inline float deceleration_time_s() const { return deceleration_time_s_; }
  inline float peak_velocity() const { return peak_velocity_; }

private:
  inline float seconds_since_start(const unsigned long now_us) const
  {
    return static_cast<float>(now_us - start_time_us_) / 1000000.0f;
  }

  float start_position_ = 0.0f;
  float goal_position_ = 0.0f;
  float start_velocity_ = 0.0f;
  float max_velocity_ = 0.0f;
  float max_acceleration_ = 0.0f;
  float peak_velocity_ = 0.0f;
  float acceleration_time_s_ = 0.0f;
  float cruise_time_s_ = 0.0f;
  float deceleration_time_s_ = 0.0f;
  float acceleration_distance_ = 0.0f;
  float cruise_distance_ = 0.0f;
  float deceleration_distance_ = 0.0f;
  float direction_ = 1.0f;
  unsigned long start_time_us_ = 0;
  bool active_ = false;
};
