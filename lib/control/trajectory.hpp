#pragma once

#include <algorithm>
#include <cmath>

struct TrajectoryLimits
{
  double max_velocity{0.0};
  double max_acceleration{0.0};
};

struct TrapezoidProfile
{
  double q0{0.0};
  double qf{0.0};
  double dq{0.0};
  double dir{1.0};

  double v_peak{0.0};
  double t_acc{0.0};
  double t_cruise{0.0};
  double t_total{0.0};
  double max_acceleration{0.0};

  bool triangular{false};
};

struct SynchronizedTrapezoids2D
{
  TrapezoidProfile axis1;
  TrapezoidProfile axis2;
  double total_time_s{0.0};
};

struct TrajectorySample
{
  double q{0.0};
  double v{0.0};
  double a{0.0};
};

struct SynchronizedTrajectorySample2D
{
  TrajectorySample axis1;
  TrajectorySample axis2;
};

inline TrapezoidProfile compute_minimum_time_profile(
  const double q0,
  const double qf,
  const TrajectoryLimits & limits)
{
  TrapezoidProfile profile;
  profile.q0 = q0;
  profile.qf = qf;
  profile.dq = qf - q0;
  profile.dir = (profile.dq >= 0.0) ? 1.0 : -1.0;
  profile.max_acceleration = std::max(0.0, limits.max_acceleration);

  const double distance = std::abs(profile.dq);
  const double max_velocity = std::max(0.0, limits.max_velocity);
  const double max_acceleration = profile.max_acceleration;

  if (distance <= 0.0 || max_velocity <= 0.0 || max_acceleration <= 0.0) {
    profile.v_peak = 0.0;
    profile.t_acc = 0.0;
    profile.t_cruise = 0.0;
    profile.t_total = 0.0;
    profile.triangular = true;
    return profile;
  }

  const double distance_for_full_trapezoid =
    (max_velocity * max_velocity) / max_acceleration;

  if (distance >= distance_for_full_trapezoid) {
    profile.triangular = false;
    profile.v_peak = max_velocity;
    profile.t_acc = max_velocity / max_acceleration;
    profile.t_cruise = (distance - distance_for_full_trapezoid) / max_velocity;
    profile.t_total = (2.0 * profile.t_acc) + profile.t_cruise;
  } else {
    profile.triangular = true;
    profile.v_peak = std::sqrt(max_acceleration * distance);
    profile.t_acc = profile.v_peak / max_acceleration;
    profile.t_cruise = 0.0;
    profile.t_total = 2.0 * profile.t_acc;
  }

  return profile;
}

inline TrapezoidProfile compute_synchronized_profile(
  const double q0,
  const double qf,
  const double requested_total_time_s,
  const TrajectoryLimits & limits)
{
  const TrapezoidProfile minimum_profile = compute_minimum_time_profile(q0, qf, limits);

  TrapezoidProfile profile;
  profile.q0 = q0;
  profile.qf = qf;
  profile.dq = qf - q0;
  profile.dir = (profile.dq >= 0.0) ? 1.0 : -1.0;
  profile.max_acceleration = std::max(0.0, limits.max_acceleration);

  const double distance = std::abs(profile.dq);
  const double max_velocity = std::max(0.0, limits.max_velocity);
  const double max_acceleration = profile.max_acceleration;
  const double total_time_s = std::max(requested_total_time_s, minimum_profile.t_total);

  if (distance <= 0.0 || total_time_s <= 0.0 || max_velocity <= 0.0 || max_acceleration <= 0.0) {
    profile.v_peak = 0.0;
    profile.t_acc = 0.0;
    profile.t_cruise = std::max(0.0, total_time_s);
    profile.t_total = std::max(0.0, total_time_s);
    profile.triangular = true;
    return profile;
  }

  const double discriminant =
    (max_acceleration * max_acceleration * total_time_s * total_time_s) -
    (4.0 * max_acceleration * distance);
  const double clamped_discriminant = std::max(0.0, discriminant);

  double v_peak =
    0.5 * ((max_acceleration * total_time_s) - std::sqrt(clamped_discriminant));
  v_peak = std::max(0.0, std::min(v_peak, max_velocity));

  profile.v_peak = v_peak;
  profile.t_acc = v_peak / max_acceleration;
  profile.t_cruise = std::max(0.0, total_time_s - (2.0 * profile.t_acc));
  profile.t_total = total_time_s;
  profile.triangular = (profile.t_cruise <= 1e-12);
  return profile;
}

inline SynchronizedTrapezoids2D compute_synchronized_trapezoids_2d(
  const double q1_0,
  const double q1_f,
  const double q2_0,
  const double q2_f,
  const TrajectoryLimits & limits)
{
  const TrapezoidProfile q1_min = compute_minimum_time_profile(q1_0, q1_f, limits);
  const TrapezoidProfile q2_min = compute_minimum_time_profile(q2_0, q2_f, limits);
  const double shared_total_time_s = std::max(q1_min.t_total, q2_min.t_total);

  SynchronizedTrapezoids2D profiles;
  profiles.total_time_s = shared_total_time_s;
  profiles.axis1 = compute_synchronized_profile(q1_0, q1_f, shared_total_time_s, limits);
  profiles.axis2 = compute_synchronized_profile(q2_0, q2_f, shared_total_time_s, limits);
  return profiles;
}

inline TrajectorySample sample_profile(const TrapezoidProfile & profile, const double time_s)
{
  TrajectorySample sample{};

  if (time_s <= 0.0 || profile.t_total <= 0.0) {
    sample.q = profile.q0;
    sample.v = 0.0;
    sample.a = 0.0;
    return sample;
  }

  if (time_s >= profile.t_total) {
    sample.q = profile.qf;
    sample.v = 0.0;
    sample.a = 0.0;
    return sample;
  }

  const double acceleration = profile.max_acceleration;
  const double direction = profile.dir;
  const double acceleration_distance = 0.5 * acceleration * profile.t_acc * profile.t_acc;

  if (time_s < profile.t_acc) {
    sample.q = profile.q0 + direction * (0.5 * acceleration * time_s * time_s);
    sample.v = direction * acceleration * time_s;
    sample.a = direction * acceleration;
    return sample;
  }

  if (time_s < profile.t_acc + profile.t_cruise) {
    sample.q = profile.q0 +
      direction * (acceleration_distance + (profile.v_peak * (time_s - profile.t_acc)));
    sample.v = direction * profile.v_peak;
    sample.a = 0.0;
    return sample;
  }

  const double remaining_time_s = profile.t_total - time_s;
  sample.q = profile.qf - direction * (0.5 * acceleration * remaining_time_s * remaining_time_s);
  sample.v = direction * acceleration * remaining_time_s;
  sample.a = -direction * acceleration;
  return sample;
}

inline SynchronizedTrajectorySample2D sample_synchronized_trapezoids_2d(
  const SynchronizedTrapezoids2D & profiles,
  const double time_s)
{
  return {
    sample_profile(profiles.axis1, time_s),
    sample_profile(profiles.axis2, time_s)};
}
