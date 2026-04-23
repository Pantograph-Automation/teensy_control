#pragma once

#if defined(ARDUINO)
#undef PI
#include "teensystep4.h"
#include "lifecycle.hpp"

// Hardware configuration constants
static constexpr int k_step_pin_1 = 22;
static constexpr int k_dir_pin_1 = 21;
static constexpr float k_home_1 = 1.10f; // rad
static constexpr int k_step_pin_2 = 4;
static constexpr int k_dir_pin_2 = 5;
static constexpr float k_home_2 = 2.08f; // rad
static constexpr int k_step_pin_z = 2;
static constexpr int k_dir_pin_z = 3;
static constexpr float k_home_z = 0.22f; // m

// Motion configuration constants
static constexpr float k_pi =  3.14159265359f;
static constexpr float k_gear_ratio = 5.0f;
static constexpr float k_joint_steps_per_rev = 800.0f * k_gear_ratio;
static constexpr float k_joint_steps_per_rad = k_joint_steps_per_rev / (2.0f*k_pi);
static constexpr float k_z_steps_per_rev = 400.0f;
static constexpr float k_z_meters_per_rev = 0.004f;
static constexpr float k_z_steps_per_meter = k_z_steps_per_rev / k_z_meters_per_rev;

// Motion limiting constants
static constexpr float k_joint_velocity = 1.25f * k_pi; // rad per second
static constexpr float k_joint_acceleration = 3.0f*k_pi; // rad per second^2
static constexpr float k_z_velocity = 0.05f; // meters per second
static constexpr float k_z_acceleration = 0.1f; // meters per second^2

// Step limiting constants
static constexpr int32_t k_joint_step_speed = k_joint_velocity * k_joint_steps_per_rad; // steps per second
static constexpr int32_t k_joint_step_accel = k_joint_acceleration * k_joint_steps_per_rad; // steps per second^2
static constexpr int32_t k_z_step_speed = k_z_velocity * k_z_steps_per_meter; // meters per second
static constexpr int32_t k_z_step_accel = k_z_acceleration * k_z_steps_per_meter; // meters per second^2

// Calibration values
static constexpr float k_joint_calibration_velocity = k_pi / 4.0f; // rad per s
static constexpr float k_z_calibration_velocity = 0.02f; // meters per s
static constexpr float k_j1_calibration_pos = -0.2617f;
static constexpr float k_j2_calibration_pos = k_pi + 0.2617f;
static constexpr float k_z_calibration_pos = 0.272f;

using namespace TS4;

/** @brief The full Pantograph controlled by the Teensy 4 */

class Pantograph {
  public:

    Pantograph() 
    : stepper_1(k_step_pin_1, k_dir_pin_1),
      stepper_2(k_step_pin_2, k_dir_pin_2),
      stepper_z(k_step_pin_z, k_dir_pin_z),
      joint_group({stepper_1, stepper_2}) {};


    /**
     * @brief Initialize the stepper motors and TS4 namespace
     */
    inline void begin() {
      TS4::begin();

      stepper_1
        .setMaxSpeed(k_joint_step_speed)
        .setAcceleration(k_joint_step_accel);

      stepper_2
        .setMaxSpeed(k_joint_step_speed)
        .setAcceleration(k_joint_step_accel);
      
      stepper_z
        .setMaxSpeed(k_z_step_speed)
        .setAcceleration(k_z_step_accel);
    }

    /**
     * @brief Move the steppers syncronously to a new setpoint
     * @param setpoint The setpoint to track
     */
    inline void move(Setpoint* setpoint) {
      
      stepper_1.setTargetAbs(
        (int32_t)((k_j1_calibration_pos - setpoint->q1) * k_joint_steps_per_rad));

      stepper_2.setTargetAbs(
        (int32_t)((k_j2_calibration_pos -setpoint->q2) * k_joint_steps_per_rad));
      
      stepper_z.setTargetAbs(
        (int32_t)((k_z_calibration_pos -setpoint->z) * k_z_steps_per_meter));

      joint_group.startMove();
      stepper_z.moveAsync();
    }

    /** 
     * @brief Move joints at some velocity
     * @param v1 Joint 1 velocity (rad / s)
     * @param v2 Joint 2 velocity (rad / s)
     * @param vz Z stage velocity (m / s)
     */
    inline void rotate(float v1, float v2, float vz) {
      if(v1 != 0.0f) { stepper_1.rotateAsync((int32_t)(-v1 * k_joint_steps_per_rad)); }
      if(v2 != 0.0f) { stepper_2.rotateAsync((int32_t)(-v2 * k_joint_steps_per_rad)); }
      if(vz != 0.0f) { stepper_z.rotateAsync((int32_t)(-vz * k_z_steps_per_meter)); }
    }

    /** @brief Stops any active motion */
    inline void stop() {
      stepper_1.stopAsync();
      stepper_2.stopAsync();
      stepper_z.stopAsync();
    };

    /**
     * @brief Whether or not the Pantograph has reached its setpoint
     * @returns false if any motor is moving, otherwise true
     */
    inline bool is_moving() {
      return (stepper_1.isMoving || stepper_2.isMoving || stepper_z.isMoving);
    }

    /** @brief set stepper 1 position */
    inline void set_pos_j1(float pos) { 
      set_pos_single_(stepper_1, (int32_t)((k_j1_calibration_pos - pos) * k_joint_steps_per_rad)); }

    /** @brief set stepper 2 position */
    inline void set_pos_j2(float pos) { 
      set_pos_single_(stepper_2, (int32_t)((k_j2_calibration_pos - pos) * k_joint_steps_per_rad)); }

    /** @brief set stepper z position */
    inline void set_pos_z(float pos) { 
      set_pos_single_(stepper_z, (int32_t)((k_z_calibration_pos - pos) * k_z_steps_per_meter)); }

  private:

    /** @brief The joint 1 stepper object */
    Stepper stepper_1;

    /** @brief The joint 2 stepper object */
    Stepper stepper_2;

    /** @brief The z stage stepper object */
    Stepper stepper_z;

    /** @brief The joint group */
    StepperGroup joint_group;

    /**
     * @brief Rotates a single stepper asynch
     * @param stepper The stepper to rotate
     * @param velocity Target velocity for the stepper (steps per sec)
     */
    inline void rotate_async_single_(Stepper& stepper, int32_t velocity) {
      stepper.rotateAsync(velocity);
    }

    /** @brief Stops active asynch rotation */
    inline void stop_async_single_(Stepper& stepper) {
      stepper.stopAsync();
    }

    /** @brief Sets a single stepper position */
    inline void set_pos_single_(Stepper& stepper, int32_t pos) {
      stepper.setPosition(pos);
    }

};

#endif
