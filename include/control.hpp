#pragma once
#include "stepper.hpp"

enum class Status
{
  ACTIVE,
  FINISHED,
  ERROR
};

class Controller
{
  int BUFFER_CAP = 3;

  public:

    Controller(Stepper& stepper) : stepper_(stepper) {
      error_buffer_.fill(0.0);
      cmd_buffer_.fill(0.0);
    };

    /**
     * @brief compute the PID law for a velocity-controlled stepper motor
     * @param Kp Stiffness gain
     * @param Ki Integral gain
     * @param Kd Damping gain
     * @param desired_position Desired angle (rads)
     * @param max_velocity Maximum allowed velocity
     * @param max_acceleration Maximum allowed acceleration
     * @param position_deadband Allowable error deadband
     * @returns Commanded angular velocity (rad/s)
     * 
     */
    inline Status pid(
      const float Kp,
      const float Ki,
      const float Kd,
      const float desired_position,
      const float max_velocity,
      const float max_acceleration,
      const float max_pos_cmd_delta,
      const float position_deadband = 0.002f,
      const float rad_per_step = 0.003926991f
    ) {

      // update error, deadbanding if neccessary
      float error = desired_position - stepper_.get_position();
      if (fabs(error) < position_deadband) {
        error = 0.0f;
      }
      error_buffer_.pushOverwrite(error);

      if(error + error_buffer_.get(1) == 0.0f)
      {
        return Status::FINISHED;
      }
      
      // vel_cmd = Kp (e(k) - e(k - 1)) + Ki e(k) dt + Kd * (e(k) - 2e(k - 1) + e(k-2)) / (dt)
      float proportional = Kp * (error - error_buffer_.get(1));
      float integral = Ki * (error * stepper_.get_time_delta() * 1e-9);
      float derivative = Kd * ((error - 2*error_buffer_.get(1) 
        + error_buffer_.get(2)) / (stepper_.get_time_delta() * 1e-9));
      
      float velocity_cmd = proportional + integral + derivative;

      // Clamp between allowable levels
      clamp(velocity_cmd, -max_velocity, max_velocity);
      clamp(velocity_cmd,
        cmd_buffer_.get(1) - max_acceleration, 
        cmd_buffer_.get(1) + max_acceleration);
      
      // Update command buffer
      cmd_buffer_.pushOverwrite(velocity_cmd);

      float pos_cmd_delta = velocity_cmd * stepper_.get_time_delta() * 1e-9;
      
      clamp(pos_cmd_delta, -max_pos_cmd_delta, max_pos_cmd_delta);

      pos_cmd += pos_cmd_delta;

      stepper_.check_step_required(pos_cmd, rad_per_step, pos_est);

      return Status::ACTIVE;
    }

    template <typename T>
    void clamp(T& val, const T& low, const T& high)
    {
      if (val < low) val = low;
      if (val > high) val = high;
    }


  private:

    Stepper stepper_;
    Buffer<float> error_buffer_{2};
    Buffer<float> cmd_buffer_{2};

    bool is_state_updated_ = false;

    // Commanded state
    float pos_cmd;
    // Internally estimated state (use to smooth pulse patterns)
    float pos_est;

};