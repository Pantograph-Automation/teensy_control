#include <AS5600.h>
#include "control.hpp"
#include "config.h"

// usbipd list
// usbipd attach --wsl --busid 2-1

float offset = 0.0f;

const float POS_CMD_MAX_ERROR = 5 * RAD_PER_STEP;

// Encoder E1(E1_A_PIN, E2_B_PIN);
AS5600 as5600;

// Stepper 1
Stepper stepper(S1_PULSE_PIN, S1_ENABLE_PIN, S1_DIR_PIN);

Controller controller(stepper);

uint32_t clk = 0;

const float desired_position = 1.570796; // 90 degrees

float calibrate(const int sample_size, const unsigned long ms_delay)
{
  float offset_ = 0.0;

  for (int i = 0; i < sample_size; i++)
  {
    offset_ +=  as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
    delay(ms_delay);
  }
  offset_ = offset_ / (float)sample_size;
  return offset_;
}

float read_encoder(float offset_)
{

  float radians = as5600.rawAngle() * AS5600_RAW_TO_RADIANS - offset_;

  if (radians < 0)
  {
    radians += 2*PI;
  }

  return radians;
}

void setup()
{
  Serial.begin(11520);
  Wire.begin();
  Wire.setClock(400000);

  as5600.begin(4);  //  set direction pin.

  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  stepper.disable();
  Serial.println("Calibrating...");
  delay(2000);
  offset = calibrate(100, 20);
  Serial.println("Calibrated!! Move the arm.");
  delay(2000);
  stepper.enable();
  
}


void loop()
{
  
  // Pulse the stepper motor if required
  stepper.pulse_if_required(PULSE_WIDTH_US);

  // Get the current measured position
  float current_angle = read_encoder(offset);

  stepper.update_state(current_angle, micros());


  /**
   * Other code goes here
   */


  // Run the PID law to determine whether a step is required
  Status status = controller.pid(
    Kp, Ki, Kd,
    desired_position,
    V_MAX,
    ACCEL_MAX,
    POS_CMD_MAX_ERROR,
    POS_DEADBAND,
    RAD_PER_STEP
  );

  if (status == Status::FINISHED)
  {
    Serial.println("Finished moving to setpoint!");
  }
}