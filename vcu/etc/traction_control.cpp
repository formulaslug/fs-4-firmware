#include "traction_control.h"

TractionController::TractionController()
{
  // set all values to default state
  this->slip = 0.0f;
  this->prev_error = 0.0f;
  this->prev_error_stale = true;
  this->integral = 0.0f;
  this->loop_time = 0.0f;
  this->last_output = 1.0f;
  this->saturated = false;
  // start timer
  this->loop_timer.start();
}

float TractionController::update(float ws_fl, float ws_fr, float ws_rl, float ws_rr)
{
  // compute average front and rear wheel speeds
  float v_front = (ws_fr + ws_fl) / 2.0f;
  float v_rear  = (ws_rr + ws_rl) / 2.0f;

  // compute slip & error if above minimum activation speed
  this->slip = 0.0f;
  if (v_front > this->ACTIVATION_RPM && v_rear > 0)
  {
    slip = (v_rear - v_front) / v_rear;
  }
  else 
  {
    this->reset();  // reset timer
    return 1.0f;    // dont reduce torque
  }
  // clamp slip to [0, 1]
  if (slip < 0.0f) slip = 0.0f;
  if (slip > 1.0f) slip = 1.0f;

  float error = slip - this->TARGET_WHEEL_SLIP;

  // read & restart timer
  this->loop_timer.stop();
  std::chrono::microseconds loop_time_micro = this->loop_timer.elapsed_time();  // microseconds
  this->loop_time = loop_time_micro.count() / 1000000.0f;                       // seconds
  this->loop_timer.reset();
  this->loop_timer.start();

  // compute derivative
  float derivative = 0.0f;
  if (!this->prev_error_stale && this->loop_time > 0.0f)
  {
    derivative = (error - this->prev_error) / loop_time;
  }

  // compute & clamp output
  float unclamped = 1 - (KP * error + KI * integral + KD * derivative);
  float output = unclamped;
  if (output > this->MAX_OUTPUT) output = this->MAX_OUTPUT;
  if (output < this->MIN_OUTPUT) output = this->MIN_OUTPUT;

  // prevent windup by checking if the controller is saturated - dont integrate if saturated
  this->saturated = ((unclamped <= this->MIN_OUTPUT && error > 0) || (unclamped >= this->MAX_OUTPUT && error < 0));
  if (!this->saturated && !this->prev_error_stale) 
  {
    this->integral += error * loop_time;
  }

  // update prev_error, last_output
  this->prev_error = error;
  this->prev_error_stale = false;
  this->last_output = output;

  return output;
}

void TractionController::reset()
{
  // reset prev_error
  this->prev_error       = 0.0f;
  this->prev_error_stale = true;

  // reset integral
  this->integral = 0.0f;

  // restart timer
  this->loop_timer.stop();
  this->loop_timer.reset();
  this->loop_timer.start();
}

float TractionController::get_last_error() const
{
  return this->prev_error;
}

float TractionController::get_last_loop_time() const
{
  return this->loop_time;
}

float TractionController::get_last_slip() const 
{
  return this->slip;
}

float TractionController::get_last_output() const
{
  return this->last_output;
}

bool TractionController::get_last_saturated() const
{
  return this->saturated;
}
