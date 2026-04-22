#include "pd.h"

TractionController::TractionController()
{
  prev_error = 0.0f;
  prev_error_stale = true;
  this->loop_timer.start();
}

float TractionController::get_reduction_factor(float ws_fl, float ws_fr, float ws_rl, float ws_rr)
{
  // compute average front and rear wheel speeds
  float v_front = (ws_fr + ws_fl) / 2.0f;
  float v_rear  = (ws_rr + ws_rl) / 2.0f;

  // compute slip & error if above minimum activation speed
  float slip = 0.0f;
  if (v_front > this->ACTIVATION_RPM)
  {
    slip = (v_rear - v_front) / v_front;
  }
  else 
  {
    this->reset();  // reset timer
    return 1.0f;    // dont reduce torque
  }
  float error = slip - this->TARGET_SLIP;

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

  // update prev_error 
  this->prev_error = error;
  this->prev_error_stale = false;

  // compute & clamp output
  float output = 1 - (KP * error + KD * derivative);
  if (output > this->MAX_OUTPUT) output = this->MAX_OUTPUT;
  if (output < this->MIN_OUTPUT) output = this->MIN_OUTPUT;
  return output;
}

void TractionController::reset()
{
  // reset prev_error
  this->prev_error       = 0.0f;
  this->prev_error_stale = true;

  // restart timer
  this->loop_timer.stop();
  this->loop_timer.reset();
  this->loop_timer.start();
}

float TractionController::get_last_error()
{
  return this->prev_error;
}

float TractionController::get_last_loop_time()
{
  return this->loop_time;
}
