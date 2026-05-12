#include "traction_control.h"
#include <numbers>

TractionController::TractionController()
{
  // set all values to default state
  this->slip = 0.0f;
  this->prev_slip = 0.0f;
  this->prev_slip_stale = true;
  this->integral = 0.0f;
  this->smoothed_derivative = 0.0f;
  this->filter_init = false;
  this->loop_time = 0.0f;
  this->last_output = 1.0f;
  this->saturated = false;
  // start timer
  this->loop_timer.start();
  // set filter time constant 
  this->filter_time_constant = 1.0f / (2.0f * M_PI * this->FILTER_CUTOFF_FREQ);
  this->max_integral = KI > 0.0f ? MAX_INTEGRATOR_REDUCTION_FACTOR / KI : 0.0f;
}

float TractionController::update(float ws_fl, float ws_fr, float ws_rl, float ws_rr)
{
  // compute average front and rear wheel speeds
  float v_front = (ws_fr + ws_fl) / 2.0f;
  float v_rear  = (ws_rr + ws_rl) / 2.0f;

  // compute slip if above minimum activation speed
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

  // read & restart timer
  this->loop_timer.stop();
  std::chrono::microseconds loop_time_micro = this->loop_timer.elapsed_time();  // microseconds
  this->loop_time = loop_time_micro.count() / 1000000.0f;                       // seconds
  this->loop_timer.reset();
  this->loop_timer.start();

  // compute error
  const float error = this->slip - TARGET_WHEEL_SLIP;

  // compute derivative of slip
  float raw_derivative = 0.0f;
  if (!prev_slip_stale && loop_time > 0.0f)
  {
    raw_derivative = (slip - prev_slip) / loop_time;
    if (!filter_init) 
    {
      this->smoothed_derivative = raw_derivative;
      filter_init = true;
    } 
    else
    {
      const float exponent = -1.0f * loop_time / this->filter_time_constant;
      const float filter_coefficient = exp(exponent);
      this->smoothed_derivative = (1 - filter_coefficient) * raw_derivative + filter_coefficient * this->smoothed_derivative;
    }
  }

  // compute & clamp output
  float unclamped = 0.0f;
  if (!prev_slip_stale)
  {
    unclamped = 1 - (KP * error + KI * integral + KD * smoothed_derivative);
  } 
  else
  {
    unclamped = 1 - (KP * error + KI * integral);
  }
  float output = unclamped;
  if (output > this->MAX_OUTPUT) output = this->MAX_OUTPUT;
  if (output < this->MIN_OUTPUT) output = this->MIN_OUTPUT;

  // determine if saturated (error is positive, but torque reduction factor is being clamped to MIN_OUTPUT)
  // skip integration if saturated to prevent integral windup
  // we only care about the positive error case because negative windup
  // is solved by the integral clamp below
  this->saturated = (unclamped <= this->MIN_OUTPUT) && (error > 0);
  if (!this->saturated && !this->prev_slip_stale) 
  {
    this->integral += error * loop_time;
    // clamp integral [0, MAX_INTEGRATOR_REDUCTION_FACTOR / KI]
    if (integral > max_integral) integral = max_integral;
    if (integral < 0.0f) integral = 0.0f; 
  }

  // update prev_error, last_output
  this->prev_slip = slip;
  this->prev_slip_stale = false;
  this->last_output = output;

  return output;
}

void TractionController::reset()
{
  // reset slip
  this->slip = 0.0f;
  this->prev_slip = 0.0f;
  this->prev_slip_stale = true;

  // reset integral
  this->integral = 0.0f;

  // reset derivative & filter
  this->smoothed_derivative = 0.0f;
  this->filter_init = false;

  // reset output & saturated
  this->last_output = 1.0f;
  this->saturated = false;

  // restart timer
  this->loop_timer.stop();
  this->loop_timer.reset();
  this->loop_timer.start();
}

float TractionController::get_smoothed_derivative() const
{
  return this->smoothed_derivative;
}

float TractionController::get_integral() const
{
  return this->integral;
}

float TractionController::get_loop_time() const
{
  return this->loop_time;
}

float TractionController::get_slip() const 
{
  return this->slip;
}

float TractionController::get_output() const
{
  return this->last_output;
}

bool TractionController::get_saturated() const
{
  return this->saturated;
}
