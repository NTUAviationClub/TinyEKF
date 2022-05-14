#include "rc_filter.h"

// Cut-off frequency and Sampling period
DTYPE rc_cf_, rc_sp_;
// Filter coefficient
DTYPE rc_a1_, rc_b0_, rc_b1_;

void set_coeff(DTYPE cutoff, DTYPE sp) {
  DTYPE rc = 1 / cutoff;
  rc_a1_ = (2 * rc - sp) / (sp + 2 * rc);
  rc_b0_ = sp / (sp + 2 * rc);
  rc_b1_ = rc_b0_;
}

void rc_update(DTYPE now, FT *filter) {
  // update coefficients
  DTYPE _sp = now - filter->last_time;
  // For RC here, filter->param[0] is the cut off frequency
  set_coeff(filter->param[RC_CUTOFF], _sp);
  // y0 initialize
  filter->current_output = 0;
  // y0 = b0*u(0)+b1*u(-1)+a1*y(-1)
  filter->current_output += rc_b0_ * filter->current_input;
  // the position of last 1st value
  int pl1v = (filter->last_pos + filter->size - 0) % filter->size;
  filter->current_output += rc_b1_ * filter->last_input[pl1v];
  filter->current_output += rc_a1_ * filter->last_output[pl1v];
  // store current value to last in/output
  filter->last_input[(pl1v + 1) % filter->size] = filter->current_input;
  filter->last_output[(pl1v + 1) % filter->size] = filter->current_output;
  // update last position
  filter->last_pos = (filter->last_pos + 1) % filter->size;
  filter->last_time = now;
}