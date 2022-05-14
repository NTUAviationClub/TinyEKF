#include "pid.h"

DTYPE last_p_ = 0, last_i_ = 0, last_d_ = 0;

DTYPE Dabs(DTYPE r) { return (r > 0 ? r : (-r)); }

void pid_update(DTYPE now, DTYPE setpoint, FT *ctl, int isqueue, FT *dlpf) {
  DTYPE _p = 0, _i = 0, _d = 0, _e = 0, _dot = 0;
  DTYPE dt = now - ctl->last_time;
  _e = setpoint - ctl->current_input;
  // Calculate input derivative.
  _dot =
      (ctl->current_input - ctl->last_input[(ctl->last_pos) % ctl->size]) / dt;
  // printf("dt: %lf, this input: %lf, last input: %lf\r\n", dt,
  //       ctl->current_input, ctl->last_input[(ctl->last_pos) % ctl->size]);
  // And the d-term needs a LPF to prevent oscillation.
  dlpf->current_input = _dot;
  rc_update(now, dlpf);
  _dot = dlpf->current_output;
  // If we use queue to store the past integral value, we will need to
  // handle the error integral window.
  // The param 3 is the integral of all past errors in the window.
  // We need to add the new one and delete the oldest one.
  // Also, the past _e*dt will be stored in last_output array.
  // If we, however, choose not to use the queue, we will rely on the integral
  // limit to prevent wind-up effect.
  ctl->param[PID_SIdt] += _e * dt;
  if (isqueue == PID_USE_QUEUE) {
    ctl->param[PID_SIdt] -= ctl->last_output[(ctl->last_pos + 1) % ctl->size];
  } else if (isqueue == PID_NOT_QUEUE) {
    if (Dabs(ctl->param[PID_SIdt]) > ctl->param[PID_Ilimit] * dt) {
      if (ctl->param[PID_SIdt] > 0) {
        ctl->param[PID_SIdt] = ctl->param[PID_Ilimit] * dt;
      } else if (ctl->param[PID_SIdt] <= 0) {
        ctl->param[PID_SIdt] = -ctl->param[PID_Ilimit] * dt;
      }
    }
  }
  // Then we can update last_pose,
  // add the latest -e*dt to the last_output array (if we use queue), and
  // add the latest input to the last_input array.
  ctl->last_pos = (ctl->last_pos + 1) % ctl->size;
  ctl->last_input[ctl->last_pos] = ctl->current_input;
  if (isqueue == PID_USE_QUEUE) {
    ctl->last_output[ctl->last_pos] = _e * dt;
  }
  // Finally we can put things together.
  // param 0 = Kp, param 1 = Ki, param 2 = Kd
  _p = ctl->param[PID_KP] * _e;
  _i = ctl->param[PID_KI] * ctl->param[PID_SIdt];
  _d = ctl->param[PID_KD] * _dot;
  // record the last value
  last_p_ = _p;
  last_i_ = _i;
  last_d_ = _d;
  ctl->current_output = _p + _i + _d;
  ctl->last_time = now;
  // printf("p: %lf, i: %lf, d: %lf\r\n", _p, _i, _d);
}

void get_last_pid(DTYPE *p, DTYPE *i, DTYPE *d) {
  *p = last_p_;
  *i = last_i_;
  *d = last_d_;
}
