#include "cascade_attitude_control.h"

Qtn qs_ = {.w = 1, .x = 0, .y = 0, .z = 0},
    qm_ = {.w = 1, .x = 0, .y = 0, .z = 0};
DTYPE rrm_ = 0, prm_ = 0, yrm_ = 0, tau_ = 1;
DTYPE rs_ = 0, ps_ = 0, ys_ = 0;
DTYPE cx_ = 0, cy_ = 0, cz_ = 0;
char first_update_ = 1;

// PID spaces
FT rpid_, ppid_, ypid_;
FT rlpf_, plpf_, ylpf_;
// roll
DTYPE rlpf_last_input[RC_MEMORY_SIZE];
DTYPE rlpf_last_output[RC_MEMORY_SIZE];
DTYPE rlpf_param[RC_PARAM_SIZE];
DTYPE rpid_last_input[CAC_PID_MEMORY_SIZE];
// DTYPE rpid_last_output[CAC_PID_MEMORY_SIZE];
DTYPE rpid_param[PID_PARAM_SIZE];
// pitch
DTYPE plpf_last_input[RC_MEMORY_SIZE];
DTYPE plpf_last_output[RC_MEMORY_SIZE];
DTYPE plpf_param[RC_PARAM_SIZE];
DTYPE ppid_last_input[CAC_PID_MEMORY_SIZE];
// DTYPE ppid_last_output[CAC_PID_MEMORY_SIZE];
DTYPE ppid_param[PID_PARAM_SIZE];
// yaw
DTYPE ylpf_last_input[RC_MEMORY_SIZE];
DTYPE ylpf_last_output[RC_MEMORY_SIZE];
DTYPE ylpf_param[RC_PARAM_SIZE];
DTYPE ypid_last_input[CAC_PID_MEMORY_SIZE];
// DTYPE ypid_last_output[CAC_PID_MEMORY_SIZE];
DTYPE ypid_param[PID_PARAM_SIZE];

DTYPE cac_sign(DTYPE x) {
  if (x >= 0) {
    return 1;
  } else {
    return -1;
  }
}

void cac_init() {
  if (!first_update_) {
    return;
  }
  // roll d-term lpf initialize
  rlpf_.size = RC_MEMORY_SIZE;
  rlpf_.last_pos = 0;
  rlpf_.last_time = 0.0f;
  rlpf_.current_input = 0;
  rlpf_.current_output = 0;
  rlpf_.last_input = rlpf_last_input;
  rlpf_.last_output = rlpf_last_output;
  rlpf_.param = rlpf_param;
  rlpf_.param[RC_CUTOFF] = CAC_LPF_CUTOFF;
  for (int i = 0; i < rlpf_.size; i++) {
    rlpf_last_input[i] = 0;
    rlpf_last_output[i] = 0;
  }
  // roll pid initialize
  rpid_.size = CAC_PID_MEMORY_SIZE;
  rpid_.last_pos = 0;
  rpid_.last_time = 0.0f;
  rpid_.current_input = 0;
  rpid_.current_output = 0;
  rpid_.last_input = rpid_last_input;
  // rpid_.last_output = rpid_last_output;
  rpid_.param = rpid_param;
  rpid_.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < CAC_PID_MEMORY_SIZE; i++) {
    rpid_last_input[i] = 0;
    // rpid_last_output[i] = 0;
  }
  // pitch d-term lpf initialize
  plpf_.size = RC_MEMORY_SIZE;
  plpf_.last_pos = 0;
  plpf_.last_time = 0.0f;
  plpf_.current_input = 0;
  plpf_.current_output = 0;
  plpf_.last_input = plpf_last_input;
  plpf_.last_output = plpf_last_output;
  plpf_.param = plpf_param;
  plpf_.param[RC_CUTOFF] = CAC_LPF_CUTOFF;
  for (int i = 0; i < plpf_.size; i++) {
    plpf_last_input[i] = 0;
    plpf_last_output[i] = 0;
  }
  // pitch pid initialize
  ppid_.size = CAC_PID_MEMORY_SIZE;
  ppid_.last_pos = 0;
  ppid_.last_time = 0.0f;
  ppid_.current_input = 0;
  ppid_.current_output = 0;
  ppid_.last_input = ppid_last_input;
  // ppid_.last_output = ppid_last_output;
  ppid_.param = ppid_param;
  ppid_.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < CAC_PID_MEMORY_SIZE; i++) {
    ppid_last_input[i] = 0;
    // ppid_last_output[i] = 0;
  }
  // yaw d-term lpf initialize
  ylpf_.size = RC_MEMORY_SIZE;
  ylpf_.last_pos = 0;
  ylpf_.last_time = 0.0f;
  ylpf_.current_input = 0;
  ylpf_.current_output = 0;
  ylpf_.last_input = ylpf_last_input;
  ylpf_.last_output = ylpf_last_output;
  ylpf_.param = ylpf_param;
  ylpf_.param[RC_CUTOFF] = CAC_LPF_CUTOFF;
  for (int i = 0; i < ylpf_.size; i++) {
    ylpf_last_input[i] = 0;
    ylpf_last_output[i] = 0;
  }
  // yaw pid initialize
  ypid_.size = CAC_PID_MEMORY_SIZE;
  ypid_.last_pos = 0;
  ypid_.last_time = 0.0f;
  ypid_.current_input = 0;
  ypid_.current_output = 0;
  ypid_.last_input = ypid_last_input;
  // ypid_.last_output = ypid_last_output;
  ypid_.param = ypid_param;
  ypid_.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < CAC_PID_MEMORY_SIZE; i++) {
    ypid_last_input[i] = 0;
    // ypid_last_output[i] = 0;
  }
}

void cac_set_rpy_setpoint(DTYPE roll, DTYPE pitch, DTYPE yaw) {
  from_RPY(&qs_, roll, pitch, yaw);
}

void cac_set_qtn_measured(Qtn *qm) {
  qm_.w = qm->w;
  qm_.x = qm->x;
  qm_.y = qm->y;
  qm_.z = qm->z;
}

void cac_set_gyr_estimate(DTYPE roll_rate, DTYPE pitch_rate, DTYPE yaw_rate) {
  rrm_ = roll_rate;
  prm_ = pitch_rate;
  yrm_ = yaw_rate;
}

void cac_set_attitude_control_tau(DTYPE tau) { tau_ = tau; }

void cac_set_rate_control_pid(char pid_type, DTYPE kp, DTYPE ki, DTYPE kd) {
  switch (pid_type) {
  case CAC_ROLL:
    rpid_.param[PID_KP] = kp;
    rpid_.param[PID_KI] = ki;
    rpid_.param[PID_KD] = kd;
    break;
  case CAC_PITCH:
    ppid_.param[PID_KP] = kp;
    ppid_.param[PID_KI] = ki;
    ppid_.param[PID_KD] = kd;
    break;
  case CAC_YAW:
    ypid_.param[PID_KP] = kp;
    ypid_.param[PID_KI] = ki;
    ypid_.param[PID_KD] = kd;
    break;
  default:
    break;
  }
}

void cac_update_attitude_control(DTYPE now) {
  Qtn qe, inv_qm;
  inv(&qm_, &inv_qm);
  mul(&inv_qm, &qs_, &qe);
  rs_ = (2 / tau_) * cac_sign(qe.w) * qe.x;
  ps_ = (2 / tau_) * cac_sign(qe.w) * qe.y;
  ys_ = (2 / tau_) * cac_sign(qe.w) * qe.z;
}

void cac_update_rate_control(DTYPE now) {
  // roll rate control
  pid_update(now, rs_, &rpid_, PID_NOT_QUEUE, &rlpf_);
  cx_ = rpid_.current_output;
  // pitch rate control
  pid_update(now, ps_, &ppid_, PID_NOT_QUEUE, &plpf_);
  cy_ = ppid_.current_output;
  // yaw rate control
  pid_update(now, ys_, &ypid_, PID_NOT_QUEUE, &ylpf_);
  cz_ = ypid_.current_output;
}

void cac_get_command(DTYPE *cx, DTYPE *cy, DTYPE *cz) {
  *cx = cx_;
  *cy = cy_;
  *cz = cz_;
}