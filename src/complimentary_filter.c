#include "complimentary_filter.h"

unsigned char inited_ = 0;
FT cf_;

DTYPE last_input_[CF_OUTPUT_SIZE_], last_output_[CF_OUTPUT_SIZE_],
    param_[CF_PARAM_SIZE_];

void cf_init(DTYPE _now, DTYPE _alpha, DTYPE _beta) {
  if (inited_) {
    return;
  }
  cf_.current_input = 0;
  cf_.current_output = 0;
  cf_.last_input = last_input_;
  cf_.last_output = last_output_;
  cf_.param = param_;
  cf_.param[CF_ALPHA_] = _alpha;
  cf_.param[CF_BETA_] = _beta;
  for (int i = 0; i < CF_OUTPUT_SIZE_; i++) {
    last_input_[i] = 0;
    last_output_[i] = 0;
  }
  cf_.last_output[CF_QW_] = 1;
  inited_ = 1;
}

void cf_update(DTYPE _now, Qtn *_q_acc, DTYPE _wx, DTYPE _wy, DTYPE _wz) {
  if (inited_) {
    cf_update_ui(_now, &cf_, _q_acc, _wx, _wy, _wz);
  }
}

void cf_get_filtered_qtn(DTYPE *qw, DTYPE *qx, DTYPE *qy, DTYPE *qz) {
  if (inited_) {
    *qw = cf_.last_output[CF_QW_];
    *qx = cf_.last_output[CF_QX_];
    *qy = cf_.last_output[CF_QY_];
    *qz = cf_.last_output[CF_QZ_];
  }
}

void cf_get_bias(DTYPE *bx, DTYPE *by, DTYPE *bz) {
  if (inited_) {
    *bx = cf_.last_output[CF_BX_];
    *by = cf_.last_output[CF_BY_];
    *bz = cf_.last_output[CF_BZ_];
  }
}

void cf_update_ui(DTYPE _now, FT *_cf, Qtn *_q_acc, DTYPE _wx, DTYPE _wy,
                  DTYPE _wz) {
  // get dt
  DTYPE dt = _now - _cf->last_time;
  _cf->last_time = _now;
  // q_gyr_beforebias = -0.5*w*dt (*) q_k-1 + q_k-1
  Qtn w_qt = {.w = 0, .x = _wx, .y = _wy, .z = _wz};
  Qtn b_qt = {.w = 0,
              .x = _cf->last_output[CF_BX_],
              .y = _cf->last_output[CF_BY_],
              .z = _cf->last_output[CF_BZ_]};
  Qtn last_qt = {.w = _cf->last_output[CF_QW_],
                 .x = _cf->last_output[CF_QX_],
                 .y = _cf->last_output[CF_QY_],
                 .z = _cf->last_output[CF_QZ_]};
  Qtn q_gyr_bb, q_gyr, q_new, q_b, q_b_fix, q_last_i;
  scal(&w_qt, -0.5 * dt, &w_qt);
  mul(&w_qt, &last_qt, &q_gyr_bb);
  add(&q_gyr_bb, &last_qt, &q_gyr_bb);
  // q_gyr = q_gyr_bb - (-0.5)*b*dt (*) q_k-1
  scal(&b_qt, -0.5 * dt, &b_qt);
  mul(&b_qt, &last_qt, &q_b);
  sub(&q_gyr_bb, &q_b, &q_gyr);
  // q_k = slerp(q_acc, q_gyr)
  DTYPE alpha = _cf->param[CF_ALPHA_];
  slerp(_q_acc, &q_gyr, &q_new, alpha);
  // Update bias
  DTYPE beta = _cf->param[CF_BETA_];
  sub(_q_acc, &q_gyr_bb, &q_b_fix);
  inv(&last_qt, &q_last_i);
  mul(&q_b_fix, &q_last_i, &q_b_fix);
  _cf->last_output[CF_BX_] =
      beta * _cf->last_output[CF_BX_] + 2 * (1 - beta) * q_b_fix.x / dt;
  _cf->last_output[CF_BY_] =
      beta * _cf->last_output[CF_BY_] + 2 * (1 - beta) * q_b_fix.y / dt;
  _cf->last_output[CF_BZ_] =
      beta * _cf->last_output[CF_BZ_] + 2 * (1 - beta) * q_b_fix.z / dt;
  // Update the value in _cf
  _cf->last_output[CF_QW_] = q_new.w;
  _cf->last_output[CF_QX_] = q_new.x;
  _cf->last_output[CF_QY_] = q_new.y;
  _cf->last_output[CF_QZ_] = q_new.z;
}