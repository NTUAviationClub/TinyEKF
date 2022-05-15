#ifndef COM_FILTER_H__
#define COM_FILTER_H__

#include "quaternion.h"
#include "stdio.h"

#define CF_OUTPUT_SIZE_ 7
#define CF_PARAM_SIZE_ 2

#define CF_ALPHA_ 0
#define CF_BETA_ 1
#define CF_QW_ 0
#define CF_QX_ 1
#define CF_QY_ 2
#define CF_QZ_ 3
#define CF_BX_ 4
#define CF_BY_ 5
#define CF_BZ_ 6

// If the alpha goes smaller, the filter will trust accerometer (_q_acc) more
void cf_init(DTYPE _now, DTYPE _aplha, DTYPE _beta);

void cf_update(DTYPE _now, Qtn *_q_acc, DTYPE _wx, DTYPE _wy, DTYPE _wz);

void cf_get_filtered_qtn(DTYPE *qw, DTYPE *qx, DTYPE *qy, DTYPE *qz);

void cf_get_bias(DTYPE *bx, DTYPE *by, DTYPE *bz);

// if the FT struct is initialized by the user, then they should use this update
// function.
void cf_update_ui(DTYPE _now, FT *_cf, Qtn *_q_acc, DTYPE _wx, DTYPE _wy,
                  DTYPE _wz);

#endif