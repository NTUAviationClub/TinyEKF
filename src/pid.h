#ifndef PID__H_
#define PID__H_

#ifdef __cplusplus
extern "C" {
#endif

#include "filter_base.h"
#include "rc_filter.h"
#include "stdio.h"

#define PID_PARAM_SIZE 5
#define PID_KP 0
#define PID_KI 1
#define PID_KD 2
#define PID_Ilimit 3
#define PID_SIdt 4

#define PID_USE_QUEUE 1
#define PID_NOT_QUEUE 2

void pid_update(DTYPE now, DTYPE setpoint, FT *ctl, int isqueue, FT *dlpf);

#ifdef __cplusplus
}
#endif

#endif