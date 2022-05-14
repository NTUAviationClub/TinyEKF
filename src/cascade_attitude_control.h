#ifndef Cascade_Attitude_Control__H
#define Cascade_Attitude_Control__H

#ifdef __cplusplus
extern "C" {
#endif

#include "filter_base.h"
#include "math.h"
#include "pid.h"
#include "quaternion.h"

#define CAC_ROLL 1
#define CAC_PITCH 2
#define CAC_YAW 3

#define CAC_PID_MEMORY_SIZE 2
#define CAC_LPF_CUTOFF 3.0f

void cac_set_rpy_setpoint(DTYPE roll, DTYPE pitch, DTYPE yaw);

void cac_set_qtn_measured(Qtn *qm);

void cac_set_gyr_estimate(DTYPE roll_rate, DTYPE pitch_rate, DTYPE yaw_rate);

void cac_set_attitude_control_tau(DTYPE tau);

void cac_set_rate_control_pid(char pid_type, DTYPE kp, DTYPE ki, DTYPE kd);

void cac_update_attitude_control(DTYPE now);

void cac_update_rate_control(DTYPE now);

void cac_get_command(DTYPE *cx, DTYPE *cy, DTYPE *cz);

#ifdef __cplusplus
}
#endif

#endif
