#ifndef Qtn_C_H
#define Qtn_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "filter_base.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

typedef struct Quaternion {
  DTYPE w, x, y, z;
} Qtn;

// DTYPE test;

void print_Qtn(Qtn *q);

void from_Rod(Qtn *result, DTYPE theta, DTYPE x, DTYPE y, DTYPE z);
void from_RPY(Qtn *result, DTYPE roll, DTYPE pitch, DTYPE yaw);
DTYPE get_theta(Qtn *q);
void get_RPY(Qtn *q, DTYPE *roll, DTYPE *pitch, DTYPE *yaw);
void add(Qtn *q1, Qtn *q2, Qtn *q3);
void sub(Qtn *q1, Qtn *q2, Qtn *q3);
void mul(Qtn *q1, Qtn *q2, Qtn *q3);
void scal(Qtn *q1, DTYPE b, Qtn *q3);
void inv(Qtn *q, Qtn *qt);
DTYPE norm_2(Qtn *q);
void normalize(Qtn *q, Qtn *qn);

#ifdef __cplusplus
}
#endif

#endif
