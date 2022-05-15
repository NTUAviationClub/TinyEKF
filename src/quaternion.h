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
/* Use printf to print q.
 */
void print_Qtn(Qtn *q);

/* Fill result with Rodriguez representation of a rotation.
 */
void from_Rod(Qtn *result, DTYPE theta, DTYPE x, DTYPE y, DTYPE z);

/* Fill result with RPY representation of a rotation.
 * Defined under Body 3-2-1 rotation.
 */
void from_RPY(Qtn *result, DTYPE roll, DTYPE pitch, DTYPE yaw);

/* Get the rotation angle of q
 */
DTYPE get_theta(Qtn *q);

/* Get the RPY angle of q
 */
void get_RPY(Qtn *q, DTYPE *roll, DTYPE *pitch, DTYPE *yaw);

/* q1 + q2 = q3
 */
void add(Qtn *q1, Qtn *q2, Qtn *q3);

/* q1 - q2 = q3
 */
void sub(Qtn *q1, Qtn *q2, Qtn *q3);

/* q1 (*) q2 = q3
 */
void mul(Qtn *q1, Qtn *q2, Qtn *q3);

/* b * q1 = q3
 */
void scal(Qtn *q1, DTYPE b, Qtn *q3);

/* q (*) qt = (1, 0, 0, 0) (if q is already normalized)
 */
void inv(Qtn *q, Qtn *qt);

/* Spherical Linear Interpolation
 * When t is bigger, q3 gets closer to q2 and away from q1
 */
void slerp(Qtn *q1, Qtn *q2, Qtn *q3, DTYPE t);

/* return 1 / sqrt(x)
 */
float inverse_squareroot(float x);

/* return ||q||_2
 */
DTYPE norm_2(Qtn *q);

/* return 1 / ||q||_2
 */
DTYPE inverse_norm_2(Qtn *q);

/* qn = q / ||q||_2
 */
void normalize(Qtn *q, Qtn *qn);

/* q = q / ||q||_2
 */
void normalize_inplace(Qtn *q);

#ifdef __cplusplus
}
#endif

#endif
