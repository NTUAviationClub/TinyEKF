#include "quaternion.h"

void print_Qtn(Qtn *q) {
  printf("w: %lf, x: %lf, y: %lf, z: %lf \r\n", q->w, q->x, q->y, q->z);
}

//(x,y,z) is the rotation axis and theta is the angle
void from_Rod(Qtn *result, DTYPE theta, DTYPE x, DTYPE y, DTYPE z) {
  DTYPE norm = sqrt(x * x + y * y + z * z);
  x = x / norm;
  y = y / norm;
  z = z / norm;
  result->w = cos(theta / 2);
  result->x = sin(theta / 2) * x;
  result->y = sin(theta / 2) * y;
  result->z = sin(theta / 2) * z;
}

// Defined under Body 3-2-1 rotation
void from_RPY(Qtn *result, DTYPE roll, DTYPE pitch, DTYPE yaw) {
  DTYPE phi = roll / 2;
  DTYPE tht = pitch / 2;
  DTYPE psi = yaw / 2;
  result->w = cos(phi) * cos(tht) * cos(psi) + sin(phi) * sin(tht) * sin(psi);
  result->x = sin(phi) * cos(tht) * cos(psi) - cos(phi) * sin(tht) * sin(psi);
  result->y = cos(phi) * sin(tht) * cos(psi) + sin(phi) * cos(tht) * sin(psi);
  result->z = cos(phi) * cos(tht) * sin(psi) - sin(phi) * sin(tht) * cos(psi);
}

// theta is ranged from -PI to PI
DTYPE get_theta(Qtn *q) { return acos(fabs(q->w)) * 2; }

// Defined under Body 3-2-1 rotation
void get_RPY(Qtn *q, DTYPE *roll, DTYPE *pitch, DTYPE *yaw) {
  *roll = atan2(2 * (q->w * q->x + q->y * q->z),
                1 - 2 * (q->x * q->x + q->y * q->y));
  *pitch = asin(2 * (q->w * q->y - q->z * q->x));
  *yaw = atan2(2 * (q->w * q->z + q->x * q->y),
               1 - 2 * (q->y * q->y + q->z * q->z));
}

void add(Qtn *q1, Qtn *q2, Qtn *q3) {
  q3->w = q1->w + q2->w;
  q3->x = q1->x + q2->x;
  q3->y = q1->y + q2->y;
  q3->z = q1->z + q2->z;
}

void sub(Qtn *q1, Qtn *q2, Qtn *res) {
  res->w = q1->w - q2->w;
  res->x = q1->x - q2->x;
  res->y = q1->y - q2->y;
  res->z = q1->z - q2->z;
}

void mul(Qtn *q1, Qtn *q2, Qtn *res) {
  res->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  res->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  res->y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
  res->z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
}

void scal(Qtn *q1, DTYPE b, Qtn *res) {
  res->w = q1->w * b;
  res->x = q1->x * b;
  res->y = q1->y * b;
  res->z = q1->z * b;
}

void inv(Qtn *q, Qtn *res) {
  res->w = q->w;
  res->x = -q->x;
  res->y = -q->y;
  res->z = -q->z;
}

// Ref: http://codewee.com/view.php?idx=42
void slerp(Qtn *q1, Qtn *q2, Qtn *q3, DTYPE t) {
  float t_ = 1 - t;
  DTYPE Wq1, Wq2;
  DTYPE theta =
      acos(q1->x * q2->x + q1->y * q2->y + q1->z * q2->z + q1->w * q2->w);
  DTYPE sn = sin(theta);
  Wq1 = sin(t_ * theta) / sn;
  Wq2 = sin(t * theta) / sn;
  q3->x = Wq1 * q1->x + Wq2 * q2->x;
  q3->y = Wq1 * q1->y + Wq2 * q2->y;
  q3->z = Wq1 * q1->z + Wq2 * q2->z;
  q3->w = Wq1 * q1->w + Wq2 * q2->w;
  normalize_inplace(q3);
}

// From Wikipedia LOL
float inverse_squareroot(float x) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = x * 0.5F;
  y = x;
  i = *(long *)&y;           // evil floating point bit level hacking
  i = 0x5f3759df - (i >> 1); // what the fuck?
  y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y)); // 1st iteration
  // y  = y * ( threehalfs - ( x2 * y * y ) );
  // 2nd iteration, this can be removed

  return y;
}

DTYPE norm_2(Qtn *q) {
  return sqrt(pow(q->w, 2) + pow(q->x, 2) + pow(q->y, 2) + pow(q->z, 2));
}

DTYPE inverse_norm_2(Qtn *q) {
  return inverse_squareroot(pow(q->w, 2) + pow(q->x, 2) + pow(q->y, 2) +
                            pow(q->z, 2));
}

void normalize(Qtn *q, Qtn *res) {
  // DTYPE norm = norm_2(q);
  DTYPE inorm = inverse_norm_2(q);
  res->w = q->w * inorm;
  res->x = q->x * inorm;
  res->y = q->y * inorm;
  res->z = q->z * inorm;
}

void normalize_inplace(Qtn *q) {
  // DTYPE norm = norm_2(q);
  DTYPE inorm = inverse_norm_2(q);
  q->w = q->w * inorm;
  q->x = q->x * inorm;
  q->y = q->y * inorm;
  q->z = q->z * inorm;
}