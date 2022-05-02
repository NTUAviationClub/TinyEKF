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

DTYPE norm_2(Qtn *q) {
  return sqrt(pow(q->w, 2) + pow(q->x, 2) + pow(q->y, 2) + pow(q->z, 2));
}

void normalize(Qtn *q, Qtn *res) {
  DTYPE norm = norm_2(q);
  res->w = q->w / norm;
  res->x = q->x / norm;
  res->y = q->y / norm;
  res->z = q->z / norm;
}
