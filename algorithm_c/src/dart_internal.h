#ifndef DART_ALGORITHM_DART_INTERNAL_H
#define DART_ALGORITHM_DART_INTERNAL_H

double dart_clamp(double value, double lower, double upper);
int dart_all_finite(const double * values, int count);
int dart_inverse(double * matrix, double * inverse, int size);
void dart_quaternion_normalize(double quaternion[4]);
void dart_rotate_world_to_body(
  const double quaternion_wxyz[4], const double world[3], double body[3]);

#endif
