#include "dart_internal.h"

#include <math.h>
#include <string.h>

double dart_clamp(double value, double lower, double upper)
{
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

int dart_all_finite(const double * values, int count)
{
  int index;
  for (index = 0; index < count; ++index) {
    if (!isfinite(values[index])) return 0;
  }
  return 1;
}

int dart_inverse(double * matrix, double * inverse, int size)
{
  int row;
  int column;
  int pivot_row;
  double augmented[6][12];
  if (size <= 0 || size > 6) return 0;
  memset(augmented, 0, sizeof(augmented));
  for (row = 0; row < size; ++row) {
    for (column = 0; column < size; ++column) {
      augmented[row][column] = matrix[row * size + column];
    }
    augmented[row][size + row] = 1.0;
  }
  for (column = 0; column < size; ++column) {
    double pivot_abs = fabs(augmented[column][column]);
    pivot_row = column;
    for (row = column + 1; row < size; ++row) {
      const double candidate = fabs(augmented[row][column]);
      if (candidate > pivot_abs) {
        pivot_abs = candidate;
        pivot_row = row;
      }
    }
    if (pivot_abs < 1e-12 || !isfinite(pivot_abs)) return 0;
    if (pivot_row != column) {
      for (int element = 0; element < 2 * size; ++element) {
        const double temporary = augmented[column][element];
        augmented[column][element] = augmented[pivot_row][element];
        augmented[pivot_row][element] = temporary;
      }
    }
    {
      const double pivot = augmented[column][column];
      for (int element = 0; element < 2 * size; ++element) {
        augmented[column][element] /= pivot;
      }
    }
    for (row = 0; row < size; ++row) {
      if (row != column) {
        const double factor = augmented[row][column];
        for (int element = 0; element < 2 * size; ++element) {
          augmented[row][element] -= factor * augmented[column][element];
        }
      }
    }
  }
  for (row = 0; row < size; ++row) {
    for (column = 0; column < size; ++column) {
      inverse[row * size + column] = augmented[row][size + column];
    }
  }
  return 1;
}

void dart_quaternion_normalize(double quaternion[4])
{
  const double norm = sqrt(
    quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
    quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  if (norm < 1e-12 || !isfinite(norm)) {
    quaternion[0] = 1.0;
    quaternion[1] = quaternion[2] = quaternion[3] = 0.0;
    return;
  }
  for (int index = 0; index < 4; ++index) quaternion[index] /= norm;
}

void dart_rotate_world_to_body(
  const double q[4], const double world[3], double body[3])
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  body[0] = (1.0 - 2.0 * (y * y + z * z)) * world[0] +
    2.0 * (x * y + z * w) * world[1] + 2.0 * (x * z - y * w) * world[2];
  body[1] = 2.0 * (x * y - z * w) * world[0] +
    (1.0 - 2.0 * (x * x + z * z)) * world[1] + 2.0 * (y * z + x * w) * world[2];
  body[2] = 2.0 * (x * z + y * w) * world[0] + 2.0 * (y * z - x * w) * world[1] +
    (1.0 - 2.0 * (x * x + y * y)) * world[2];
}
