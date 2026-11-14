#include "dart_algorithm/dart_algorithm.h"
#include "dart_internal.h"

#include <math.h>
#include <string.h>

void dart_control_default_config(DartControlConfig * config)
{
  if (config == NULL) return;
  memset(config, 0, sizeof(*config));
  for (int axis = 0; axis < 3; ++axis) {
    config->line_of_sight_gain[axis] = 1.0;
    config->angular_rate_gain[axis] = 0.1;
  }
  for (int fin = 0; fin < 4; ++fin) {
    config->fin_min_rad[fin] = -0.35;
    config->fin_max_rad[fin] = 0.35;
  }
}

int dart_control_step(
  const DartControlConfig * config, const DartSelfEkf * self_filter,
  const DartSingerFilter * target_filter, double fin_angles[4])
{
  double target_body[3];
  double desired_moment[3];
  double norm;
  if (config == NULL || self_filter == NULL || target_filter == NULL || fin_angles == NULL ||
    !self_filter->initialized || !target_filter->initialized) return 0;
  dart_rotate_world_to_body(self_filter->x, target_filter->x, target_body);
  norm = sqrt(target_body[0] * target_body[0] + target_body[1] * target_body[1] + target_body[2] * target_body[2]);
  if (!isfinite(norm) || norm < 1e-6 || target_body[0] <= 0.0) return 0;
  target_body[0] /= norm;
  target_body[1] /= norm;
  target_body[2] /= norm;
  /* Cross(+X, LOS) = [0, -los_z, los_y]. Roll is damped only until a roll objective exists. */
  desired_moment[0] = -config->angular_rate_gain[0] * self_filter->x[4];
  desired_moment[1] = -config->line_of_sight_gain[1] * target_body[2] -
    config->angular_rate_gain[1] * self_filter->x[5];
  desired_moment[2] = config->line_of_sight_gain[2] * target_body[1] -
    config->angular_rate_gain[2] * self_filter->x[6];
  for (int fin = 0; fin < 4; ++fin) {
    double angle = config->fin_neutral_rad[fin];
    for (int axis = 0; axis < 3; ++axis) angle += config->fin_allocation[fin][axis] * desired_moment[axis];
    fin_angles[fin] = dart_clamp(angle, config->fin_min_rad[fin], config->fin_max_rad[fin]);
  }
  return dart_all_finite(fin_angles, 4);
}
