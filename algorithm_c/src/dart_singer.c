#include "dart_algorithm/dart_algorithm.h"
#include "dart_internal.h"

#include <math.h>
#include <string.h>

static void axis_matrices(
  double dt, double tau, double density, double transition[3][3], double noise[3][3])
{
  const double a = 1.0 / tau;
  const double e = exp(-a * dt);
  const double e2 = e * e;
  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a3 * a;
  const double a5 = a4 * a;
  memset(transition, 0, sizeof(double) * 9);
  transition[0][0] = 1.0;
  transition[0][1] = dt;
  transition[0][2] = (e + a * dt - 1.0) / a2;
  transition[1][1] = 1.0;
  transition[1][2] = (1.0 - e) / a;
  transition[2][2] = e;
  memset(noise, 0, sizeof(double) * 9);
  noise[0][0] = dt * dt * dt / (3.0 * a2) - dt * dt / a3 + dt / a4 +
    (0.5 - 2.0 * a * dt * e - 0.5 * e2) / a5;
  noise[0][1] = dt * dt / (2.0 * a2) - dt / a3 +
    (0.5 - e + a * dt * e + 0.5 * e2) / a4;
  noise[0][2] = (0.5 - a * dt * e - 0.5 * e2) / a3;
  noise[1][1] = (dt - 2.0 * (1.0 - e) / a + (1.0 - e2) / (2.0 * a)) / a2;
  noise[1][2] = (1.0 - 2.0 * e + e2) / (2.0 * a2);
  noise[2][2] = (1.0 - e2) / (2.0 * a);
  noise[1][0] = noise[0][1];
  noise[2][0] = noise[0][2];
  noise[2][1] = noise[1][2];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) noise[row][column] *= density;
  }
}

void dart_singer_default_config(DartSingerConfig * config)
{
  if (config == NULL) return;
  config->tau_s = 0.5;
  config->acceleration_spectral_density = 1.0;
  config->position_variance = 0.01;
  config->initial_position_variance = 0.25;
  config->initial_velocity_variance = 4.0;
  config->initial_acceleration_variance = 16.0;
  config->mahalanobis_limit = 16.27;
}

int dart_singer_init(
  DartSingerFilter * filter, const DartSingerConfig * config, const double position[3])
{
  if (filter == NULL || config == NULL || position == NULL || !dart_all_finite(position, 3) ||
    config->tau_s <= 0.0 || config->acceleration_spectral_density < 0.0 ||
    config->position_variance <= 0.0)
  {
    return 0;
  }
  memset(filter, 0, sizeof(*filter));
  filter->config = *config;
  for (int axis = 0; axis < 3; ++axis) {
    filter->x[axis] = position[axis];
    filter->covariance[axis][axis] = config->initial_position_variance;
    filter->covariance[axis + 3][axis + 3] = config->initial_velocity_variance;
    filter->covariance[axis + 6][axis + 6] = config->initial_acceleration_variance;
  }
  filter->initialized = 1;
  return 1;
}

int dart_singer_predict(DartSingerFilter * filter, double dt_s)
{
  double axis_transition[3][3];
  double axis_noise[3][3];
  double transition[9][9] = {{0.0}};
  double noise[9][9] = {{0.0}};
  double temporary[9][9] = {{0.0}};
  double predicted_covariance[9][9] = {{0.0}};
  double predicted_state[9] = {0.0};
  if (filter == NULL || !filter->initialized || !isfinite(dt_s) || dt_s <= 0.0) return 0;
  dt_s = dart_clamp(dt_s, 1e-4, 0.2);
  axis_matrices(
    dt_s, filter->config.tau_s, filter->config.acceleration_spectral_density,
    axis_transition, axis_noise);
  for (int axis = 0; axis < 3; ++axis) {
    const int indices[3] = {axis, axis + 3, axis + 6};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        transition[indices[row]][indices[column]] = axis_transition[row][column];
        noise[indices[row]][indices[column]] = axis_noise[row][column];
      }
    }
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      predicted_state[row] += transition[row][column] * filter->x[column];
      for (int inner = 0; inner < 9; ++inner) {
        temporary[row][column] += transition[row][inner] * filter->covariance[inner][column];
      }
    }
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      predicted_covariance[row][column] = noise[row][column];
      for (int inner = 0; inner < 9; ++inner) {
        predicted_covariance[row][column] += temporary[row][inner] * transition[column][inner];
      }
    }
  }
  memcpy(filter->x, predicted_state, sizeof(predicted_state));
  memcpy(filter->covariance, predicted_covariance, sizeof(predicted_covariance));
  return dart_all_finite(filter->x, 9) && dart_all_finite(&filter->covariance[0][0], 81);
}

int dart_singer_update_position(DartSingerFilter * filter, const double position[3])
{
  double innovation[3];
  double innovation_covariance[3][3];
  double inverse[3][3];
  double gain[9][3];
  double ikh[9][9] = {{0.0}};
  double temporary[9][9] = {{0.0}};
  double updated[9][9] = {{0.0}};
  if (filter == NULL || position == NULL || !filter->initialized ||
    !dart_all_finite(position, 3)) return 0;
  for (int row = 0; row < 3; ++row) {
    innovation[row] = position[row] - filter->x[row];
    for (int column = 0; column < 3; ++column) {
      innovation_covariance[row][column] = filter->covariance[row][column];
    }
    innovation_covariance[row][row] += filter->config.position_variance;
  }
  if (!dart_inverse(&innovation_covariance[0][0], &inverse[0][0], 3)) return 0;
  filter->last_mahalanobis = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      filter->last_mahalanobis += innovation[row] * inverse[row][column] * innovation[column];
    }
  }
  if (!isfinite(filter->last_mahalanobis) ||
    filter->last_mahalanobis > filter->config.mahalanobis_limit) return 0;
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 3; ++column) {
      gain[row][column] = 0.0;
      for (int inner = 0; inner < 3; ++inner) {
        gain[row][column] += filter->covariance[row][inner] * inverse[inner][column];
      }
    }
    for (int column = 0; column < 3; ++column) filter->x[row] += gain[row][column] * innovation[column];
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      ikh[row][column] = (row == column ? 1.0 : 0.0) - (column < 3 ? gain[row][column] : 0.0);
      for (int inner = 0; inner < 9; ++inner) {
        temporary[row][column] += ikh[row][inner] * filter->covariance[inner][column];
      }
    }
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      for (int inner = 0; inner < 9; ++inner) {
        updated[row][column] += temporary[row][inner] * ikh[column][inner];
      }
      for (int inner = 0; inner < 3; ++inner) {
        updated[row][column] += gain[row][inner] * filter->config.position_variance *
          gain[column][inner];
      }
    }
  }
  memcpy(filter->covariance, updated, sizeof(updated));
  return dart_all_finite(filter->x, 9) && dart_all_finite(&filter->covariance[0][0], 81);
}

void dart_singer_predict_position(
  const DartSingerFilter * filter, double horizon_s, double position[3])
{
  double transition[3][3];
  double noise[3][3];
  if (filter == NULL || position == NULL || !filter->initialized) return;
  axis_matrices(
    horizon_s > 0.0 ? horizon_s : 0.0, filter->config.tau_s,
    filter->config.acceleration_spectral_density, transition, noise);
  for (int axis = 0; axis < 3; ++axis) {
    position[axis] = transition[0][0] * filter->x[axis] +
      transition[0][1] * filter->x[axis + 3] +
      transition[0][2] * filter->x[axis + 6];
  }
}
