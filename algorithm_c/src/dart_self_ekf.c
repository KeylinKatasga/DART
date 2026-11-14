#include "dart_algorithm/dart_algorithm.h"
#include "dart_internal.h"

#include <math.h>
#include <string.h>

static void transition_model(
  const DartSelfEkfConfig * config, const double state[7], double dt,
  const double fins[4], double output[7])
{
  double moment[3];
  const double wx = state[4];
  const double wy = state[5];
  const double wz = state[6];
  for (int axis = 0; axis < 3; ++axis) {
    moment[axis] = config->moment_bias[axis];
    for (int fin = 0; fin < 4; ++fin) moment[axis] += config->moment_allocation[axis][fin] * fins[fin];
  }
  output[0] = state[0] + 0.5 * dt * (-state[1] * wx - state[2] * wy - state[3] * wz);
  output[1] = state[1] + 0.5 * dt * (state[0] * wx + state[2] * wz - state[3] * wy);
  output[2] = state[2] + 0.5 * dt * (state[0] * wy + state[3] * wx - state[1] * wz);
  output[3] = state[3] + 0.5 * dt * (state[0] * wz + state[1] * wy - state[2] * wx);
  dart_quaternion_normalize(output);
  output[4] = wx + dt * (moment[0] - wy * config->inertia_diagonal[2] * wz +
    wz * config->inertia_diagonal[1] * wy) / config->inertia_diagonal[0];
  output[5] = wy + dt * (moment[1] - wz * config->inertia_diagonal[0] * wx +
    wx * config->inertia_diagonal[2] * wz) / config->inertia_diagonal[1];
  output[6] = wz + dt * (moment[2] - wx * config->inertia_diagonal[1] * wy +
    wy * config->inertia_diagonal[0] * wx) / config->inertia_diagonal[2];
}

static void measurement_model(
  const DartSelfEkfConfig * config, const double state[7], const double fins[4],
  double output[6])
{
  double gravity_body[3] = {0.0};
  if (config->include_gravity_projection) {
    dart_rotate_world_to_body(state, config->gravity_world, gravity_body);
  }
  for (int axis = 0; axis < 3; ++axis) {
    double force = config->force_bias[axis];
    for (int fin = 0; fin < 4; ++fin) force += config->force_allocation[axis][fin] * fins[fin];
    output[axis] = force / config->mass_kg + gravity_body[axis];
    output[axis + 3] = state[axis + 4];
  }
}

void dart_self_ekf_default_config(DartSelfEkfConfig * config)
{
  if (config == NULL) return;
  memset(config, 0, sizeof(*config));
  config->mass_kg = 1.0;
  config->inertia_diagonal[0] = config->inertia_diagonal[1] = config->inertia_diagonal[2] = 1.0;
  config->gravity_world[2] = -9.80665;
  config->quaternion_process_variance = 1e-6;
  config->angular_velocity_process_variance = 1e-3;
  config->accelerometer_variance = 0.25;
  config->gyroscope_variance = 1e-3;
  config->mahalanobis_limit = 30.0;
  config->include_gravity_projection = 1;
}

int dart_self_ekf_init(
  DartSelfEkf * filter, const DartSelfEkfConfig * config,
  const double quaternion[4], const double angular_velocity[3])
{
  if (filter == NULL || config == NULL || quaternion == NULL || angular_velocity == NULL ||
    config->mass_kg <= 0.0 || config->inertia_diagonal[0] <= 0.0 ||
    config->inertia_diagonal[1] <= 0.0 || config->inertia_diagonal[2] <= 0.0 ||
    !dart_all_finite(quaternion, 4) || !dart_all_finite(angular_velocity, 3)) return 0;
  memset(filter, 0, sizeof(*filter));
  filter->config = *config;
  memcpy(filter->x, quaternion, sizeof(double) * 4);
  memcpy(filter->x + 4, angular_velocity, sizeof(double) * 3);
  dart_quaternion_normalize(filter->x);
  for (int index = 0; index < 7; ++index) {
    filter->covariance[index][index] = index < 4 ? 0.01 : 0.1;
  }
  filter->initialized = 1;
  return 1;
}

int dart_self_ekf_step(
  DartSelfEkf * filter, double dt_s, const double fins[4],
  const double accelerometer[3], const double gyroscope[3])
{
  double predicted[7];
  double jacobian[7][7];
  double temporary[7][7] = {{0.0}};
  double predicted_covariance[7][7] = {{0.0}};
  double predicted_measurement[6];
  double h[6][7];
  double innovation[6];
  double s[6][6] = {{0.0}};
  double s_inverse[6][6];
  double gain[7][6] = {{0.0}};
  double ikh[7][7] = {{0.0}};
  double joseph_left[7][7] = {{0.0}};
  double updated_covariance[7][7] = {{0.0}};
  const double epsilon = 1e-6;
  if (filter == NULL || fins == NULL || accelerometer == NULL || gyroscope == NULL ||
    !filter->initialized || !isfinite(dt_s) || dt_s <= 0.0 ||
    !dart_all_finite(accelerometer, 3) || !dart_all_finite(gyroscope, 3)) return 0;
  dt_s = dart_clamp(dt_s, 1e-4, 0.05);
  transition_model(&filter->config, filter->x, dt_s, fins, predicted);
  for (int column = 0; column < 7; ++column) {
    double plus[7];
    double minus[7];
    double plus_output[7];
    double minus_output[7];
    memcpy(plus, filter->x, sizeof(plus));
    memcpy(minus, filter->x, sizeof(minus));
    plus[column] += epsilon;
    minus[column] -= epsilon;
    transition_model(&filter->config, plus, dt_s, fins, plus_output);
    transition_model(&filter->config, minus, dt_s, fins, minus_output);
    for (int row = 0; row < 7; ++row) jacobian[row][column] = (plus_output[row] - minus_output[row]) / (2.0 * epsilon);
  }
  for (int row = 0; row < 7; ++row) {
    for (int column = 0; column < 7; ++column) {
      for (int inner = 0; inner < 7; ++inner) temporary[row][column] += jacobian[row][inner] * filter->covariance[inner][column];
    }
  }
  for (int row = 0; row < 7; ++row) {
    for (int column = 0; column < 7; ++column) {
      for (int inner = 0; inner < 7; ++inner) predicted_covariance[row][column] += temporary[row][inner] * jacobian[column][inner];
      if (row == column) predicted_covariance[row][column] += dt_s * (row < 4 ?
        filter->config.quaternion_process_variance : filter->config.angular_velocity_process_variance);
    }
  }
  measurement_model(&filter->config, predicted, fins, predicted_measurement);
  for (int column = 0; column < 7; ++column) {
    double plus[7];
    double minus[7];
    double plus_output[6];
    double minus_output[6];
    memcpy(plus, predicted, sizeof(plus));
    memcpy(minus, predicted, sizeof(minus));
    plus[column] += epsilon;
    minus[column] -= epsilon;
    measurement_model(&filter->config, plus, fins, plus_output);
    measurement_model(&filter->config, minus, fins, minus_output);
    for (int row = 0; row < 6; ++row) h[row][column] = (plus_output[row] - minus_output[row]) / (2.0 * epsilon);
  }
  for (int row = 0; row < 6; ++row) {
    innovation[row] = (row < 3 ? accelerometer[row] : gyroscope[row - 3]) - predicted_measurement[row];
    for (int column = 0; column < 6; ++column) {
      for (int state_row = 0; state_row < 7; ++state_row) {
        for (int state_column = 0; state_column < 7; ++state_column) {
          s[row][column] += h[row][state_row] * predicted_covariance[state_row][state_column] * h[column][state_column];
        }
      }
    }
    s[row][row] += row < 3 ? filter->config.accelerometer_variance : filter->config.gyroscope_variance;
  }
  if (!dart_inverse(&s[0][0], &s_inverse[0][0], 6)) return 0;
  filter->last_mahalanobis = 0.0;
  for (int row = 0; row < 6; ++row) for (int column = 0; column < 6; ++column)
    filter->last_mahalanobis += innovation[row] * s_inverse[row][column] * innovation[column];
  if (!isfinite(filter->last_mahalanobis) || filter->last_mahalanobis > filter->config.mahalanobis_limit) return 0;
  for (int row = 0; row < 7; ++row) for (int column = 0; column < 6; ++column)
    for (int measurement_index = 0; measurement_index < 6; ++measurement_index)
      for (int state_index = 0; state_index < 7; ++state_index)
        gain[row][column] += predicted_covariance[row][state_index] * h[measurement_index][state_index] * s_inverse[measurement_index][column];
  for (int row = 0; row < 7; ++row) {
    filter->x[row] = predicted[row];
    for (int column = 0; column < 6; ++column) filter->x[row] += gain[row][column] * innovation[column];
  }
  dart_quaternion_normalize(filter->x);
  for (int row = 0; row < 7; ++row) for (int column = 0; column < 7; ++column) {
    ikh[row][column] = row == column ? 1.0 : 0.0;
    for (int measurement_index = 0; measurement_index < 6; ++measurement_index)
      ikh[row][column] -= gain[row][measurement_index] * h[measurement_index][column];
  }
  for (int row = 0; row < 7; ++row) for (int column = 0; column < 7; ++column)
    for (int inner = 0; inner < 7; ++inner) joseph_left[row][column] += ikh[row][inner] * predicted_covariance[inner][column];
  for (int row = 0; row < 7; ++row) for (int column = 0; column < 7; ++column) {
    for (int inner = 0; inner < 7; ++inner) updated_covariance[row][column] += joseph_left[row][inner] * ikh[column][inner];
    for (int measurement_index = 0; measurement_index < 6; ++measurement_index) {
      const double variance = measurement_index < 3 ? filter->config.accelerometer_variance : filter->config.gyroscope_variance;
      updated_covariance[row][column] += gain[row][measurement_index] * variance * gain[column][measurement_index];
    }
  }
  memcpy(filter->covariance, updated_covariance, sizeof(updated_covariance));
  return dart_all_finite(filter->x, 7) && dart_all_finite(&filter->covariance[0][0], 49);
}
