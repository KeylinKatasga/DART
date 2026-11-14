#include "dart_algorithm/dart_algorithm.h"

#include <string.h>

int dart_algorithm_init(
  DartAlgorithm * algorithm, const DartSelfEkfConfig * self_config,
  const DartSingerConfig * target_config, const DartControlConfig * control_config,
  const double quaternion[4], const double angular_velocity[3],
  const double initial_target_position[3])
{
  if (algorithm == NULL || control_config == NULL) return 0;
  memset(algorithm, 0, sizeof(*algorithm));
  algorithm->control = *control_config;
  return dart_self_ekf_init(
    &algorithm->self_filter, self_config, quaternion, angular_velocity) &&
    dart_singer_init(&algorithm->target_filter, target_config, initial_target_position);
}

void dart_algorithm_set_vision_processor(
  DartAlgorithm * algorithm, DartVisionProcessFn process, void * user_context)
{
  if (algorithm == NULL) return;
  algorithm->vision_process = process;
  algorithm->vision_context = user_context;
}

int dart_algorithm_process_image(
  DartAlgorithm * algorithm, const DartImageFrame * frame, DartVisionObservation * observation)
{
  if (algorithm == NULL || frame == NULL || observation == NULL ||
    algorithm->vision_process == NULL) return 0;
  return algorithm->vision_process(frame, observation, algorithm->vision_context);
}

int dart_algorithm_step(
  DartAlgorithm * algorithm, double dt_s, const double fin_feedback[4],
  const double accelerometer[3], const double gyroscope[3],
  const DartVisionObservation * observation, double fin_command[4])
{
  int self_valid;
  int target_valid;
  if (algorithm == NULL || fin_command == NULL) return 0;
  self_valid = dart_self_ekf_step(
    &algorithm->self_filter, dt_s, fin_feedback, accelerometer, gyroscope);
  target_valid = dart_singer_predict(&algorithm->target_filter, dt_s);
  if (observation != NULL && observation->valid) {
    target_valid = dart_singer_update_position(
      &algorithm->target_filter, observation->position_world_m);
  }
  if (!self_valid || !target_valid) return 0;
  return dart_control_step(
    &algorithm->control, &algorithm->self_filter, &algorithm->target_filter, fin_command);
}
