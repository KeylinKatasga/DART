#include "dart_algorithm/dart_algorithm.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int fake_vision(
  const DartImageFrame * frame, DartVisionObservation * observation, void * context)
{
  (void)context;
  observation->valid = frame->data != NULL;
  observation->position_world_m[0] = 5.0;
  return observation->valid;
}

int main(void)
{
  DartSelfEkfConfig self_config;
  DartSingerConfig singer_config;
  DartControlConfig control_config;
  DartAlgorithm algorithm;
  const double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
  const double zero3[3] = {0.0, 0.0, 0.0};
  const double target[3] = {5.0, 0.0, 0.0};
  const double fins[4] = {0.0, 0.0, 0.0, 0.0};
  const double acceleration[3] = {0.0, 0.0, -9.80665};
  double command[4];
  DartVisionObservation observation = {0};
  uint8_t pixel = 1;
  DartImageFrame image = {&pixel, 1, 1, 1, 0, 1};
  dart_self_ekf_default_config(&self_config);
  self_config.mahalanobis_limit = 1000.0;
  dart_singer_default_config(&singer_config);
  dart_control_default_config(&control_config);
  control_config.fin_allocation[0][1] = 1.0;
  control_config.fin_allocation[1][1] = -1.0;
  control_config.fin_allocation[2][2] = 1.0;
  control_config.fin_allocation[3][2] = -1.0;
  assert(dart_algorithm_init(
    &algorithm, &self_config, &singer_config, &control_config, quaternion, zero3, target));
  dart_algorithm_set_vision_processor(&algorithm, fake_vision, NULL);
  assert(dart_algorithm_process_image(&algorithm, &image, &observation));
  assert(dart_algorithm_step(
    &algorithm, 0.01, fins, acceleration, zero3, &observation, command));
  assert(fabs(
    algorithm.self_filter.x[0] * algorithm.self_filter.x[0] +
    algorithm.self_filter.x[1] * algorithm.self_filter.x[1] +
    algorithm.self_filter.x[2] * algorithm.self_filter.x[2] +
    algorithm.self_filter.x[3] * algorithm.self_filter.x[3] - 1.0) < 1e-9);
  for (int index = 0; index < 4; ++index) assert(isfinite(command[index]));
  puts("dart_algorithm_c tests passed");
  return 0;
}
