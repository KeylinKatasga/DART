#ifndef DART_ALGORITHM_DART_ALGORITHM_H
#define DART_ALGORITHM_DART_ALGORITHM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DART_SELF_STATE_DIM 7
#define DART_TARGET_STATE_DIM 9
#define DART_FIN_COUNT 4

typedef struct DartImageFrame
{
  const uint8_t * data;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint64_t timestamp_ns;
  uint8_t channels;
} DartImageFrame;

typedef struct DartVisionObservation
{
  uint8_t valid;
  double timestamp_s;
  double center_x_px;
  double center_y_px;
  double radius_px;
  double position_world_m[3];
  double reprojection_error_px;
} DartVisionObservation;

typedef int (*DartVisionProcessFn)(
  const DartImageFrame * frame, DartVisionObservation * observation, void * user_context);

typedef struct DartSelfEkfConfig
{
  double mass_kg;
  double inertia_diagonal[3];
  double moment_allocation[3][DART_FIN_COUNT];
  double force_allocation[3][DART_FIN_COUNT];
  double moment_bias[3];
  double force_bias[3];
  double gravity_world[3];
  double quaternion_process_variance;
  double angular_velocity_process_variance;
  double accelerometer_variance;
  double gyroscope_variance;
  double mahalanobis_limit;
  uint8_t include_gravity_projection;
} DartSelfEkfConfig;

typedef struct DartSelfEkf
{
  DartSelfEkfConfig config;
  double x[DART_SELF_STATE_DIM];
  double covariance[DART_SELF_STATE_DIM][DART_SELF_STATE_DIM];
  double last_mahalanobis;
  uint8_t initialized;
} DartSelfEkf;

typedef struct DartSingerConfig
{
  double tau_s;
  double acceleration_spectral_density;
  double position_variance;
  double initial_position_variance;
  double initial_velocity_variance;
  double initial_acceleration_variance;
  double mahalanobis_limit;
} DartSingerConfig;

typedef struct DartSingerFilter
{
  DartSingerConfig config;
  double x[DART_TARGET_STATE_DIM];
  double covariance[DART_TARGET_STATE_DIM][DART_TARGET_STATE_DIM];
  double last_mahalanobis;
  uint8_t initialized;
} DartSingerFilter;

typedef struct DartControlConfig
{
  double line_of_sight_gain[3];
  double angular_rate_gain[3];
  double fin_allocation[DART_FIN_COUNT][3];
  double fin_neutral_rad[DART_FIN_COUNT];
  double fin_min_rad[DART_FIN_COUNT];
  double fin_max_rad[DART_FIN_COUNT];
} DartControlConfig;

typedef struct DartAlgorithm
{
  DartSelfEkf self_filter;
  DartSingerFilter target_filter;
  DartControlConfig control;
  DartVisionProcessFn vision_process;
  void * vision_context;
} DartAlgorithm;

void dart_self_ekf_default_config(DartSelfEkfConfig * config);
int dart_self_ekf_init(
  DartSelfEkf * filter, const DartSelfEkfConfig * config,
  const double quaternion_wxyz[4], const double angular_velocity_rad_s[3]);
int dart_self_ekf_step(
  DartSelfEkf * filter, double dt_s, const double fin_angles_rad[DART_FIN_COUNT],
  const double accelerometer_m_s2[3], const double gyroscope_rad_s[3]);

void dart_singer_default_config(DartSingerConfig * config);
int dart_singer_init(
  DartSingerFilter * filter, const DartSingerConfig * config, const double position_m[3]);
int dart_singer_predict(DartSingerFilter * filter, double dt_s);
int dart_singer_update_position(DartSingerFilter * filter, const double position_m[3]);
void dart_singer_predict_position(
  const DartSingerFilter * filter, double horizon_s, double position_m[3]);

void dart_control_default_config(DartControlConfig * config);
int dart_control_step(
  const DartControlConfig * config, const DartSelfEkf * self_filter,
  const DartSingerFilter * target_filter, double fin_angles_rad[DART_FIN_COUNT]);

int dart_algorithm_init(
  DartAlgorithm * algorithm, const DartSelfEkfConfig * self_config,
  const DartSingerConfig * target_config, const DartControlConfig * control_config,
  const double quaternion_wxyz[4], const double angular_velocity_rad_s[3],
  const double initial_target_position_m[3]);
void dart_algorithm_set_vision_processor(
  DartAlgorithm * algorithm, DartVisionProcessFn process, void * user_context);
int dart_algorithm_process_image(
  DartAlgorithm * algorithm, const DartImageFrame * frame, DartVisionObservation * observation);
int dart_algorithm_step(
  DartAlgorithm * algorithm, double dt_s, const double fin_feedback_rad[DART_FIN_COUNT],
  const double accelerometer_m_s2[3], const double gyroscope_rad_s[3],
  const DartVisionObservation * target_observation,
  double fin_command_rad[DART_FIN_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
