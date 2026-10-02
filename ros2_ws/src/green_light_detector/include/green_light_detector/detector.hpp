#pragma once

#include <optional>

#include <opencv2/core.hpp>

namespace green_light_detector
{

struct DetectorConfig
{
  int hue_min{35};
  int hue_max{95};
  int saturation_min{70};
  int value_min{100};
  double min_area_px{40.0};
  double max_area_px{1000000.0};
  double min_radius_px{3.0};
  double max_radius_px{1000.0};
  double min_circularity{0.65};
  double min_fill_ratio{0.45};
  double max_aspect_ratio{1.35};
  int morphology_kernel{5};
  int open_iterations{1};
  int close_iterations{2};
};

struct CircleDetection
{
  cv::Point2f center;
  float radius{0.0F};
  float circularity{0.0F};
  float fill_ratio{0.0F};
  float score{0.0F};
};

class Detector
{
public:
  explicit Detector(DetectorConfig config);

  [[nodiscard]] std::optional<CircleDetection> detect(
    const cv::Mat & bgr, cv::Mat * output_mask = nullptr) const;
  [[nodiscard]] const DetectorConfig & config() const noexcept {return config_;}

private:
  DetectorConfig config_;
};

}  // namespace green_light_detector
