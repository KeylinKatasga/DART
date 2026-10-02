#include "green_light_detector/detector.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace green_light_detector
{

Detector::Detector(DetectorConfig config) : config_(std::move(config))
{
  if (config_.hue_min < 0 || config_.hue_max > 179 ||
    config_.hue_min > config_.hue_max || config_.saturation_min < 0 ||
    config_.saturation_min > 255 || config_.value_min < 0 || config_.value_min > 255 ||
    config_.min_area_px <= 0.0 || config_.max_area_px < config_.min_area_px ||
    config_.min_radius_px <= 0.0 || config_.max_radius_px < config_.min_radius_px ||
    config_.min_circularity <= 0.0 || config_.min_circularity > 1.0 ||
    config_.min_fill_ratio <= 0.0 || config_.min_fill_ratio > 1.0 ||
    config_.max_aspect_ratio < 1.0 || config_.morphology_kernel <= 0 ||
    config_.morphology_kernel % 2 == 0 || config_.open_iterations < 0 ||
    config_.close_iterations < 0)
  {
    throw std::invalid_argument("invalid green-light detector configuration");
  }
}

std::optional<CircleDetection> Detector::detect(
  const cv::Mat & bgr, cv::Mat * output_mask) const
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("detector input must be a non-empty CV_8UC3 BGR image");
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(config_.hue_min, config_.saturation_min, config_.value_min),
    cv::Scalar(config_.hue_max, 255, 255), mask);

  const auto kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, {config_.morphology_kernel, config_.morphology_kernel});
  if (config_.open_iterations > 0) {
    cv::morphologyEx(
      mask, mask, cv::MORPH_OPEN, kernel, {-1, -1}, config_.open_iterations);
  }
  if (config_.close_iterations > 0) {
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE, kernel, {-1, -1}, config_.close_iterations);
  }
  if (output_mask != nullptr) {
    mask.copyTo(*output_mask);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::optional<CircleDetection> best;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < config_.min_area_px || area > config_.max_area_px) continue;
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0.0) continue;
    const double circularity = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
    if (circularity < config_.min_circularity) continue;

    cv::Point2f center;
    float radius = 0.0F;
    cv::minEnclosingCircle(contour, center, radius);
    if (radius < config_.min_radius_px || radius > config_.max_radius_px) continue;
    const double circle_area = std::numbers::pi * radius * radius;
    const double fill_ratio = area / circle_area;
    if (fill_ratio < config_.min_fill_ratio) continue;

    const auto box = cv::boundingRect(contour);
    const double aspect_ratio = static_cast<double>(std::max(box.width, box.height)) /
      static_cast<double>(std::max(1, std::min(box.width, box.height)));
    if (aspect_ratio > config_.max_aspect_ratio) continue;

    const float score = static_cast<float>(
      std::clamp(circularity, 0.0, 1.0) * std::clamp(fill_ratio, 0.0, 1.0) *
      std::sqrt(area));
    CircleDetection candidate{
      center, radius, static_cast<float>(circularity), static_cast<float>(fill_ratio), score};
    if (!best || candidate.score > best->score) best = candidate;
  }
  return best;
}

}  // namespace green_light_detector
