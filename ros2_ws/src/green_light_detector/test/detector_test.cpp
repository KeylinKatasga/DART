#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "green_light_detector/detector.hpp"

namespace
{
using green_light_detector::Detector;
using green_light_detector::DetectorConfig;

TEST(DetectorTest, FindsGreenCircle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::circle(image, {321, 203}, 42, {0, 255, 0}, cv::FILLED, cv::LINE_AA);
  const auto result = Detector(DetectorConfig{}).detect(image);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->center.x, 321.0, 1.0);
  EXPECT_NEAR(result->center.y, 203.0, 1.0);
  EXPECT_NEAR(result->radius, 42.0, 2.0);
  EXPECT_GT(result->circularity, 0.8);
}

TEST(DetectorTest, RejectsRedCircle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::circle(image, {320, 240}, 50, {0, 0, 255}, cv::FILLED);
  EXPECT_FALSE(Detector(DetectorConfig{}).detect(image).has_value());
}

TEST(DetectorTest, RejectsElongatedGreenShape)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(image, {100, 200}, {500, 230}, {0, 255, 0}, cv::FILLED);
  EXPECT_FALSE(Detector(DetectorConfig{}).detect(image).has_value());
}

TEST(DetectorTest, ChoosesBestQualifiedCircle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::circle(image, {100, 100}, 12, {0, 255, 0}, cv::FILLED);
  cv::circle(image, {400, 300}, 45, {0, 255, 0}, cv::FILLED);
  const auto result = Detector(DetectorConfig{}).detect(image);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->center.x, 400.0, 1.0);
  EXPECT_NEAR(result->center.y, 300.0, 1.0);
}
}  // namespace
