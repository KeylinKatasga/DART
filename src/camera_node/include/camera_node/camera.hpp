#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace camera_node
{
struct CameraConfig
{
  std::string camera_sn;
  std::int64_t roi_width{1280};
  std::int64_t roi_height{1024};
  std::int64_t offset_x{0};
  std::int64_t offset_y{0};
  bool auto_exposure{false};
  double exposure_time_us{1700.0};
  double auto_exposure_min_us{500.0};
  double auto_exposure_max_us{10000.0};
  bool auto_gain{false};
  double gain{6.0};
  double auto_gain_min{0.0};
  double auto_gain_max{10.0};
  std::int64_t gray_value{200};
  bool white_balance{true};
  bool flip_image{false};
  std::uint32_t grab_timeout_ms{500};
};

struct CameraFrame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::uint8_t> bgr;
  std::uint64_t sequence{0};
  std::uint64_t device_timestamp_ticks{0};
  double device_tick_frequency_hz{0.0};
  std::chrono::steady_clock::time_point host_receive_time{};
  std::chrono::steady_clock::time_point acquisition_time{};
  std::string time_source{"host_receive"};
  double conversion_ms{0.0};
};

enum class GrabResult { frame, timeout };

class Camera
{
public:
  virtual ~Camera() = default;
  virtual GrabResult grab(CameraFrame & frame) = 0;
  virtual std::string serial_number() const = 0;
  virtual std::string sdk_version() const = 0;
  virtual std::string configuration_summary() const {return {};}
};

void validate_config(const CameraConfig & config);
std::unique_ptr<Camera> make_daheng_camera(const CameraConfig & config);
}  // namespace camera_node

