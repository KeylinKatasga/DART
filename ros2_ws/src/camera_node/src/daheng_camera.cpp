#include "camera_node/daheng_camera.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "DxImageProc.h"
#include "GxIAPI.h"

namespace camera_node
{
namespace
{
std::string gx_error(GX_STATUS status)
{
  size_t size = 0;
  GX_STATUS code = status;
  if (GXGetLastError(&code, nullptr, &size) != GX_STATUS_SUCCESS || size == 0) {
    return "Galaxy SDK status " + std::to_string(status);
  }
  std::vector<char> message(size, '\0');
  if (GXGetLastError(&code, message.data(), &size) != GX_STATUS_SUCCESS) {
    return "Galaxy SDK status " + std::to_string(status);
  }
  return std::string(message.data());
}

void check(GX_STATUS status, const char * operation)
{
  if (status != GX_STATUS_SUCCESS) {
    throw std::runtime_error(std::string(operation) + ": " + gx_error(status));
  }
}

std::string get_string(GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature)
{
  size_t size = 0;
  check(GXGetStringLength(handle, feature, &size), "query string length");
  std::vector<char> value(size, '\0');
  check(GXGetString(handle, feature, value.data(), &size), "read string");
  return std::string(value.data());
}

bool implemented(GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature)
{
  bool value = false;
  check(GXIsImplemented(handle, feature, &value), "query feature support");
  return value;
}
}  // namespace

void validate_config(const CameraConfig & config)
{
  if (config.roi_width <= 0 || config.roi_height <= 0) {
    throw std::invalid_argument("roi_width and roi_height must be positive");
  }
  if (config.offset_x < 0 || config.offset_y < 0) {
    throw std::invalid_argument("ROI offsets cannot be negative");
  }
  if (config.exposure_time_us <= 0.0) {
    throw std::invalid_argument("exposure_time_us must be positive");
  }
  if (config.auto_exposure_min_us <= 0.0 ||
    config.auto_exposure_min_us > config.auto_exposure_max_us)
  {
    throw std::invalid_argument("invalid automatic exposure range");
  }
  if (config.gain < 0.0 || config.auto_gain_min < 0.0 ||
    config.auto_gain_min > config.auto_gain_max)
  {
    throw std::invalid_argument("invalid gain or automatic gain range");
  }
  if (config.gray_value < 0 || config.gray_value > 255) {
    throw std::invalid_argument("gray_value must be within [0, 255]");
  }
  if (config.grab_timeout_ms == 0) {
    throw std::invalid_argument("grab_timeout_ms must be positive");
  }
}

struct DahengCamera::Impl
{
  explicit Impl(CameraConfig value) : config(std::move(value)) {}

  CameraConfig config;
  GX_DEV_HANDLE handle{nullptr};
  bool library_open{false};
  bool streaming{false};
  std::string serial;
  std::string version;
  std::string effective_config;
  std::vector<std::uint8_t> raw;
  std::vector<std::uint8_t> bgr;
  std::int64_t payload_size{0};
  std::int64_t color_filter{0};
  std::int64_t pixel_format{0};
  std::int64_t tick_frequency{0};
  bool anchor_valid{false};
  std::uint64_t anchor_ticks{0};
  std::uint64_t previous_ticks{0};
  std::chrono::steady_clock::time_point anchor_host{};
  std::chrono::steady_clock::time_point next_latch{};

  ~Impl() {close();}

  void close() noexcept
  {
    if (streaming && handle) {
      GXStreamOff(handle);
      streaming = false;
    }
    if (handle) {
      GXCloseDevice(handle);
      handle = nullptr;
    }
    if (library_open) {
      GXCloseLib();
      library_open = false;
    }
  }

  void set_roi()
  {
    std::int64_t old_width = 0;
    std::int64_t old_height = 0;
    check(GXGetInt(handle, GX_INT_WIDTH, &old_width), "read current ROI width");
    check(GXGetInt(handle, GX_INT_HEIGHT, &old_height), "read current ROI height");
    if (old_width < config.roi_width) {
      check(GXSetInt(handle, GX_INT_OFFSET_X, config.offset_x), "set ROI offset_x");
      check(GXSetInt(handle, GX_INT_WIDTH, config.roi_width), "set ROI width");
    } else {
      check(GXSetInt(handle, GX_INT_WIDTH, config.roi_width), "set ROI width");
      check(GXSetInt(handle, GX_INT_OFFSET_X, config.offset_x), "set ROI offset_x");
    }
    if (old_height < config.roi_height) {
      check(GXSetInt(handle, GX_INT_OFFSET_Y, config.offset_y), "set ROI offset_y");
      check(GXSetInt(handle, GX_INT_HEIGHT, config.roi_height), "set ROI height");
    } else {
      check(GXSetInt(handle, GX_INT_HEIGHT, config.roi_height), "set ROI height");
      check(GXSetInt(handle, GX_INT_OFFSET_Y, config.offset_y), "set ROI offset_y");
    }
  }

  void set_exposure_gain()
  {
    if (config.auto_exposure) {
      check(GXSetEnum(handle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS),
        "enable automatic exposure");
      check(GXSetFloat(handle, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, config.auto_exposure_min_us),
        "set automatic exposure minimum");
      check(GXSetFloat(handle, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, config.auto_exposure_max_us),
        "set automatic exposure maximum");
    } else {
      check(GXSetEnum(handle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED),
        "set timed exposure mode");
      check(GXSetEnum(handle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF),
        "disable automatic exposure");
      check(GXSetFloat(handle, GX_FLOAT_EXPOSURE_TIME, config.exposure_time_us),
        "set exposure time");
    }
    if (config.auto_gain) {
      check(GXSetEnum(handle, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS),
        "enable automatic gain");
      check(GXSetFloat(handle, GX_FLOAT_AUTO_GAIN_MIN, config.auto_gain_min),
        "set automatic gain minimum");
      check(GXSetFloat(handle, GX_FLOAT_AUTO_GAIN_MAX, config.auto_gain_max),
        "set automatic gain maximum");
    } else {
      check(GXSetEnum(handle, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF),
        "disable automatic gain");
      check(GXSetEnum(handle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL),
        "select all gain channels");
      check(GXSetFloat(handle, GX_FLOAT_GAIN, config.gain), "set gain");
    }
    if (implemented(handle, GX_INT_GRAY_VALUE)) {
      check(GXSetInt(handle, GX_INT_GRAY_VALUE, config.gray_value), "set gray target");
    }
  }

  void set_white_balance()
  {
    if (!implemented(handle, GX_ENUM_BALANCE_WHITE_AUTO)) {
      if (config.white_balance) {
        throw std::runtime_error("camera does not support automatic white balance");
      }
      return;
    }
    check(GXSetEnum(
        handle, GX_ENUM_BALANCE_WHITE_AUTO,
        config.white_balance ? GX_BALANCE_WHITE_AUTO_CONTINUOUS : GX_BALANCE_WHITE_AUTO_OFF),
      "set white balance");
  }

  void latch()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_latch) return;
    next_latch = now + std::chrono::seconds(5);
    if (tick_frequency <= 0 || !implemented(handle, GX_COMMAND_TIMESTAMP_LATCH)) return;
    const auto before = std::chrono::steady_clock::now();
    check(GXSendCommand(handle, GX_COMMAND_TIMESTAMP_LATCH), "latch device timestamp");
    std::int64_t ticks = 0;
    check(GXGetInt(handle, GX_INT_TIMESTAMP_LATCH_VALUE, &ticks), "read latched timestamp");
    const auto after = std::chrono::steady_clock::now();
    if (ticks > 0) {
      anchor_ticks = static_cast<std::uint64_t>(ticks);
      anchor_host = before + (after - before) / 2;
      anchor_valid = true;
    }
  }

  std::chrono::steady_clock::time_point map_time(std::uint64_t ticks, std::string & source)
  {
    if (ticks == 0 || tick_frequency <= 0 || !anchor_valid) {
      source = "host_receive";
      return {};
    }
    if (previous_ticks != 0 && ticks < previous_ticks) {
      anchor_valid = false;
      source = "host_receive_counter_reset";
      return {};
    }
    const auto delta = static_cast<std::int64_t>(ticks) - static_cast<std::int64_t>(anchor_ticks);
    const auto nanoseconds = static_cast<std::int64_t>(
      static_cast<long double>(delta) * 1000000000.0L /
      static_cast<long double>(tick_frequency));
    source = "device_latch";
    return anchor_host + std::chrono::nanoseconds(nanoseconds);
  }
};

DahengCamera::DahengCamera(const CameraConfig & config)
: impl_(std::make_unique<Impl>(config))
{
  validate_config(config);
  try {
    check(GXInitLib(), "initialize Galaxy SDK");
    impl_->library_open = true;
    impl_->version = GXGetLibVersion() ? GXGetLibVersion() : "unknown";

    if (config.camera_sn.empty()) {
      std::uint32_t count = 0;
      check(GXUpdateDeviceList(&count, 1000), "enumerate cameras");
      if (count == 0) throw std::runtime_error("no Daheng camera found");
      check(GXOpenDeviceByIndex(1, &impl_->handle), "open first camera");
    } else {
      GX_OPEN_PARAM open{};
      open.accessMode = GX_ACCESS_EXCLUSIVE;
      open.openMode = GX_OPEN_SN;
      open.pszContent = const_cast<char *>(config.camera_sn.c_str());
      check(GXOpenDevice(&open, &impl_->handle), "open camera by serial number");
    }

    impl_->serial = get_string(impl_->handle, GX_STRING_DEVICE_SERIAL_NUMBER);
    if (!implemented(impl_->handle, GX_ENUM_PIXEL_COLOR_FILTER)) {
      throw std::runtime_error("only color Bayer cameras are supported");
    }
    impl_->set_roi();
    impl_->set_exposure_gain();
    impl_->set_white_balance();
    std::int64_t width = 0, height = 0, offset_x = 0, offset_y = 0;
    double exposure = 0.0, gain = 0.0;
    check(GXGetInt(impl_->handle, GX_INT_WIDTH, &width), "read effective ROI width");
    check(GXGetInt(impl_->handle, GX_INT_HEIGHT, &height), "read effective ROI height");
    check(GXGetInt(impl_->handle, GX_INT_OFFSET_X, &offset_x), "read effective ROI offset_x");
    check(GXGetInt(impl_->handle, GX_INT_OFFSET_Y, &offset_y), "read effective ROI offset_y");
    check(GXGetFloat(impl_->handle, GX_FLOAT_EXPOSURE_TIME, &exposure),
      "read effective exposure");
    check(GXGetFloat(impl_->handle, GX_FLOAT_GAIN, &gain), "read effective gain");
    std::ostringstream effective;
    effective << "roi=" << width << 'x' << height << '+' << offset_x << '+' << offset_y
              << " exposure=" << (config.auto_exposure ? "auto:" : "manual:") << exposure << "us"
              << " gain=" << (config.auto_gain ? "auto:" : "manual:") << gain
              << " white_balance=" << (config.white_balance ? "on" : "off")
              << " flip=" << (config.flip_image ? "180deg" : "off");
    impl_->effective_config = effective.str();
    check(GXGetEnum(impl_->handle, GX_ENUM_PIXEL_COLOR_FILTER, &impl_->color_filter),
      "read Bayer layout");
    check(GXGetEnum(impl_->handle, GX_ENUM_PIXEL_FORMAT, &impl_->pixel_format),
      "read pixel format");
    const bool bayer8 = impl_->pixel_format == GX_PIXEL_FORMAT_BAYER_GR8 ||
      impl_->pixel_format == GX_PIXEL_FORMAT_BAYER_RG8 ||
      impl_->pixel_format == GX_PIXEL_FORMAT_BAYER_GB8 ||
      impl_->pixel_format == GX_PIXEL_FORMAT_BAYER_BG8;
    if (!bayer8) throw std::runtime_error("camera pixel format is not supported Bayer8");

    check(GXSetEnum(impl_->handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS),
      "set continuous acquisition");
    check(GXGetInt(impl_->handle, GX_INT_PAYLOAD_SIZE, &impl_->payload_size),
      "read payload size");
    if (impl_->payload_size <= 0) throw std::runtime_error("invalid camera payload size");
    impl_->raw.resize(static_cast<std::size_t>(impl_->payload_size));
    impl_->bgr.resize(
      static_cast<std::size_t>(config.roi_width * config.roi_height * 3));
    check(GXStreamOn(impl_->handle), "start camera stream");
    impl_->streaming = true;

    if (implemented(impl_->handle, GX_INT_TIMESTAMP_TICK_FREQUENCY)) {
      check(GXGetInt(
          impl_->handle, GX_INT_TIMESTAMP_TICK_FREQUENCY, &impl_->tick_frequency),
        "read timestamp frequency");
    }
    impl_->latch();
  } catch (...) {
    impl_->close();
    throw;
  }
}

DahengCamera::~DahengCamera() = default;

GrabResult DahengCamera::grab(CameraFrame & frame)
{
  GX_FRAME_DATA raw_frame{};
  raw_frame.pImgBuf = impl_->raw.data();
  raw_frame.nImgSize = impl_->payload_size;
  const auto status = GXGetImage(impl_->handle, &raw_frame, impl_->config.grab_timeout_ms);
  if (status == GX_STATUS_TIMEOUT) return GrabResult::timeout;
  check(status, "grab image");
  if (raw_frame.nStatus != GX_FRAME_STATUS_SUCCESS) {
    throw std::runtime_error("camera returned an incomplete frame");
  }

  const auto received = std::chrono::steady_clock::now();
  const auto required = static_cast<std::size_t>(raw_frame.nWidth) * raw_frame.nHeight * 3;
  impl_->bgr.resize(required);
  const auto conversion_start = std::chrono::steady_clock::now();
  const auto conversion = DxRaw8toRGB24Ex(
    raw_frame.pImgBuf, impl_->bgr.data(), raw_frame.nWidth, raw_frame.nHeight,
    RAW2RGB_NEIGHBOUR3, static_cast<DX_PIXEL_COLOR_FILTER>(impl_->color_filter),
    false, DX_ORDER_BGR);
  if (conversion != DX_OK) {
    throw std::runtime_error("Bayer8 to BGR conversion failed: " + std::to_string(conversion));
  }
  if (impl_->config.flip_image) {
    cv::Mat image(raw_frame.nHeight, raw_frame.nWidth, CV_8UC3, impl_->bgr.data());
    cv::flip(image, image, -1);
  }

  impl_->latch();
  frame.width = raw_frame.nWidth;
  frame.height = raw_frame.nHeight;
  frame.bgr = impl_->bgr;
  frame.sequence = raw_frame.nFrameID;
  frame.device_timestamp_ticks = raw_frame.nTimestamp;
  frame.device_tick_frequency_hz = static_cast<double>(impl_->tick_frequency);
  frame.host_receive_time = received;
  frame.acquisition_time = impl_->map_time(raw_frame.nTimestamp, frame.time_source);
  if (frame.acquisition_time == std::chrono::steady_clock::time_point{}) {
    frame.acquisition_time = received;
  }
  frame.conversion_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - conversion_start).count();
  impl_->previous_ticks = raw_frame.nTimestamp;
  return GrabResult::frame;
}

std::string DahengCamera::serial_number() const {return impl_->serial;}
std::string DahengCamera::sdk_version() const {return impl_->version;}
std::string DahengCamera::configuration_summary() const {return impl_->effective_config;}

std::unique_ptr<Camera> make_daheng_camera(const CameraConfig & config)
{
  return std::make_unique<DahengCamera>(config);
}
}  // namespace camera_node

