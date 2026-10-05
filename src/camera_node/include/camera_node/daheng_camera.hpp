#pragma once

#include <memory>

#include "camera_node/camera.hpp"

namespace camera_node
{
class DahengCamera final : public Camera
{
public:
  explicit DahengCamera(const CameraConfig & config);
  ~DahengCamera() override;
  DahengCamera(const DahengCamera &) = delete;
  DahengCamera & operator=(const DahengCamera &) = delete;

  GrabResult grab(CameraFrame & frame) override;
  std::string serial_number() const override;
  std::string sdk_version() const override;
  std::string configuration_summary() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace camera_node

