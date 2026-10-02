#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace mcu_node
{
constexpr std::uint8_t kFrameHead = 0x3e;
constexpr std::size_t kReceivePayloadSize = 66;
constexpr std::size_t kSendPayloadSize = 30;

struct ReceivePayload
{
  float imu_yaw{};
  float imu_pitch{};
  float imu_roll{};
  float imu_yaw_vel{};
  float imu_pitch_vel{};
  float imu_roll_vel{};
  float imu_yaw_acc{};
  float imu_pitch_acc{};
  float imu_roll_acc{};
  float imu_quat_w{};
  float imu_quat_x{};
  float imu_quat_y{};
  float imu_quat_z{};
  std::int16_t order{};
  std::int16_t remaining_time_s{};
  std::int8_t sentry_behavior_mode{};
  std::uint8_t colour{};
  std::uint64_t system_time_point_us{};
};

struct SendPayload
{
  float yaw{-1.0F};
  float yaw_vel{-1.0F};
  float yaw_acc{-1.0F};
  float pitch{-1.0F};
  float pitch_vel{-1.0F};
  float pitch_acc{-1.0F};
  std::uint8_t enable_lock{};
  std::uint8_t enable_shoot{};
  std::int32_t autoaim_living_waterfall_flag{};
};

std::array<std::uint8_t, 2> crc16(std::span<const std::uint8_t> payload);
bool verify_frame(std::span<const std::uint8_t> frame, std::size_t expected_payload_size);
std::vector<std::uint8_t> encode(const SendPayload & payload);
std::optional<ReceivePayload> decode_receive(std::span<const std::uint8_t> frame);
}  // namespace mcu_node

