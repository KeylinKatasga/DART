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
constexpr std::size_t kReceivePayloadSize = 30;
constexpr std::size_t kSendPayloadSize = 22;

struct ReceivePayload
{
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
  float angle1{};
  float angle2{};
  float angle3{};
  float angle4{};
  std::uint8_t enable_lock{};
  std::uint8_t enable_shoot{};
  std::int32_t autoaim_living_waterfall_flag{};
};

std::array<std::uint8_t, 2> crc16(std::span<const std::uint8_t> payload);
bool verify_frame(std::span<const std::uint8_t> frame, std::size_t expected_payload_size);
std::vector<std::uint8_t> encode(const SendPayload & payload);
std::optional<ReceivePayload> decode_receive(std::span<const std::uint8_t> frame);
}  // namespace mcu_node
