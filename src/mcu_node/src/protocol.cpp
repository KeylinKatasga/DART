#include "mcu_node/protocol.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace mcu_node
{
namespace
{
template<typename T>
void append_le(std::vector<std::uint8_t> & out, T value)
{
  static_assert(std::is_trivially_copyable_v<T>);
  auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);
  if constexpr (std::endian::native == std::endian::big) {
    for (auto it = bytes.rbegin(); it != bytes.rend(); ++it) {
      out.push_back(*it);
    }
  } else {
    out.insert(out.end(), bytes.begin(), bytes.end());
  }
}

template<typename T>
T read_le(std::span<const std::uint8_t> bytes, std::size_t & offset)
{
  if (offset + sizeof(T) > bytes.size()) {throw std::out_of_range("short gimbal payload");}
  std::array<std::uint8_t, sizeof(T)> raw{};
  std::memcpy(raw.data(), bytes.data() + offset, sizeof(T));
  offset += sizeof(T);
  if constexpr (std::endian::native == std::endian::big) {
    std::reverse(raw.begin(), raw.end());
  }
  return std::bit_cast<T>(raw);
}

std::vector<std::uint8_t> make_frame(std::span<const std::uint8_t> payload)
{
  if (payload.size() > 255) {throw std::length_error("gimbal payload is too large");}
  std::vector<std::uint8_t> frame{kFrameHead, static_cast<std::uint8_t>(payload.size())};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const auto check = crc16(payload);
  frame.insert(frame.end(), check.begin(), check.end());
  return frame;
}
}  // namespace

std::array<std::uint8_t, 2> crc16(std::span<const std::uint8_t> payload)
{
  std::uint16_t crc = 0xffff;
  for (const auto byte : payload) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xa001U) :
        static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  // Match the current sp_-vpie Crc16::Encode() wire order: the MCU stores its
  // uint16 CRC in little-endian order, low byte first and high byte second.
  return {static_cast<std::uint8_t>(crc & 0xffU), static_cast<std::uint8_t>(crc >> 8U)};
}

bool verify_frame(std::span<const std::uint8_t> frame, const std::size_t expected_payload_size)
{
  if (frame.size() != expected_payload_size + 4 || frame[0] != kFrameHead ||
    frame[1] != expected_payload_size) {return false;}
  const auto payload = frame.subspan(2, expected_payload_size);
  const auto check = crc16(payload);
  return frame[expected_payload_size + 2] == check[0] &&
         frame[expected_payload_size + 3] == check[1];
}

std::vector<std::uint8_t> encode(const SendPayload & p)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(kSendPayloadSize);
  append_le(payload, p.yaw);
  append_le(payload, p.yaw_vel);
  append_le(payload, p.yaw_acc);
  append_le(payload, p.pitch);
  append_le(payload, p.pitch_vel);
  append_le(payload, p.pitch_acc);
  append_le(payload, p.enable_lock);
  append_le(payload, p.enable_shoot);
  append_le(payload, p.autoaim_living_waterfall_flag);
  if (payload.size() != kSendPayloadSize) {throw std::logic_error("invalid send payload layout");}
  return make_frame(payload);
}

std::optional<ReceivePayload> decode_receive(std::span<const std::uint8_t> frame)
{
  if (!verify_frame(frame, kReceivePayloadSize)) {return std::nullopt;}
  const auto bytes = frame.subspan(2, kReceivePayloadSize);
  std::size_t offset = 0;
  ReceivePayload p;
  p.imu_yaw = read_le<float>(bytes, offset);
  p.imu_pitch = read_le<float>(bytes, offset);
  p.imu_roll = read_le<float>(bytes, offset);
  p.imu_yaw_vel = read_le<float>(bytes, offset);
  p.imu_pitch_vel = read_le<float>(bytes, offset);
  p.imu_roll_vel = read_le<float>(bytes, offset);
  p.imu_yaw_acc = read_le<float>(bytes, offset);
  p.imu_pitch_acc = read_le<float>(bytes, offset);
  p.imu_roll_acc = read_le<float>(bytes, offset);
  p.imu_quat_w = read_le<float>(bytes, offset);
  p.imu_quat_x = read_le<float>(bytes, offset);
  p.imu_quat_y = read_le<float>(bytes, offset);
  p.imu_quat_z = read_le<float>(bytes, offset);
  p.order = read_le<std::int16_t>(bytes, offset);
  p.remaining_time_s = read_le<std::int16_t>(bytes, offset);
  p.sentry_behavior_mode = read_le<std::int8_t>(bytes, offset);
  p.colour = read_le<std::uint8_t>(bytes, offset);
  p.system_time_point_us = read_le<std::uint64_t>(bytes, offset);
  return p;
}
}  // namespace mcu_node

