#include "mcu_node/protocol.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

using mcu_node::ReceivePayload;

TEST(Crc16, MatchesSpVpieCheckVector)
{
  constexpr std::array<std::uint8_t, 9> bytes{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  // Golden bytes produced by the current sp_-vpie Crc16::Encode(), not by this function.
  EXPECT_EQ(mcu_node::crc16(bytes), (std::array<std::uint8_t, 2>{0x37, 0x4b}));
}

TEST(Protocol, EncodesTwentyTwoByteFinCommandPayload)
{
  mcu_node::SendPayload payload;
  payload.angle1 = 1.0F;
  payload.angle2 = -2.0F;
  payload.angle3 = 3.0F;
  payload.angle4 = -4.0F;
  payload.enable_lock = 1;
  payload.enable_shoot = 1;
  payload.autoaim_living_waterfall_flag = 42;
  const auto frame = mcu_node::encode(payload);
  ASSERT_EQ(frame.size(), 26U);
  EXPECT_EQ(frame[0], 0x3e);
  EXPECT_EQ(frame[1], 22);
  EXPECT_TRUE(mcu_node::verify_frame(frame, 22));
  EXPECT_EQ(frame[2], 0x00);
  EXPECT_EQ(frame[3], 0x00);
  EXPECT_EQ(frame[4], 0x80);
  EXPECT_EQ(frame[5], 0x3f);
  // Golden CRC for the exact simplified 22-byte payload above.
  EXPECT_EQ(frame[24], 0x34);
  EXPECT_EQ(frame[25], 0x0f);
}

TEST(Protocol, DecodesThirtyByteReceivePayload)
{
  std::vector<std::uint8_t> frame{0x3e, 30};
  const std::array<float, 4> quaternion{1.0F, 0.1F, 0.2F, 0.3F};
  for (const auto value : quaternion) {
    const auto bytes = std::bit_cast<std::array<std::uint8_t, 4>>(value);
    frame.insert(frame.end(), bytes.begin(), bytes.end());
  }
  frame.push_back(0x02);  // order, little endian
  frame.push_back(0x00);
  frame.push_back(0x07);  // remaining time, little endian
  frame.push_back(0x00);
  frame.push_back(0x03);  // sentry behavior
  frame.push_back(0x01);  // colour
  frame.push_back(0x08);  // device time = 8 us, little endian
  frame.insert(frame.end(), 7, 0x00);
  const auto check = mcu_node::crc16(
    std::span<const std::uint8_t>(frame).subspan(2, 30));
  frame.push_back(check[0]);
  frame.push_back(check[1]);

  const auto decoded = mcu_node::decode_receive(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->imu_quat_w, 1.0F);
  EXPECT_FLOAT_EQ(decoded->imu_quat_x, 0.1F);
  EXPECT_FLOAT_EQ(decoded->imu_quat_y, 0.2F);
  EXPECT_FLOAT_EQ(decoded->imu_quat_z, 0.3F);
  EXPECT_EQ(decoded->order, 2);
  EXPECT_EQ(decoded->remaining_time_s, 7);
  EXPECT_EQ(decoded->sentry_behavior_mode, 3);
  EXPECT_EQ(decoded->colour, 1);
  EXPECT_EQ(decoded->system_time_point_us, 8U);
}

TEST(Protocol, RejectsCorruptCrc)
{
  const auto valid = mcu_node::encode({});
  for (const std::size_t index : {2U, 24U, 25U}) {
    auto corrupt = valid;
    corrupt[index] ^= 1;
    EXPECT_FALSE(mcu_node::verify_frame(corrupt, 22)) << "corrupt byte " << index;
  }

  auto wrong_head = valid;
  wrong_head[0] = 0;
  EXPECT_FALSE(mcu_node::verify_frame(wrong_head, 22));

  auto wrong_length = valid;
  wrong_length[1] = 21;
  EXPECT_FALSE(mcu_node::verify_frame(wrong_length, 22));
}
