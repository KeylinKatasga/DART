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

TEST(Protocol, EncodesThirtyByteCommandPayload)
{
  mcu_node::SendPayload payload;
  payload.yaw = 1.0F;
  payload.pitch = -2.0F;
  payload.enable_lock = 1;
  payload.enable_shoot = 1;
  payload.autoaim_living_waterfall_flag = 42;
  const auto frame = mcu_node::encode(payload);
  ASSERT_EQ(frame.size(), 34U);
  EXPECT_EQ(frame[0], 0x3e);
  EXPECT_EQ(frame[1], 30);
  EXPECT_TRUE(mcu_node::verify_frame(frame, 30));
  EXPECT_EQ(frame[2], 0x00);
  EXPECT_EQ(frame[3], 0x00);
  EXPECT_EQ(frame[4], 0x80);
  EXPECT_EQ(frame[5], 0x3f);
  // Golden CRC for the exact 30-byte SP payload above.
  EXPECT_EQ(frame[32], 0x21);
  EXPECT_EQ(frame[33], 0xc3);
}

TEST(Protocol, DecodesSixtySixByteReceivePayload)
{
  std::vector<std::uint8_t> frame{0x3e, 66};
  frame.resize(68, 0);
  const float yaw = 12.5F;
  const auto yaw_bytes = std::bit_cast<std::array<std::uint8_t, 4>>(yaw);
  std::copy(yaw_bytes.begin(), yaw_bytes.end(), frame.begin() + 2);
  frame[54] = 0x02;  // order, little endian
  frame[55] = 0x00;
  frame[58] = 0x03;  // sentry behavior
  frame[59] = 0x01;  // colour
  frame[60] = 0x08;  // device time = 8 us
  const auto check = mcu_node::crc16(
    std::span<const std::uint8_t>(frame).subspan(2, 66));
  frame.push_back(check[0]);
  frame.push_back(check[1]);

  const auto decoded = mcu_node::decode_receive(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->imu_yaw, yaw);
  EXPECT_EQ(decoded->order, 2);
  EXPECT_EQ(decoded->sentry_behavior_mode, 3);
  EXPECT_EQ(decoded->colour, 1);
  EXPECT_EQ(decoded->system_time_point_us, 8U);
}

TEST(Protocol, RejectsCorruptCrc)
{
  const auto valid = mcu_node::encode({});
  for (const std::size_t index : {2U, 32U, 33U}) {
    auto corrupt = valid;
    corrupt[index] ^= 1;
    EXPECT_FALSE(mcu_node::verify_frame(corrupt, 30)) << "corrupt byte " << index;
  }

  auto wrong_head = valid;
  wrong_head[0] = 0;
  EXPECT_FALSE(mcu_node::verify_frame(wrong_head, 30));

  auto wrong_length = valid;
  wrong_length[1] = 29;
  EXPECT_FALSE(mcu_node::verify_frame(wrong_length, 30));
}

