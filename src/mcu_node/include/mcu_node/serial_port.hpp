#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace mcu_node
{
class SerialPort
{
public:
  SerialPort(const std::string & device, int baud_rate);
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  std::vector<std::uint8_t> read_frame();
  void write_all(std::span<const std::uint8_t> bytes);
  void cancel();

private:
  int fd_{-1};
  int cancel_pipe_[2]{-1, -1};
  std::uint8_t read_byte();
};
}  // namespace mcu_node

