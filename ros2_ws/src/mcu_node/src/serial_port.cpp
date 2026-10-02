#include "mcu_node/serial_port.hpp"

#include "mcu_node/protocol.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace mcu_node
{
namespace
{
speed_t baud_constant(const int baud_rate)
{
  if (baud_rate == 115200) {return B115200;}
  throw std::invalid_argument("only the protocol baud rate 115200 is supported");
}

std::runtime_error io_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}
}  // namespace

SerialPort::SerialPort(const std::string & device, const int baud_rate)
{
  if (::pipe2(cancel_pipe_, O_CLOEXEC | O_NONBLOCK) != 0) {throw io_error("pipe2");}
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {throw io_error("open " + device);}

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {throw io_error("tcgetattr");}
  cfmakeraw(&tty);
  const auto speed = baud_constant(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {throw io_error("tcsetattr");}
}

SerialPort::~SerialPort()
{
  if (fd_ >= 0) {::close(fd_);}
  if (cancel_pipe_[0] >= 0) {::close(cancel_pipe_[0]);}
  if (cancel_pipe_[1] >= 0) {::close(cancel_pipe_[1]);}
}

std::uint8_t SerialPort::read_byte()
{
  pollfd descriptors[2]{{fd_, POLLIN, 0}, {cancel_pipe_[0], POLLIN, 0}};
  std::uint8_t byte{};
  while (true) {
    const auto ready = ::poll(descriptors, 2, -1);
    if (ready < 0 && errno == EINTR) {continue;}
    if (ready < 0) {throw io_error("serial poll");}
    if (descriptors[1].revents & POLLIN) {throw std::runtime_error("serial read cancelled");}
    if (!(descriptors[0].revents & POLLIN)) {
      if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("serial device disconnected");
      }
      continue;
    }
    const auto count = ::read(fd_, &byte, 1);
    if (count == 1) {return byte;}
    if (count < 0 && errno == EINTR) {continue;}
    throw io_error("serial read");
  }
}

void SerialPort::cancel()
{
  const std::uint8_t signal = 1;
  if (cancel_pipe_[1] >= 0) {static_cast<void>(::write(cancel_pipe_[1], &signal, 1));}
}

std::vector<std::uint8_t> SerialPort::read_frame()
{
  std::uint8_t byte{};
  do{
    byte = read_byte();
  } while (byte != kFrameHead);
  const auto length = read_byte();
  if (length > 124) {throw std::runtime_error("invalid gimbal frame length");}
  std::vector<std::uint8_t> frame{kFrameHead, length};
  frame.reserve(static_cast<std::size_t>(length) + 4);
  for (std::size_t i = 0; i < static_cast<std::size_t>(length) + 2; ++i) {
    frame.push_back(read_byte());
  }
  return frame;
}

void SerialPort::write_all(std::span<const std::uint8_t> bytes)
{
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
    } else if (count < 0 && errno != EINTR) {throw io_error("serial write");}
  }
  // Match sp_-vpie/io/gimbal/protocol/serial.cpp: return after the complete
  // frame has been accepted by the kernel. tcdrain() can block indefinitely on
  // some USB CDC ACM implementations and would prevent tx_raw publication.
}
}  // namespace mcu_node

