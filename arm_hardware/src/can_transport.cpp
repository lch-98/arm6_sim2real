#include "arm_hardware/can_transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace arm_hardware
{

// ---------------------------------------------------------------- SocketCAN
bool SocketCanTransport::open(const std::string & channel, std::string & error)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    error = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    error = "인터페이스 " + channel + " 없음 (ip link 로 올렸는지 확인): " + std::strerror(errno);
    close();
    return false;
  }
  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    error = std::string("bind: ") + std::strerror(errno);
    close();
    return false;
  }
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

bool SocketCanTransport::send(uint32_t id, const uint8_t * data, uint8_t len)
{
  struct can_frame f {};
  f.can_id = id & CAN_SFF_MASK;
  f.can_dlc = len;
  std::memcpy(f.data, data, len);
  return ::write(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f));
}

bool SocketCanTransport::recv(CanFrame & frame)
{
  struct can_frame f {};
  while (true) {
    const ssize_t n = ::read(fd_, &f, sizeof(f));
    if (n != static_cast<ssize_t>(sizeof(f))) {
      return false;
    }
    if (f.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) {
      continue;
    }
    frame.id = f.can_id & CAN_SFF_MASK;
    frame.len = f.can_dlc;
    std::memcpy(frame.data.data(), f.data, 8);
    return true;
  }
}

void SocketCanTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ---------------------------------------------------------------- Damiao 시리얼
namespace
{
constexpr uint8_t TX_TEMPLATE[30] = {0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
  0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00};
constexpr size_t RX_LEN = 16;
}  // namespace

bool DamiaoSerialTransport::open(const std::string & channel, std::string & error)
{
  fd_ = ::open(channel.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = channel + " 열기 실패: " + std::strerror(errno) + " (dialout 그룹 권한 확인)";
    return false;
  }
  struct termios tio {};
  if (::tcgetattr(fd_, &tio) == 0) {
    ::cfmakeraw(&tio);
    ::cfsetispeed(&tio, B921600);
    ::cfsetospeed(&tio, B921600);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::tcsetattr(fd_, TCSANOW, &tio);
  }
  ::tcflush(fd_, TCIOFLUSH);
  buf_.clear();
  return true;
}

bool DamiaoSerialTransport::send(uint32_t id, const uint8_t * data, uint8_t len)
{
  uint8_t f[30];
  std::memcpy(f, TX_TEMPLATE, 30);
  f[13] = id & 0xFF;
  f[14] = (id >> 8) & 0xFF;
  std::memcpy(f + 21, data, len < 8 ? len : 8);
  return ::write(fd_, f, 30) == 30;
}

bool DamiaoSerialTransport::recv(CanFrame & frame)
{
  uint8_t tmp[512];
  ssize_t n;
  while ((n = ::read(fd_, tmp, sizeof(tmp))) > 0) {
    buf_.insert(buf_.end(), tmp, tmp + n);
  }
  size_t i = 0;
  bool got = false;
  while (buf_.size() - i >= RX_LEN) {
    if (buf_[i] == 0xAA && buf_[i + RX_LEN - 1] == 0x55) {
      const uint8_t cmd = buf_[i + 1];
      if (cmd == 0x11) {
        frame.id = buf_[i + 3] | (buf_[i + 4] << 8) | (buf_[i + 5] << 16) | (static_cast<uint32_t>(buf_[i + 6]) << 24);
        frame.len = 8;
        std::memcpy(frame.data.data(), &buf_[i + 7], 8);
        got = true;
      }
      i += RX_LEN;
      if (got) {
        break;
      }
    } else {
      ++i;
    }
  }
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i));
  return got;
}

void DamiaoSerialTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::unique_ptr<CanTransport> make_transport(const std::string & kind)
{
  if (kind == "damiao-serial") {
    return std::make_unique<DamiaoSerialTransport>();
  }
  return std::make_unique<SocketCanTransport>();
}

}  // namespace arm_hardware
