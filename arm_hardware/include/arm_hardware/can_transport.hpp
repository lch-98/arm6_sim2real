// CAN 송수신 (비차단). SocketCAN(can0, vcan0) 과 Damiao USB-CAN 시리얼(/dev/ttyACM*) 두 가지.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace arm_hardware
{

struct CanFrame
{
  uint32_t id{0};
  uint8_t len{8};
  std::array<uint8_t, 8> data{};
};

class CanTransport
{
public:
  virtual ~CanTransport() = default;
  virtual bool open(const std::string & channel, std::string & error) = 0;
  virtual bool send(uint32_t id, const uint8_t * data, uint8_t len = 8) = 0;
  /// 받은 프레임이 있으면 true (기다리지 않음)
  virtual bool recv(CanFrame & frame) = 0;
  virtual void close() = 0;
};

class SocketCanTransport : public CanTransport
{
public:
  ~SocketCanTransport() override {close();}
  bool open(const std::string & channel, std::string & error) override;
  bool send(uint32_t id, const uint8_t * data, uint8_t len) override;
  bool recv(CanFrame & frame) override;
  void close() override;

private:
  int fd_{-1};
};

/// Damiao USB-CAN 보드: 송신 30바이트 / 수신 16바이트 전용 프레임, 921600 bps
class DamiaoSerialTransport : public CanTransport
{
public:
  ~DamiaoSerialTransport() override {close();}
  bool open(const std::string & channel, std::string & error) override;
  bool send(uint32_t id, const uint8_t * data, uint8_t len) override;
  bool recv(CanFrame & frame) override;
  void close() override;

private:
  int fd_{-1};
  std::vector<uint8_t> buf_;
};

/// kind: "socketcan" | "damiao-serial"
std::unique_ptr<CanTransport> make_transport(const std::string & kind);

}  // namespace arm_hardware
