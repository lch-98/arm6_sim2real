// Damiao 액추에이터 CAN 프로토콜 (헤더 전용). arm_hardware/dm_protocol.py 와 같은 정의.
//   MIT 명령: ESC_ID, 시스템 명령: ESC_ID + FF×7 + (FC 사용 / FD 해제 / FE 영점 / FB 오류 해제)
//   레지스터: 0x7FF [id_l id_h 0x33(읽기)|0x55(쓰기) RID 값4]
//   응답: MST_ID [상태<<4|id][p16][v12|t12][T_mos][T_rotor]
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace arm_hardware::dm
{

constexpr uint32_t REGISTER_ID = 0x7FF;
constexpr uint8_t CMD_ENABLE = 0xFC, CMD_DISABLE = 0xFD, CMD_SAVE_ZERO = 0xFE, CMD_CLEAR_ERROR = 0xFB;
constexpr uint8_t REG_READ = 0x33, REG_WRITE = 0x55;
constexpr uint8_t RID_PMAX = 21, RID_VMAX = 22, RID_TMAX = 23;
constexpr double KP_MAX = 500.0, KD_MAX = 5.0;

enum Status : uint8_t
{
  DISABLED = 0x0, ENABLED = 0x1, OVER_VOLTAGE = 0x8, UNDER_VOLTAGE = 0x9, OVER_CURRENT = 0xA,
  MOS_OVER_TEMP = 0xB, ROTOR_OVER_TEMP = 0xC, LOST_COMM = 0xD, OVERLOAD = 0xE
};

inline const char * status_text(uint8_t s)
{
  switch (s) {
    case DISABLED: return "disabled";
    case ENABLED: return "enabled";
    case OVER_VOLTAGE: return "over-voltage";
    case UNDER_VOLTAGE: return "under-voltage";
    case OVER_CURRENT: return "over-current";
    case MOS_OVER_TEMP: return "driver(MOS) over-temperature";
    case ROTOR_OVER_TEMP: return "coil over-temperature";
    case LOST_COMM: return "communication lost";
    case OVERLOAD: return "overload";
    default: return "unknown";
  }
}

struct Limits
{
  double pmax{12.5}, vmax{30.0}, tmax{10.0};
};

// 공장 기본값 (Seeed DAMIAO 43 시리즈 자료). 실제 값은 활성화 전에 레지스터에서 읽는다.
inline Limits default_limits(const std::string & model)
{
  if (model == "DM-J4340P-2EC") {return {12.5, 8.0, 28.0};}
  if (model == "DM-H3510") {return {12.5, 280.0, 1.0};}
  return {12.5, 30.0, 10.0};  // DM-J4310-2EC
}

inline uint32_t float_to_uint(double x, double lo, double hi, int bits)
{
  x = std::clamp(x, lo, hi);
  return static_cast<uint32_t>((x - lo) / (hi - lo) * static_cast<double>((1u << bits) - 1));
}

inline double uint_to_float(uint32_t u, double lo, double hi, int bits)
{
  return static_cast<double>(u) / static_cast<double>((1u << bits) - 1) * (hi - lo) + lo;
}

using Data = std::array<uint8_t, 8>;

inline Data pack_mit(double q, double dq, double kp, double kd, double tau, const Limits & l)
{
  const uint32_t p = float_to_uint(q, -l.pmax, l.pmax, 16);
  const uint32_t v = float_to_uint(dq, -l.vmax, l.vmax, 12);
  const uint32_t kpu = float_to_uint(kp, 0.0, KP_MAX, 12);
  const uint32_t kdu = float_to_uint(kd, 0.0, KD_MAX, 12);
  const uint32_t t = float_to_uint(tau, -l.tmax, l.tmax, 12);
  return {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p & 0xFF),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>(((v & 0xF) << 4) | ((kpu >> 8) & 0xF)),
    static_cast<uint8_t>(kpu & 0xFF),
    static_cast<uint8_t>(kdu >> 4), static_cast<uint8_t>(((kdu & 0xF) << 4) | ((t >> 8) & 0xF)),
    static_cast<uint8_t>(t & 0xFF)};
}

inline Data sys_cmd(uint8_t cmd)
{
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
}

inline Data reg_read(uint16_t esc_id, uint8_t rid)
{
  return {static_cast<uint8_t>(esc_id & 0xFF), static_cast<uint8_t>(esc_id >> 8), REG_READ, rid, 0, 0, 0, 0};
}

inline bool is_reg_reply(const uint8_t * d, uint16_t esc_id, uint8_t rid)
{
  return (d[0] | (d[1] << 8)) == esc_id && d[2] == REG_READ && d[3] == rid;
}

inline float reg_float(const uint8_t * d)
{
  float f;
  std::memcpy(&f, d + 4, 4);  // 리틀엔디언 (x86, ARM 공통)
  return f;
}

struct Feedback
{
  uint8_t id4{0}, status{0};
  double q{0}, dq{0}, tau{0};
  uint8_t t_mos{0}, t_rotor{0};
  bool fault() const {return status != DISABLED && status != ENABLED;}
};

inline Feedback unpack_feedback(const uint8_t * d, const Limits & l)
{
  Feedback f;
  f.id4 = d[0] & 0x0F;
  f.status = (d[0] >> 4) & 0x0F;
  const uint32_t p = (static_cast<uint32_t>(d[1]) << 8) | d[2];
  const uint32_t v = (static_cast<uint32_t>(d[3]) << 4) | (d[4] >> 4);
  const uint32_t t = ((static_cast<uint32_t>(d[4]) & 0xF) << 8) | d[5];
  f.q = uint_to_float(p, -l.pmax, l.pmax, 16);
  f.dq = uint_to_float(v, -l.vmax, l.vmax, 12);
  f.tau = uint_to_float(t, -l.tmax, l.tmax, 12);
  f.t_mos = d[6];
  f.t_rotor = d[7];
  return f;
}

}  // namespace arm_hardware::dm
