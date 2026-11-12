#include "arm_hardware/damiao_system.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{
namespace hi = hardware_interface;
using CallbackReturn = hi::CallbackReturn;

namespace
{
rclcpp::Logger LOG() {return rclcpp::get_logger("DamiaoSystem");}

std::string param(const std::unordered_map<std::string, std::string> & p, const std::string & key,
  const std::string & def)
{
  auto it = p.find(key);
  return it == p.end() ? def : it->second;
}

double param_d(const std::unordered_map<std::string, std::string> & p, const std::string & key, double def)
{
  auto it = p.find(key);
  return it == p.end() ? def : std::stod(it->second);
}

int param_i(const std::unordered_map<std::string, std::string> & p, const std::string & key, int def)
{
  auto it = p.find(key);
  return it == p.end() ? def : static_cast<int>(std::stol(it->second, nullptr, 0));
}

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}
}  // namespace

// ---------------------------------------------------------------- 초기화
CallbackReturn DamiaoSystem::on_init(const hi::HardwareInfo & info)
{
  if (hi::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hp = info_.hardware_parameters;
  transport_kind_ = param(hp, "transport", "socketcan");
  channel_ = param(hp, "channel", "can0");
  max_missed_ = param_i(hp, "feedback_timeout_cycles", 20);
  max_tracking_error_ = param_d(hp, "max_tracking_error", 0.3);
  read_limits_ = param(hp, "read_limits", "true") == "true";
  const std::string active_s = param(hp, "active_joints", "all");
  const auto active_list = split(active_s);
  auto is_active = [&](const std::string & n) {
      if (active_s == "all") {return true;}
      if (active_s == "none") {return false;}
      for (const auto & a : active_list) {if (a == n) {return true;}}
      return false;
    };

  joints_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & ji = info_.joints[i];
    auto & j = joints_[i];
    j.name = ji.name;
    const auto & p = ji.parameters;
    j.esc_id = static_cast<uint16_t>(param_i(p, "esc_id", 0));
    j.mst_id = static_cast<uint16_t>(param_i(p, "mst_id", 0));
    j.model = param(p, "model", "DM-J4310-2EC");
    j.lim = dm::default_limits(j.model);
    j.kp = param_d(p, "kp", 0.0);
    j.kd = param_d(p, "kd", 0.0);
    j.direction = param_d(p, "direction", 1.0) < 0 ? -1.0 : 1.0;
    j.scale = param_d(p, "scale", 1.0);
    j.offset = param_d(p, "offset", 0.0);
    j.active = j.esc_id != 0 && is_active(j.name);
    for (const auto & ci : ji.command_interfaces) {
      if (ci.name == hi::HW_IF_VELOCITY) {j.has_cmd_vel = true;}
      if (ci.name == hi::HW_IF_EFFORT) {j.has_cmd_eff = true;}
    }
    if (ji.command_interfaces.empty() || ji.command_interfaces[0].name != hi::HW_IF_POSITION) {
      RCLCPP_ERROR(LOG(), "%s: 첫 명령 인터페이스는 position 이어야 함", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : ji.state_interfaces) {
      if (si.name == hi::HW_IF_POSITION && !si.initial_value.empty()) {
        j.pos = std::stod(si.initial_value);
      }
    }
  }
  // follow (핑거 2개를 모터 1개로)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string f = param(info_.joints[i].parameters, "follow", "");
    if (f.empty()) {continue;}
    for (size_t k = 0; k < joints_.size(); ++k) {
      if (joints_[k].name == f) {joints_[i].follow = static_cast<int>(k);}
    }
    if (joints_[i].follow < 0) {
      RCLCPP_ERROR(LOG(), "%s: follow 대상 %s 없음", joints_[i].name.c_str(), f.c_str());
      return CallbackReturn::ERROR;
    }
    joints_[i].active = false;
  }
  any_active_ = false;
  for (const auto & j : joints_) {
    if (j.active) {
      any_active_ = true;
      RCLCPP_INFO(LOG(), "%s → 모터 ESC 0x%02X / MST 0x%02X (%s), kp %.2f kd %.3f, 방향 %+.0f, 배율 %g, 오프셋 %g",
        j.name.c_str(), j.esc_id, j.mst_id, j.model.c_str(), j.kp, j.kd, j.direction, j.scale, j.offset);
    } else if (j.follow < 0) {
      RCLCPP_INFO(LOG(), "%s → 모터 없음 (명령을 상태로 돌려줌)", j.name.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hi::StateInterface> DamiaoSystem::export_state_interfaces()
{
  std::vector<hi::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hi::HW_IF_POSITION, &j.pos);
    out.emplace_back(j.name, hi::HW_IF_VELOCITY, &j.vel);
    out.emplace_back(j.name, hi::HW_IF_EFFORT, &j.eff);
  }
  return out;
}

std::vector<hi::CommandInterface> DamiaoSystem::export_command_interfaces()
{
  std::vector<hi::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hi::HW_IF_POSITION, &j.cmd_pos);
    if (j.has_cmd_vel) {out.emplace_back(j.name, hi::HW_IF_VELOCITY, &j.cmd_vel);}
    if (j.has_cmd_eff) {out.emplace_back(j.name, hi::HW_IF_EFFORT, &j.cmd_eff);}
  }
  return out;
}

// ---------------------------------------------------------------- 수신
DamiaoSystem::Joint * DamiaoSystem::owner(uint32_t can_id, const uint8_t * data)
{
  for (auto & j : joints_) {
    if (j.active && j.mst_id != 0 && can_id == j.mst_id) {return &j;}
  }
  if (can_id == 0) {
    for (auto & j : joints_) {
      if (j.active && (j.esc_id & 0xF) == (data[0] & 0xF)) {return &j;}
    }
  }
  return nullptr;
}

void DamiaoSystem::handle_frame(const CanFrame & f)
{
  if (f.len != 8) {return;}
  Joint * j = owner(f.id, f.data.data());
  if (!j) {return;}
  if (f.data[2] == dm::REG_READ || f.data[2] == dm::REG_WRITE) {
    if ((f.data[0] | (f.data[1] << 8)) == j->esc_id) {return;}  // 레지스터 응답
  }
  j->last = dm::unpack_feedback(f.data.data(), j->lim);
  j->fresh = true;
}

bool DamiaoSystem::wait_for(const std::function<bool(const CanFrame &)> & pred, double timeout_s, CanFrame & out)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    CanFrame f;
    while (tr_->recv(f)) {
      if (pred(f)) {
        out = f;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return false;
}

// ---------------------------------------------------------------- 수명주기
CallbackReturn DamiaoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!any_active_) {
    RCLCPP_WARN(LOG(), "연결된 모터 없음 (active_joints=none) — 모든 관절을 명령 그대로 돌려줌");
    return CallbackReturn::SUCCESS;
  }
  tr_ = make_transport(transport_kind_);
  std::string err;
  if (!tr_->open(channel_, err)) {
    RCLCPP_ERROR(LOG(), "%s(%s) 열기 실패: %s", transport_kind_.c_str(), channel_.c_str(), err.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOG(), "%s %s 열림", transport_kind_.c_str(), channel_.c_str());
  for (auto & j : joints_) {
    if (!j.active) {continue;}
    tr_->send(j.esc_id, dm::sys_cmd(dm::CMD_DISABLE).data());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!read_limits_) {continue;}
    double v[3];
    bool ok = true;
    const uint8_t rids[3] = {dm::RID_PMAX, dm::RID_VMAX, dm::RID_TMAX};
    for (int k = 0; k < 3 && ok; ++k) {
      CanFrame f;
      tr_->send(dm::REGISTER_ID, dm::reg_read(j.esc_id, rids[k]).data());
      ok = wait_for([&](const CanFrame & c) {return c.len == 8 && dm::is_reg_reply(c.data.data(), j.esc_id, rids[k]);},
          0.1, f);
      if (ok) {v[k] = dm::reg_float(f.data.data());}
    }
    if (ok && v[0] > 0 && v[1] > 0 && v[2] > 0) {
      j.lim = {v[0], v[1], v[2]};
      RCLCPP_INFO(LOG(), "%s: 변환 범위 PMAX %g VMAX %g TMAX %g (모터 레지스터)", j.name.c_str(), v[0], v[1], v[2]);
    } else {
      RCLCPP_ERROR(LOG(), "%s: 모터 0x%02X 레지스터 응답 없음 — 전원·배선·ID·종단저항 확인", j.name.c_str(), j.esc_id);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DamiaoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & j : joints_) {
    j.missed = 0;
    if (!j.active) {continue;}
    CanFrame f;
    tr_->send(j.esc_id, dm::sys_cmd(dm::CMD_ENABLE).data());
    const bool ok = wait_for([&](const CanFrame & c) {
          return c.len == 8 && owner(c.id, c.data.data()) == &j && c.data[2] != dm::REG_READ;
        }, 0.2, f);
    if (!ok) {
      RCLCPP_ERROR(LOG(), "%s: 사용(enable) 응답 없음", j.name.c_str());
      disable_all();
      return CallbackReturn::ERROR;
    }
    const auto fb = dm::unpack_feedback(f.data.data(), j.lim);
    if (fb.status != dm::ENABLED) {
      RCLCPP_ERROR(LOG(), "%s: 사용 실패, 상태 %s → dm_bench clear 후 다시", j.name.c_str(), dm::status_text(fb.status));
      disable_all();
      return CallbackReturn::ERROR;
    }
    j.pos = (fb.q - j.offset) / j.k();
    j.vel = fb.dq / j.k();
    j.eff = fb.tau * j.k();
    RCLCPP_INFO(LOG(), "%s: 사용, 현재 위치 %.4f rad", j.name.c_str(), j.pos);
  }
  for (auto & j : joints_) {
    if (j.follow >= 0) {j.pos = joints_[j.follow].pos;}
    j.cmd_pos = j.pos;   // 활성화 순간 현재 위치를 유지 (튀지 않게)
    j.cmd_vel = NaN;
    j.cmd_eff = NaN;
  }
  return CallbackReturn::SUCCESS;
}

void DamiaoSystem::disable_all()
{
  if (!tr_) {return;}
  for (int rep = 0; rep < 2; ++rep) {
    for (auto & j : joints_) {
      if (j.active) {tr_->send(j.esc_id, dm::sys_cmd(dm::CMD_DISABLE).data());}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

CallbackReturn DamiaoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  disable_all();
  RCLCPP_INFO(LOG(), "모든 모터 사용 안 함");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DamiaoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (tr_) {
    tr_->close();
    tr_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DamiaoSystem::on_error(const rclcpp_lifecycle::State &)
{
  disable_all();
  RCLCPP_ERROR(LOG(), "오류 → 모든 모터 사용 안 함");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 주기 동작
hi::return_type DamiaoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (tr_) {
    CanFrame f;
    while (tr_->recv(f)) {handle_frame(f);}
  }
  for (auto & j : joints_) {
    if (j.follow >= 0) {continue;}
    if (!j.active) {
      if (!std::isnan(j.cmd_pos)) {j.pos = j.cmd_pos;}
      j.vel = 0.0;
      j.eff = 0.0;
      continue;
    }
    if (!j.fresh) {
      if (++j.missed > max_missed_) {
        RCLCPP_ERROR(LOG(), "%s: 응답이 %d 주기 연속 없음", j.name.c_str(), j.missed);
        disable_all();
        return hi::return_type::ERROR;
      }
      continue;
    }
    j.fresh = false;
    j.missed = 0;
    if (j.last.fault() || j.last.status == dm::DISABLED) {
      RCLCPP_ERROR(LOG(), "%s: 모터 상태 %s (드라이버 %u °C / 코일 %u °C)", j.name.c_str(),
        dm::status_text(j.last.status), j.last.t_mos, j.last.t_rotor);
      disable_all();
      return hi::return_type::ERROR;
    }
    j.pos = (j.last.q - j.offset) / j.k();
    j.vel = j.last.dq / j.k();
    j.eff = j.last.tau * j.k();
  }
  for (auto & j : joints_) {
    if (j.follow >= 0) {
      const auto & l = joints_[j.follow];
      j.pos = l.pos;
      j.vel = l.vel;
      j.eff = l.eff;
    }
  }
  return hi::return_type::OK;
}

hi::return_type DamiaoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!tr_) {return hi::return_type::OK;}
  for (auto & j : joints_) {
    if (!j.active) {continue;}
    const double q = std::isnan(j.cmd_pos) ? j.pos : j.cmd_pos;
    if (std::abs(q - j.pos) > max_tracking_error_) {
      RCLCPP_ERROR(LOG(), "%s: 명령 %.3f 와 현재 %.3f 차이가 한계 %.3f rad 초과 → 정지", j.name.c_str(), q, j.pos,
        max_tracking_error_);
      disable_all();
      return hi::return_type::ERROR;
    }
    const double dq = (j.has_cmd_vel && !std::isnan(j.cmd_vel)) ? j.cmd_vel : 0.0;
    const double tau = (j.has_cmd_eff && !std::isnan(j.cmd_eff)) ? j.cmd_eff : 0.0;
    const auto d = dm::pack_mit(j.k() * q + j.offset, j.k() * dq, j.kp, j.kd, tau / j.k(), j.lim);
    tr_->send(j.esc_id, d.data());
  }
  return hi::return_type::OK;
}

}  // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::DamiaoSystem, hardware_interface::SystemInterface)
