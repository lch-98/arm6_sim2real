// ros2_control 하드웨어 인터페이스: Damiao 모터 (MIT 모드)
//
//   명령 인터페이스: position (필수), velocity·effort (URDF에 있으면 속도·토크 피드포워드로 사용)
//   상태 인터페이스: position, velocity, effort
//
//   관절 ↔ 모터 변환:  q_motor = direction·scale·q_joint + offset
//     (scale: 감속 등 단위 변환. 그리퍼는 핑거 이동 [m] → 피니언 각 [rad] = 1 / 피니언 반지름)
//
//   active_joints 에 없는 관절은 mock처럼 명령을 그대로 상태로 돌려준다 → 모터 1대만 연결해도 전체 팔 컨트롤러가 돈다
//   follow 파라미터가 있는 관절(finger_right_joint)은 지정한 관절의 상태를 그대로 따른다 (모터 1개가 핑거 2개 구동)
#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "arm_hardware/can_transport.hpp"
#include "arm_hardware/dm_protocol.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_hardware
{

class DamiaoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DamiaoSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

  struct Joint
  {
    std::string name;
    bool active{false};
    int follow{-1};
    uint16_t esc_id{0}, mst_id{0};
    std::string model;
    dm::Limits lim;
    double kp{0.0}, kd{0.0}, direction{1.0}, scale{1.0}, offset{0.0};
    double pos{0.0}, vel{0.0}, eff{0.0};
    double cmd_pos{NaN}, cmd_vel{NaN}, cmd_eff{NaN};
    bool has_cmd_vel{false}, has_cmd_eff{false};
    int missed{0};
    bool fresh{false};
    dm::Feedback last;
    double k() const {return direction * scale;}
  };

  bool wait_for(const std::function<bool(const CanFrame &)> & pred, double timeout_s, CanFrame & out);
  void handle_frame(const CanFrame & f);
  void disable_all();
  Joint * owner(uint32_t can_id, const uint8_t * data);

  std::vector<Joint> joints_;
  std::unique_ptr<CanTransport> tr_;
  std::string transport_kind_{"socketcan"}, channel_{"can0"};
  int max_missed_{20};
  double max_tracking_error_{0.3};
  bool read_limits_{true};
  bool any_active_{false};
};

}  // namespace arm_hardware
