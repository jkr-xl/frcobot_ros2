#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  /*
  伺服模式状态。ServoJ 必须在 ServoMoveStart() 之后才被接受;
  若机器人处于手动模式或有报警, ServoMoveStart 会返回错误码 14。
  此时不能让插件激活失败 —— Jazzy 的 resource_manager 会因此抛异常并
  终止整个 controller_manager 进程。改为降级运行: 只读关节状态, 不下发
  ServoJ, 并周期性重试, 使用户在示教器上切到自动模式后可自动恢复。
  Servo mode state. ServoJ is only accepted after ServoMoveStart(); that call
  returns 14 while the robot is in manual mode or faulted. Activation must NOT
  fail in that case -- Jazzy's resource_manager throws and kills the whole
  controller_manager process. Degrade instead: keep reading joint states, skip
  ServoJ, and retry periodically so flipping the pendant to AUTOMATIC recovers
  without relaunching.
  */
  bool _servo_mode_ok = false;
  /*
  ServoJ/ServoMoveStart 的下发通道: 0=xmlrpc, 1=UDP(机器人 20007 端口)。
  xmlrpc 是同步 TCP 往返, 每个周期每条手臂一次 —— 实测两条手臂使 write()
  耗时 67~77 ms, 而 100 Hz 的预算只有 10 ms, controller_manager 每次丢 7~8 个
  周期。UDP 透传是为这种流式下发设计的, 不阻塞。可用硬件参数
  <param name="servo_comm_type"> 覆盖。
  DEFAULT IS 1 (UDP). Both transports are verified to move the arm on hardware
  2026-09-08. xmlrpc works but blocks ~35 ms per arm per cycle, so two arms
  overrun a 10 ms budget badly (measured 67-77 ms of write()); UDP writes cost
  microseconds. One caveat: UDP is fire-and-forget, so ServoJ always returns 0
  and a rejected command is invisible at this layer -- rely on the trajectory
  controller's goal tolerances to catch a non-moving arm, and set
  servo_comm_type:=0 to get xmlrpc's per-command error codes back when
  debugging.
  Transport for ServoJ/ServoMoveStart: 0=xmlrpc, 1=UDP (robot port 20007).
  xmlrpc is a synchronous TCP round trip per arm per cycle -- measured at
  67-77 ms of write() for two arms against a 10 ms budget at 100 Hz, so the
  controller manager dropped 7-8 cycles every time. UDP passthrough is the
  streaming path and does not block. Override with the hardware parameter
  <param name="servo_comm_type">.
  */
  int _servo_comm_type = 1;
  rclcpp::Time _last_servo_retry{0, 0, RCL_STEADY_TIME};
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::unique_ptr<FRRobot> _ptr_robot;
};

} //end namespace


#endif