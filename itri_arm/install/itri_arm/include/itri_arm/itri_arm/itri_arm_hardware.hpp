#ifndef ITRI_ARM_HARDWARE_HPP_
#define ITRI_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <regex>
#include <chrono>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace itri_arm_hardware
{
class ItriArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ItriArmHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 網路連接方法
  bool connect_to_robot();
  void disconnect_from_robot();
  
  // 資料讀取執行緒
  void read_thread_function();
  bool request_joint_angles();
  bool parse_joint_angles(const std::string& response);
  
  // 指令發送
  bool send_joint_command();

  // 關節參數
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  
  // 狀態變數
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // 指令變數
  std::vector<double> hw_commands_;
  
  // 網路參數
  std::string robot_ip_;
  int robot_port_;
  std::string target_ip_;
  int socket_fd_;
  
  // 執行緒控制
  std::thread read_thread_;
  std::mutex data_mutex_;
  std::atomic<bool> thread_running_;
  
  // 控制參數
  double read_frequency_;
  
  // 日誌
  rclcpp::Logger logger_ = rclcpp::get_logger("ItriArmHardware");
};

}  // namespace itri_arm_hardware

#endif  // ITRI_ARM_HARDWARE_HPP_