#include "itri_arm/itri_arm_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace itri_arm_hardware
{
hardware_interface::CallbackReturn ItriArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化參數
  robot_ip_ = info_.hardware_parameters["robot_ip"];
  robot_port_ = std::stoi(info_.hardware_parameters["robot_port"]);
  target_ip_ = info_.hardware_parameters["target_ip"];
  read_frequency_ = std::stod(info_.hardware_parameters.count("read_frequency") ? 
                              info_.hardware_parameters["read_frequency"] : "2.0");

  // 初始化關節
  joint_names_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  num_joints_ = joint_names_.size();

  // 調整向量大小
  hw_positions_.resize(num_joints_, 0.0);
  hw_velocities_.resize(num_joints_, 0.0);
  hw_efforts_.resize(num_joints_, 0.0);
  hw_commands_.resize(num_joints_, 0.0);

  socket_fd_ = -1;
  thread_running_ = false;

  RCLCPP_INFO(logger_, "ITRI Arm Hardware initialized with %zu joints", num_joints_);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ItriArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring ITRI Arm Hardware...");
  
  // 連接到機械手臂
  if (!connect_to_robot())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to robot");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "ITRI Arm Hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ItriArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < num_joints_; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ItriArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < num_joints_; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ItriArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating ITRI Arm Hardware...");

  // 啟動讀取執行緒
  thread_running_ = true;
  read_thread_ = std::thread(&ItriArmHardware::read_thread_function, this);

  // 初始化指令為當前位置
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    hw_commands_ = hw_positions_;
  }

  RCLCPP_INFO(logger_, "ITRI Arm Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ItriArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating ITRI Arm Hardware...");

  // 停止執行緒
  thread_running_ = false;
  if (read_thread_.joinable())
  {
    read_thread_.join();
  }

  disconnect_from_robot();

  RCLCPP_INFO(logger_, "ITRI Arm Hardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ItriArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 計算速度 (簡單數值微分)
  static std::vector<double> last_positions(num_joints_, 0.0);
  
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (size_t i = 0; i < num_joints_; i++)
    {
      hw_velocities_[i] = (hw_positions_[i] - last_positions[i]) / period.seconds();
      last_positions[i] = hw_positions_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ItriArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 暫時停用自動控制指令，專注於讀取角度
  // 如果讀取到實際角度後再啟用控制
  
  static bool first_valid_read = false;
  
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 檢查是否有任何非零的關節角度
    for (size_t i = 0; i < num_joints_; i++)
    {
      if (std::abs(hw_positions_[i]) > 0.01) // 如果有角度大於 0.01 弧度
      {
        first_valid_read = true;
        break;
      }
    }
  }
  
  if (first_valid_read)
  {
    // 只有在成功讀取到實際角度後才開始發送控制指令
    if (!send_joint_command())
    {
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000)
      {
        RCLCPP_WARN(logger_, "Failed to send joint command");
        last_warn_time = now;
      }
    }
  }
  else
  {
    static auto last_info_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_info_time).count() > 5000)
    {
      RCLCPP_INFO(logger_, "Waiting for valid joint angle readings before enabling control...");
      last_info_time = now;
    }
  }

  return hardware_interface::return_type::OK;
}

bool ItriArmHardware::connect_to_robot()
{
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to create socket");
    return false;
  }

  // 設定 socket 選項
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(robot_port_);
  inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr);

  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to connect to robot controller at %s:%d", 
                 robot_ip_.c_str(), robot_port_);
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(logger_, "Connected to robot controller at %s:%d", 
              robot_ip_.c_str(), robot_port_);

  // 發送初始化指令
  std::string init_cmd = "MOVJ 0 0 0 0 0 0\0";
  send(socket_fd_, init_cmd.c_str(), init_cmd.length(), 0);

  return true;
}

void ItriArmHardware::disconnect_from_robot()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_INFO(logger_, "Disconnected from robot controller");
  }
}

void ItriArmHardware::read_thread_function()
{
  // 等待系統穩定
  RCLCPP_INFO(logger_, "Waiting for robot system to stabilize...");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  rclcpp::Rate rate(read_frequency_);
  bool request_toggle = true;  // True=請求角度, False=請求位置
  
  while (thread_running_ && rclcpp::ok())
  {
    if (socket_fd_ >= 0)
    {
      if (request_toggle)
      {
        // 請求關節角度
        request_joint_angles();
        request_toggle = false;  // 下次請求位置
      }
      else
      {
        // 請求位置資料
        request_position_data();
        request_toggle = true;   // 下次請求角度
      }
    }
    
    rate.sleep();
  }
}

bool ItriArmHardware::request_joint_angles()
{
  std::string cmd = "NETS_GETPOS " + target_ip_ + "\0";
  
  if (send(socket_fd_, cmd.c_str(), cmd.length(), 0) < 0)
  {
    return false;
  }

  char buffer[1024];
  int bytes_received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  
  if (bytes_received <= 0)
  {
    return false;
  }

  buffer[bytes_received] = '\0';
  std::string response(buffer);
  
  // 記錄收到的原始回應
  RCLCPP_INFO(logger_, "Joint angle response: '%s'", response.c_str());
  
  return parse_joint_angles(response);
}

bool ItriArmHardware::request_position_data()
{
  std::string cmd = "NETS_GETDEG " + target_ip_ + "\0";
  
  if (send(socket_fd_, cmd.c_str(), cmd.length(), 0) < 0)
  {
    return false;
  }

  char buffer[1024];
  int bytes_received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  
  if (bytes_received <= 0)
  {
    return false;
  }

  buffer[bytes_received] = '\0';
  std::string response(buffer);
  
  RCLCPP_INFO(logger_, "Position response: '%s'", response.c_str());
  
  // 解析位置資料（我們主要關心關節角度，但這能幫助通訊協議）
  std::regex float_regex(R"([-+]?\d*\.\d+(?:[eE][-+]?\d+)?)");
  std::sregex_iterator iter(response.begin(), response.end(), float_regex);
  std::sregex_iterator end;

  std::vector<double> position_data;
  for (; iter != end; ++iter)
  {
    position_data.push_back(std::stod(iter->str()));
  }

  if (position_data.size() == 6)
  {
    RCLCPP_DEBUG(logger_, "Position data: X=%.2f, Y=%.2f, Z=%.2f, RX=%.2f, RY=%.2f, RZ=%.2f",
                 position_data[0], position_data[1], position_data[2],
                 position_data[3], position_data[4], position_data[5]);
    return true;
  }
  
  return false;
}

bool ItriArmHardware::parse_joint_angles(const std::string& response)
{
  RCLCPP_INFO(logger_, "Parsing response: '%s'", response.c_str());

  std::regex float_regex(R"([-+]?\d*\.\d+(?:[eE][-+]?\d+)?)");
  std::sregex_iterator iter(response.begin(), response.end(), float_regex);
  std::sregex_iterator end;

  std::vector<double> angles;
  for (; iter != end; ++iter)
  {
    angles.push_back(std::stod(iter->str()));
  }

  RCLCPP_INFO(logger_, "Found %zu numbers in response", angles.size());

  if (angles.size() == num_joints_)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < num_joints_; i++)
    {
      hw_positions_[i] = angles[i] * M_PI / 180.0;  // 轉換為弧度
    }
    RCLCPP_INFO(logger_, "Successfully updated joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad", 
                hw_positions_[0], hw_positions_[1], hw_positions_[2], 
                hw_positions_[3], hw_positions_[4], hw_positions_[5]);
    return true;
  }
  else if (angles.size() == 0 && response.find("IRA") != std::string::npos)
  {
    // 收到 IRA 回應，這可能表示指令已接收但還沒有回傳資料
    return false;
  }
  else
  {
    RCLCPP_WARN(logger_, "Unexpected response format: expected %zu angles, got %zu (response: '%s')", 
                num_joints_, angles.size(), response.c_str());
    return false;
  }
}

bool ItriArmHardware::send_joint_command()
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  std::vector<double> cmd_degrees(num_joints_);
  
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < num_joints_; i++)
    {
      cmd_degrees[i] = hw_commands_[i] * 180.0 / M_PI;  // 轉換為度
    }
  }

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "MOVJ %.3f %.3f %.3f %.3f %.3f %.3f",
           cmd_degrees[0], cmd_degrees[1], cmd_degrees[2],
           cmd_degrees[3], cmd_degrees[4], cmd_degrees[5]);

  // 手動添加 null terminator
  size_t len = strlen(cmd);
  cmd[len] = '\0';
  cmd[len + 1] = '\0';  // 添加結束符

  RCLCPP_INFO(logger_, "Sending command: %s", cmd);
  
  bool success = send(socket_fd_, cmd, len + 1, 0) > 0;
  if (success)
  {
    RCLCPP_INFO(logger_, "Command sent successfully");
  }
  else
  {
    RCLCPP_WARN(logger_, "Failed to send command");
  }
  
  return success;
}

}  // namespace itri_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  itri_arm_hardware::ItriArmHardware, hardware_interface::SystemInterface)
