#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
<<<<<<< HEAD
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class RobotController : public rclcpp::Node
{
public:
  RobotController() : Node("robot_controller"), state_(State::INIT), orientation_type_(-1), gripper_state_("FALSE")
  {
    coordinates_subscription = this->create_subscription<geometry_msgs::msg::Point>(
      "/lego_coordinates", 10, 
      std::bind(&RobotController::coordinatesCallback, this, std::placeholders::_1));

    orientation_subscription = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/lego_orientation", 10, 
      std::bind(&RobotController::orientationCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    force_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/sensor_data", 10, 
        std::bind(&RobotController::forceCallback, this, std::placeholders::_1));

    orientation_type_subscription = this->create_subscription<std_msgs::msg::Int8>(
      "/lego_orientation_type", 10,
      std::bind(&RobotController::orientationTypeCallback, this, std::placeholders::_1));
    
    gripper_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/gripper_state", 10, 
      std::bind(&RobotController::gripperStateCallback, this, std::placeholders::_1));   

    force_result_publisher_ = this->create_publisher<std_msgs::msg::Bool>("force_change_result", 10);

    gripper_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("custom_gripper", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    // 可以在這裡設置一些初始值
    orientation_type_ = 0;  // 設置預設方向類型，這樣就不會卡在等待方向

    RCLCPP_INFO(this->get_logger(), "Robot controller initialized.");
  }

private:
  enum class State {
    INIT,
    MOVE_TO_HOME,
    MOVE_TO_INTERMEDIATE,
    WAIT_AFTER_INTERMEDIATE,
    WAIT_FOR_COORDINATES,
    MOVE_TO_OBJECT,
    GRASP_OBJECT,
    CHECK_GRASP,
    MOVE_TO_OBSERVATION,
    WAIT_FOR_ORIENTATION,
    MOVE_TO_FINAL_POSITION,
    MOVE_TO_ASSEMBLE,
    RELEASE_OBJECT,
    FINISHED
  };

  State state_;
  int orientation_type_;
  int cycle_count_ = 0;
  rclcpp::Time state_start_time_;
  geometry_msgs::msg::Pose target_pose_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coordinates_subscription;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_subscription;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr orientation_type_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr gripper_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string gripper_state_;
  rclcpp::Time gripper_check_start_time_;
  bool coordinates_received_ = false;
  bool orientation_received_ = false;
  rclcpp::Time intermediate_move_time_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_result_publisher_;
  std::vector<std::vector<double>> force_values_;  // 改為儲存三軸力值
  std::vector<double> current_forces_;             // 當前的三軸力值
  double current_fx_;
  bool is_recording_;


  void timerCallback()
  {
    switch (state_) {
      case State::INIT:
        initializeMoveGroup();
        break;
      case State::MOVE_TO_HOME:
        if (goHome()) {
//            setState(State::MOVE_TO_OBSERVATION);
            setState(State::MOVE_TO_INTERMEDIATE);
        }
        break;


      case State::MOVE_TO_INTERMEDIATE:
        if (moveToIntermediate()) {
          intermediate_move_time_ = this->now();
          setState(State::WAIT_AFTER_INTERMEDIATE);
        }
        break;
      case State::WAIT_AFTER_INTERMEDIATE:
        if ((this->now() - intermediate_move_time_).seconds() >= 3.0) {
          setState(State::WAIT_FOR_COORDINATES);
        }
        break;
      case State::WAIT_FOR_COORDINATES:
        // 在這個狀態下，我們等待坐標和方向信息
        // 實際的處理在coordinatesCallback和orientationCallback中進行
        break;
      case State::MOVE_TO_OBJECT:
        if (moveToObject()) {
          setState(State::GRASP_OBJECT);
        }
        break;
      case State::GRASP_OBJECT:
        if (graspObject()) {
          gripper_check_start_time_ = this->now();
          setState(State::CHECK_GRASP);
        }
        break;
      case State::CHECK_GRASP:
        if ((this->now() - gripper_check_start_time_).seconds() >= 3.0) {
          if (gripper_state_ == "TRUE") {
            RCLCPP_INFO(this->get_logger(), "Grasp successful");
            setState(State::MOVE_TO_OBSERVATION);
          } else {
            RCLCPP_INFO(this->get_logger(), "Grasp failed, resetting to wait for coordinates");
            setState(State::WAIT_FOR_COORDINATES);
          }
        }
        break;


      case State::MOVE_TO_OBSERVATION:
        if (moveToObservation()) {
          setState(State::WAIT_FOR_ORIENTATION);
          state_start_time_ = this->now();
        }
        break;

      case State::WAIT_FOR_ORIENTATION:
        // 確保至少等待2秒進行辨識
        if ((this->now() - state_start_time_).seconds() >= 2.0) {
            // 只有在收到有效的orientation_type或超時(5秒)後才繼續
            if (orientation_type_ != -1 || 
                    (this->now() - state_start_time_).seconds() > 5.0) {
                if (orientation_type_ == 0) {
                    setState(State::MOVE_TO_ASSEMBLE);
                } else {
                    setState(State::MOVE_TO_FINAL_POSITION);
                }
            }
        }
        break;

      case State::MOVE_TO_FINAL_POSITION:
        if (moveToFinalPosition()) {
                setState(State::WAIT_FOR_ORIENTATION);  // 完成後回到等待方向狀態
            }
        break;

      case State::MOVE_TO_ASSEMBLE:
        if (moveToAssemble()) {
                setState(State::RELEASE_OBJECT);
            }
        break;

      case State::RELEASE_OBJECT:
        if (releaseObject()) {
          setState(State::FINISHED);
        }
        break;

      case State::FINISHED:
          RCLCPP_INFO(this->get_logger(), "Task completed. Restarting...");
          // 增加循環計數
          cycle_count_++;
          RCLCPP_INFO(this->get_logger(), "Starting cycle %d", cycle_count_);
          // 重置必要的變數
          orientation_type_ = 0;
          coordinates_received_ = false;
          orientation_received_ = false;
          // 回到初始狀態
          setState(State::INIT);
          break;
    }
  }

  void setState(State new_state)
  {
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "Entering state: %s", stateToString(state_).c_str());
  }

  void initializeMoveGroup()
  {
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_arm_->setMaxVelocityScalingFactor(0.5);
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    setState(State::MOVE_TO_HOME);
  }

  bool goHome()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    move_group_arm_->setStartStateToCurrentState();
    RCLCPP_INFO(this->get_logger(), "Going Home");
//    std::vector<double> joint_group_positions = {-1.69, -1.88, 2.2, -1.9, -1.57, 3.02};

    std::vector<double> joint_group_positions = {-1.64, -2.27, 1.92, -1.22, -1.57, 1.57};   //////////////////////////////////////////////
    move_group_arm_->setPlanningTime(20.0);
    move_group_arm_->setJointValueTarget(joint_group_positions);

    return executePlan();
  }

  bool moveToIntermediate()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to intermediate position");
    std::vector<double> intermediate_position = {-2.17, -1.24, 0.68, -1.01, -1.57, 2.52};
    move_group_arm_->setJointValueTarget(intermediate_position);
    move_group_arm_->setPlanningTime(10.0);

    return executePlan();
  }

  bool moveToObject()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to object");
    move_group_arm_->setPoseTarget(target_pose_);

    return executePlan();
  }

  bool graspObject()
  {
    RCLCPP_INFO(LOGGER, "Grasping object");
    publishGripperCommand(40, 10);  // Open gripper
    rclcpp::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose approach_pose = target_pose_;
    approach_pose.position.z -= 0.05;
//    approach_pose.position.z -= 0.095;
    std::vector<geometry_msgs::msg::Pose> waypoints{approach_pose};
    if (!moveGroupCartesianPath(waypoints)) {
      RCLCPP_ERROR(LOGGER, "Failed to approach object");
      return false;
    }

    publishGripperCommand(6, 10);  // Close gripper
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  void startRecording() {
      force_values_.clear();
      is_recording_ = true;
  }


  void stopRecording() {
      is_recording_ = false;
  }

  double getAverageForce() {
      if (force_values_.empty()) {
          return 0.0;
      }
      
      double sum_fx = 0.0;
      double sum_fy = 0.0;
      double sum_fz = 0.0;
      
      for (const auto& forces : force_values_) {
          sum_fx += forces[0];
          sum_fy += forces[1];
          sum_fz += forces[2];
      }
      
      int n = force_values_.size();
      double avg_fx = sum_fx / n;
      double avg_fy = sum_fy / n;
      double avg_fz = sum_fz / n;
      
      return (avg_fx + avg_fy + avg_fz) / 3.0;  // 三個方向的平均
  }

  double getCurrentForce() {
      if (current_forces_.size() >= 3) {
          return (current_forces_[0] + current_forces_[1] + current_forces_[2]) / 3.0;
      }
      return 0.0;
  }

  void publishForceResult(bool result) {
      std_msgs::msg::Bool msg;
      msg.data = result;
      force_result_publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published force result: %s", result ? "TRUE" : "FALSE");
  }

  void forceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 3) {  // 確保有三個軸的數據
          current_forces_ = {msg->data[0], msg->data[1], msg->data[2]};
          if (is_recording_) {
              force_values_.push_back(current_forces_);
          }
      }
  }

  void moveAlongYAxis(moveit::planning_interface::MoveGroupInterface& move_group, double distance) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
      geometry_msgs::msg::Pose target_pose = current_pose;
      
      target_pose.position.y += distance;
      waypoints.push_back(target_pose);
      
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      if (fraction == -1.0 || fraction < 0.9) {
          RCLCPP_ERROR(LOGGER, "Path planning for Y-axis movement failed");
      } else {
          move_group.execute(trajectory);
      }
  }

  void moveAlongZAxis(moveit::planning_interface::MoveGroupInterface& move_group, double distance) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
      geometry_msgs::msg::Pose target_pose = current_pose;
      
      target_pose.position.z += distance;
      waypoints.push_back(target_pose);
      
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      if (fraction == -1.0 || fraction < 0.9) {
          RCLCPP_ERROR(LOGGER, "Path planning for Z-axis movement failed");
      } else {
          move_group.execute(trajectory);
      }
  }

  bool moveToObservation()
  {
      if (!move_group_arm_) {
          RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
          return false;
      }

      // 定義4個觀察姿勢
      std::vector<geometry_msgs::msg::Pose> observation_poses(5);
      
      // 觀察姿勢1 - 正面觀察
      observation_poses[0].orientation.x = -0.707;
      observation_poses[0].orientation.y = 0.707;
      observation_poses[0].orientation.z = 0.0;
      observation_poses[0].orientation.w = 0.0;
      observation_poses[0].position.x = -0.065;
      observation_poses[0].position.y = -0.494;
      observation_poses[0].position.z = 0.52;

      // 觀察姿勢2 - 側面觀察
      observation_poses[1].orientation.x = -0.707;
      observation_poses[1].orientation.y = 0.707;
      observation_poses[1].orientation.z = 0.0;
      observation_poses[1].orientation.w = 0.0;
      observation_poses[1].position.x = 0.078;
      observation_poses[1].position.y = -0.485;
      observation_poses[1].position.z = 0.19;

      // 觀察姿勢3 - 頂部觀察
      observation_poses[2].orientation.x = 1.0;
      observation_poses[2].orientation.y = 0.0;
      observation_poses[2].orientation.z = 0.0;
      observation_poses[2].orientation.w = 0.0;
      observation_poses[2].position.x = 0.078;
      observation_poses[2].position.y = -0.485;
      observation_poses[2].position.z = 0.275;

      // 觀察姿勢4 - 斜角觀察
      observation_poses[3].orientation.x = 1.0;
      observation_poses[3].orientation.y = 0.0;
      observation_poses[3].orientation.z = 0.0;
      observation_poses[3].orientation.w = 0.0;
      observation_poses[3].position.x = 0.078;
      observation_poses[3].position.y = -0.485;
      observation_poses[3].position.z = 0.185;

      // 觀察姿勢5 - 斜角觀察
      observation_poses[4].orientation.x = 1.0;
      observation_poses[4].orientation.y = 0.0;
      observation_poses[4].orientation.z = 0.0;
      observation_poses[4].orientation.w = 0.0;
      observation_poses[4].position.x = 0.03;
      observation_poses[4].position.y = -0.450;
      observation_poses[4].position.z = 0.2;

      // 執行觀察序列
      RCLCPP_INFO(LOGGER, "Starting observation sequence");

      // 移動到第一個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[0]);
      if (!executePlan()) return false;

///////////////////////////////////////////////////////////////////////////////
/*
      publishGripperCommand(60, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));
      publishGripperCommand(30, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));
      move_group_arm_->setPoseTarget(observation_poses[0]);
      if (!executePlan()) return false;
      publishGripperCommand(10, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));
*/
///////////////////////////////////////////////////////////////////////////////

      // 移動到第二個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[1]);
      if (!executePlan()) return false;

      publishGripperCommand(30, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));

      // 移動到第三個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[2]);
      if (!executePlan()) return false;

      publishGripperCommand(60, 10);  // 調整夾爪開度
      rclcpp::sleep_for(std::chrono::seconds(2));

      // 移動到第四個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[3]);
      if (!executePlan()) return false;

      publishGripperCommand(40, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(60, 10);  // 調整夾爪開度
      rclcpp::sleep_for(std::chrono::seconds(2));

      // 移動到第三個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[2]);
      if (!executePlan()) return false;

      publishGripperCommand(30, 10);  // 略微打開夾爪
      rclcpp::sleep_for(std::chrono::seconds(2));

      // 移動到第五個觀察位置
      move_group_arm_->setPoseTarget(observation_poses[4]);
      if (!executePlan()) return false;

      RCLCPP_INFO(LOGGER, "Observation sequence completed");
      return true;
  }

  bool moveToFinalPosition()
  {
      if (!move_group_arm_) {
          RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
          return false;
      }

      RCLCPP_INFO(LOGGER, "Moving to final position based on orientation type: %d", orientation_type_);

      if (orientation_type_ < 0 || orientation_type_ > 3) {
          RCLCPP_ERROR(LOGGER, "Invalid orientation type: %d", orientation_type_);
          return false;
      }

      // 定義基本姿勢
      std::vector<geometry_msgs::msg::Pose> final_poses(9);
      
      // 直上
      final_poses[0].orientation.x = -0.707;
      final_poses[0].orientation.y = 0.707;
      final_poses[0].orientation.z = 0.0;
      final_poses[0].orientation.w = 0.0;
      final_poses[0].position.x = 0.078;
      final_poses[0].position.y = -0.485;
      final_poses[0].position.z = 0.3;

      // 直下
      final_poses[1].orientation.x = -0.707;
      final_poses[1].orientation.y = 0.707;
      final_poses[1].orientation.z = 0.0;
      final_poses[1].orientation.w = 0.0;
      final_poses[1].position.x = 0.078;
      final_poses[1].position.y = -0.485;
      final_poses[1].position.z = 0.189;

      // 頂部位置
      final_poses[2].orientation.x = 0.97;
      final_poses[2].orientation.y = 0.0;
      final_poses[2].orientation.z = 0.0;
      final_poses[2].orientation.w = 0.243;
      final_poses[2].position.x = 0.078;
      final_poses[2].position.y = -0.3727;
      final_poses[2].position.z = 0.4;

      // 放置位置
      final_poses[3].orientation.x = 0.97;
      final_poses[3].orientation.y = 0.0;
      final_poses[3].orientation.z = 0.0;
      final_poses[3].orientation.w = 0.243;
      final_poses[3].position.x = 0.078;
      final_poses[3].position.y = -0.3727;
      final_poses[3].position.z = 0.1547;

      // 放置位置
      final_poses[4].orientation.x = 0.0;
      final_poses[4].orientation.y = -0.97;
      final_poses[4].orientation.z = 0.243;
      final_poses[4].orientation.w = 0.0;
      final_poses[4].position.x = 0.078;
      final_poses[4].position.y = -0.3727;
      final_poses[4].position.z = 0.4;   

      // 放置位置
      final_poses[5].orientation.x = 0.0;
      final_poses[5].orientation.y = -0.97;
      final_poses[5].orientation.z = 0.243;
      final_poses[5].orientation.w = 0.0;
      final_poses[5].position.x = 0.078;
      final_poses[5].position.y = -0.3727;
      final_poses[5].position.z = 0.1547;

      // 直上
      final_poses[6].orientation.x = 0.707;
      final_poses[6].orientation.y = 0.707;
      final_poses[6].orientation.z = 0.0;
      final_poses[6].orientation.w = 0.0;
      final_poses[6].position.x = 0.078;
      final_poses[6].position.y = -0.485;
      final_poses[6].position.z = 0.3;

      // 直上
      final_poses[7].orientation.x = 0.707;
      final_poses[7].orientation.y = 0.707;
      final_poses[7].orientation.z = 0.0;
      final_poses[7].orientation.w = 0.0;
      final_poses[7].position.x = 0.078;
      final_poses[7].position.y = -0.485;
      final_poses[7].position.z = 0.19;

      // 觀察
      final_poses[8].orientation.x = 1.0;
      final_poses[8].orientation.y = 0.0;
      final_poses[8].orientation.z = 0.0;
      final_poses[8].orientation.w = 0.0;
      final_poses[8].position.x = 0.03;
      final_poses[8].position.y = -0.450;
      final_poses[8].position.z = 0.2;

      switch (orientation_type_) {
          case 0: // Front
          {
              // 移動到正面位置
              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));
/*
              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[1]);
              if (!executePlan()) return false;

              // 打開夾爪
              publishGripperCommand(6, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 打開夾爪
              publishGripperCommand(40, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));
*/
          }
          break;

          case 1: // Side-down
          {
              // 移動到頂部位置
              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[3]);
              if (!executePlan()) return false;

              publishGripperCommand(40, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[5]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

///////////////////////////////////////////////////////////////////////////////
              move_group_arm_->setPoseTarget(final_poses[8]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));     
          }
          break;

          case 2: // Side-top
          {
              // 移動到正面位置
              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[1]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 打開夾爪
              publishGripperCommand(6, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[6]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[7]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));              

              // 打開夾爪
              publishGripperCommand(30, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

         

///////////////////////////////////////////////////////////////////////////////

              // 移動到頂部位置
              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[3]);
              if (!executePlan()) return false;

              publishGripperCommand(40, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[5]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

///////////////////////////////////////////////////////////////////////////////
              move_group_arm_->setPoseTarget(final_poses[8]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));     
          }
          break;

          case 3: // Back
          {
              // 移動到頂部位置
              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[3]);
              if (!executePlan()) return false;

              publishGripperCommand(40, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[5]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

////////////////////////////////////////////////////////////////////////////

              // 移動到正面位置
              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[1]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 打開夾爪
              publishGripperCommand(6, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[6]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[7]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));              

              // 打開夾爪
              publishGripperCommand(30, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

///////////////////////////////////////////////////////////////////////////////

              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到頂部位置
              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 移動到放置位置
              move_group_arm_->setPoseTarget(final_poses[3]);
              if (!executePlan()) return false;

              publishGripperCommand(40, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[4]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[5]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(60, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(final_poses[0]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

///////////////////////////////////////////////////////////////////////////////
              move_group_arm_->setPoseTarget(final_poses[8]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));     
          }
          break;

          default:
              RCLCPP_ERROR(LOGGER, "Invalid orientation type");
              return false;
      }

      return true;
  }

  bool moveToAssemble()
  {
      if (!move_group_arm_) {
          RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
          return false;
      }

      RCLCPP_INFO(LOGGER, "Moving to assembly position");

      // 定義裝配位置
      std::vector<geometry_msgs::msg::Pose> assembly_poses(8);

      assembly_poses[0].orientation.x = -0.707;
      assembly_poses[0].orientation.y = 0.707;
      assembly_poses[0].orientation.z = 0.0;
      assembly_poses[0].orientation.w = 0.0;
      assembly_poses[0].position.x = 0.078;
      assembly_poses[0].position.y = -0.485;
      assembly_poses[0].position.z = 0.3;

      // 直下
      assembly_poses[1].orientation.x = -0.707;
      assembly_poses[1].orientation.y = 0.707;
      assembly_poses[1].orientation.z = 0.0;
      assembly_poses[1].orientation.w = 0.0;
      assembly_poses[1].position.x = 0.078;
      assembly_poses[1].position.y = -0.485;
      assembly_poses[1].position.z = 0.189;

      float y_offset = (cycle_count_ > 0) ? 0.0495 : 0.0;

      // 中間過渡位置
      assembly_poses[2].orientation.x = 1.0;
      assembly_poses[2].orientation.y = 0.0;
      assembly_poses[2].orientation.z = 0.0;
      assembly_poses[2].orientation.w = 0.0;
      assembly_poses[2].position.x = 0.197;
      assembly_poses[2].position.y = -0.603 + y_offset;;
      assembly_poses[2].position.z = 0.225;

      // 最終裝配位置
      assembly_poses[3].orientation.x = 1.0;
      assembly_poses[3].orientation.y = 0.0;
      assembly_poses[3].orientation.z = 0.0;
      assembly_poses[3].orientation.w = 0.0;
      assembly_poses[3].position.x = 0.197;
      assembly_poses[3].position.y = -0.603 + y_offset;;
      assembly_poses[3].position.z = 0.186;

      assembly_poses[4].orientation.x = 0.707;
      assembly_poses[4].orientation.y = 0.707;
      assembly_poses[4].orientation.z = 0.0;
      assembly_poses[4].orientation.w = 0.0;
      assembly_poses[4].position.x = 0.197;
      assembly_poses[4].position.y = -0.603 + y_offset;;
      assembly_poses[4].position.z = 0.19;

      assembly_poses[5].orientation.x = 1.0;
      assembly_poses[5].orientation.y = 0.0;
      assembly_poses[5].orientation.z = 0.0;
      assembly_poses[5].orientation.w = 0.0;
      assembly_poses[5].position.x = 0.206;
      assembly_poses[5].position.y = -0.603 + y_offset;;
      assembly_poses[5].position.z = 0.186;

      assembly_poses[6].orientation.x = 1.0;
      assembly_poses[6].orientation.y = 0.0;
      assembly_poses[6].orientation.z = 0.0;
      assembly_poses[6].orientation.w = 0.0;
      assembly_poses[6].position.x = 0.078;
      assembly_poses[6].position.y = -0.485;
      assembly_poses[6].position.z = 0.3;

      assembly_poses[7].orientation.x = 1.0;
      assembly_poses[7].orientation.y = 0.0;
      assembly_poses[7].orientation.z = 0.0;
      assembly_poses[7].orientation.w = 0.0;
      assembly_poses[7].position.x = 0.078;
      assembly_poses[7].position.y = -0.485;
      assembly_poses[7].position.z = 0.18;


      // 執行移動序列

      move_group_arm_->setPoseTarget(assembly_poses[0]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(30, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[1]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(6, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(30, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[6]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(60, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[7]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(40, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));
    
      publishGripperCommand(60, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[0]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(30, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[1]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(6, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[2]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[3]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(25, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[2]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(6, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[4]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[2]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));

      publishGripperCommand(30, 10);
      rclcpp::sleep_for(std::chrono::seconds(2));

      move_group_arm_->setPoseTarget(assembly_poses[3]);
      if (!executePlan()) return false;
      rclcpp::sleep_for(std::chrono::seconds(2));


      // 進行力變化檢測
      bool fx_changed = false;
      do {
          // 關閉夾爪
          RCLCPP_INFO(LOGGER, "Closing gripper for force detection");
          publishGripperCommand(6, 40);
          rclcpp::sleep_for(std::chrono::seconds(2));

          // 開始記錄力值
          RCLCPP_INFO(LOGGER, "Starting force recording");
          startRecording();
          rclcpp::sleep_for(std::chrono::seconds(2));
          stopRecording();

          double average_force = getAverageForce();
          RCLCPP_INFO(LOGGER, "Average force value: %f", average_force);

          // 移動到檢測位置
          RCLCPP_INFO(LOGGER, "Moving to force detection position");
          move_group_arm_->setPoseTarget(assembly_poses[5]);
          if (!executePlan()) return false;
          rclcpp::sleep_for(std::chrono::seconds(2));

          // 獲取最終力值
          double final_force = getCurrentForce();
          RCLCPP_INFO(LOGGER, "Final force value: %f", final_force);

          // 比較力值變化
          fx_changed = (average_force - final_force) > 1.0;
          RCLCPP_INFO(LOGGER, fx_changed ? "Significant force change detected" : "No significant force change");
          RCLCPP_INFO(LOGGER, "Force difference: %f", average_force - final_force);

          // 發布結果
          publishForceResult(fx_changed);

          if (!fx_changed) {
              // 回到初始位置重試
              move_group_arm_->setPoseTarget(assembly_poses[3]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 調整位置
              //RCLCPP_INFO(LOGGER, "Adjusting position for retry");
              //moveAlongYAxis(*move_group_arm_, 0.005);

              // 打開夾爪
              publishGripperCommand(30, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));

              // 準備下一次嘗試
              move_group_arm_->setPoseTarget(assembly_poses[2]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              move_group_arm_->setPoseTarget(assembly_poses[3]);
              if (!executePlan()) return false;
              rclcpp::sleep_for(std::chrono::seconds(2));

              publishGripperCommand(10, 10);
              rclcpp::sleep_for(std::chrono::seconds(2));
          }
      } while (!fx_changed);

      if (fx_changed) {
          RCLCPP_INFO(LOGGER, "Force detection completed successfully");

          move_group_arm_->setPoseTarget(assembly_poses[3]);
          if (!executePlan()) return false;
          rclcpp::sleep_for(std::chrono::seconds(2));

          // 打開夾爪
          publishGripperCommand(40, 10);
          rclcpp::sleep_for(std::chrono::seconds(2));

          // 回到安全位置
          move_group_arm_->setPoseTarget(assembly_poses[2]);
          if (!executePlan()) return false;
          rclcpp::sleep_for(std::chrono::seconds(2));
      }

      return true;
  }
  
  bool releaseObject()
  {
    RCLCPP_INFO(LOGGER, "Releasing object");
    publishGripperCommand(30, 10);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool executePlan()
  {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing plan");
      move_group_arm_->execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }

    return success;
  }

  bool moveGroupCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Path planned for %f of the total path.", fraction);

    if (fraction > 0.9) {
      move_group_arm_->execute(trajectory);
      return true;
    }
    return false;
  }

  void coordinatesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.position.x = msg->x;
      target_pose_.position.y = msg->y;
      target_pose_.position.z = 0.533;  // 固定高度
      coordinates_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%f, y=%f, z=%f", 
                  msg->x, msg->y, target_pose_.position.z);
      checkStateTransition();
    }
  }

  void orientationCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.orientation = *msg;
      orientation_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received orientation: x=%f, y=%f, z=%f, w=%f", 
                  msg->x, msg->y, msg->z, msg->w);
      checkStateTransition();
    }
  }

  void orientationTypeCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    orientation_type_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received orientation type: %d", orientation_type_);
  }

  void gripperStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    gripper_state_ = msg->data.substr(7);  // Remove "State: " prefix
    RCLCPP_INFO(this->get_logger(), "Gripper state: %s", gripper_state_.c_str());
  }

  void checkStateTransition()
  {
    if (coordinates_received_ && orientation_received_) {
      setState(State::MOVE_TO_OBJECT);
      coordinates_received_ = false;
      orientation_received_ = false;
    }
  }

  void publishGripperCommand(int width, int force)
  {
    std_msgs::msg::Int32MultiArray gripper_command;
    gripper_command.data = {width, force};
    RCLCPP_INFO(LOGGER, "Publishing Gripper Command: width=%d, force=%d", width, force);
    gripper_publisher->publish(gripper_command);
  }

  bool moveToJointPosition(const std::vector<double>& target_position, const std::string& position_name)
  {
    RCLCPP_INFO(LOGGER, "Moving to %s", position_name.c_str());
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setJointValueTarget(target_position);
    move_group_arm_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    int attempts = 0;
    const int max_attempts = 3;
    bool success_arm = false;

    while (!success_arm && attempts < max_attempts) {
      success_arm = (move_group_arm_->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success_arm) {
        success_arm = (move_group_arm_->execute(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      }
      attempts++;
      if (!success_arm && attempts < max_attempts) {
        RCLCPP_WARN(LOGGER, "Attempt %d failed, retrying...", attempts);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (success_arm) {
      RCLCPP_INFO(LOGGER, "Successfully moved to %s", position_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to move to %s after %d attempts", position_name.c_str(), max_attempts);
      return false;
    }
  }

  bool controlGripper(int width, int force, const std::string& operation)
  {
    RCLCPP_INFO(LOGGER, "Performing gripper operation: %s", operation.c_str());
    publishGripperCommand(width, force);
    
    // Wait for gripper operation to complete
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    return true;
  }

  std::string stateToString(State state)
  {
    switch (state) {
      case State::INIT: return "INIT";
      case State::MOVE_TO_HOME: return "MOVE_TO_HOME";
      case State::MOVE_TO_INTERMEDIATE: return "MOVE_TO_INTERMEDIATE";
      case State::WAIT_FOR_COORDINATES: return "WAIT_FOR_COORDINATES";
      case State::MOVE_TO_OBJECT: return "MOVE_TO_OBJECT";
      case State::GRASP_OBJECT: return "GRASP_OBJECT";
      case State::CHECK_GRASP: return "CHECK_GRASP";
      case State::MOVE_TO_OBSERVATION: return "MOVE_TO_OBSERVATION";
      case State::WAIT_FOR_ORIENTATION: return "WAIT_FOR_ORIENTATION";
      case State::MOVE_TO_FINAL_POSITION: return "MOVE_TO_FINAL_POSITION";
      case State::MOVE_TO_ASSEMBLE: return "MOVE_TO_ASSEMBLE";
      case State::RELEASE_OBJECT: return "RELEASE_OBJECT";
      case State::FINISHED: return "FINISHED";
      default: return "UNKNOWN";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}










































/*
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class RobotController : public rclcpp::Node
{
public:
  RobotController() : Node("robot_controller"), state_(State::INIT), orientation_type_(-1), gripper_state_("FALSE")
  {
    coordinates_subscription = this->create_subscription<geometry_msgs::msg::Point>(
      "/lego_coordinates", 10, 
      std::bind(&RobotController::coordinatesCallback, this, std::placeholders::_1));

    orientation_subscription = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/lego_orientation", 10, 
      std::bind(&RobotController::orientationCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    orientation_type_subscription = this->create_subscription<std_msgs::msg::Int8>(
      "/lego_orientation_type", 10,
      std::bind(&RobotController::orientationTypeCallback, this, std::placeholders::_1));
    
    gripper_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/gripper_state", 10, 
      std::bind(&RobotController::gripperStateCallback, this, std::placeholders::_1));   

    gripper_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("custom_gripper", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Robot controller initialized.");
  }

private:
  enum class State {
    INIT,
    MOVE_TO_HOME,
    MOVE_TO_INTERMEDIATE,
    WAIT_AFTER_INTERMEDIATE,
    WAIT_FOR_COORDINATES,
    MOVE_TO_OBJECT,
    GRASP_OBJECT,
    CHECK_GRASP,
    MOVE_TO_OBSERVATION,
    WAIT_FOR_ORIENTATION,
    MOVE_TO_FINAL_POSITION,
    RELEASE_OBJECT,
    FINISHED
  };

  State state_;
  int orientation_type_;
  rclcpp::Time state_start_time_;
  geometry_msgs::msg::Pose target_pose_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coordinates_subscription;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_subscription;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr orientation_type_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr gripper_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string gripper_state_;
  rclcpp::Time gripper_check_start_time_;
  bool coordinates_received_ = false;
  bool orientation_received_ = false;
  rclcpp::Time intermediate_move_time_;

  void timerCallback()
  {
    switch (state_) {
      case State::INIT:
        initializeMoveGroup();
        break;
      case State::MOVE_TO_HOME:
        if (goHome()) {
          setState(State::MOVE_TO_INTERMEDIATE);
        }
        break;
      case State::MOVE_TO_INTERMEDIATE:
        if (moveToIntermediate()) {
          intermediate_move_time_ = this->now();
          setState(State::WAIT_AFTER_INTERMEDIATE);
        }
        break;
      case State::WAIT_AFTER_INTERMEDIATE:
        if ((this->now() - intermediate_move_time_).seconds() >= 3.0) {
          setState(State::WAIT_FOR_COORDINATES);
        }
        break;
      case State::WAIT_FOR_COORDINATES:
        // 在這個狀態下，我們等待坐標和方向信息
        // 實際的處理在coordinatesCallback和orientationCallback中進行
        break;
      case State::MOVE_TO_OBJECT:
        if (moveToObject()) {
          setState(State::GRASP_OBJECT);
        }
        break;
      case State::GRASP_OBJECT:
        if (graspObject()) {
          gripper_check_start_time_ = this->now();
          setState(State::CHECK_GRASP);
        }
        break;
      case State::CHECK_GRASP:
        if ((this->now() - gripper_check_start_time_).seconds() >= 3.0) {
          if (gripper_state_ == "TRUE") {
            RCLCPP_INFO(this->get_logger(), "Grasp successful");
            setState(State::MOVE_TO_OBSERVATION);
          } else {
            RCLCPP_INFO(this->get_logger(), "Grasp failed, resetting to wait for coordinates");
            setState(State::WAIT_FOR_COORDINATES);
          }
        }
        break;
      case State::MOVE_TO_OBSERVATION:
        if (moveToObservation()) {
          setState(State::WAIT_FOR_ORIENTATION);
          state_start_time_ = this->now();
        }
        break;
      case State::WAIT_FOR_ORIENTATION:
        if (orientation_type_ != -1 || 
            (this->now() - state_start_time_).seconds() > 3.0) {
          setState(State::MOVE_TO_FINAL_POSITION);  
        }
        break;
      case State::MOVE_TO_FINAL_POSITION:
        if (moveToFinalPosition()) {
          setState(State::RELEASE_OBJECT);
        }
        break;
      case State::RELEASE_OBJECT:
        if (releaseObject()) {
          setState(State::FINISHED);
        }
        break;
      case State::FINISHED:
        RCLCPP_INFO(this->get_logger(), "Task completed.");
        break;
    }
  }

  void setState(State new_state)
  {
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "Entering state: %s", stateToString(state_).c_str());
  }

  void initializeMoveGroup()
  {
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_arm_->setMaxVelocityScalingFactor(0.1);
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    setState(State::MOVE_TO_HOME);
  }

  bool goHome()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    move_group_arm_->setStartStateToCurrentState();
    RCLCPP_INFO(this->get_logger(), "Going Home");

    std::vector<double> joint_group_positions = {-1.64, -2.27, 1.92, -1.22, -1.57, 1.57};
    move_group_arm_->setPlanningTime(20.0);
    move_group_arm_->setJointValueTarget(joint_group_positions);

    return executePlan();
  }

  bool moveToIntermediate()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to intermediate position");
    std::vector<double> intermediate_position = {-2.17, -1.24, 0.68, -1.01, -1.57, 2.52};
    move_group_arm_->setJointValueTarget(intermediate_position);
    move_group_arm_->setPlanningTime(10.0);

    return executePlan();
  }

  bool moveToObject()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to object");
    move_group_arm_->setPoseTarget(target_pose_);

    return executePlan();
  }

  bool graspObject()
  {
    RCLCPP_INFO(LOGGER, "Grasping object");
    publishGripperCommand(40, 10);  // Open gripper
    rclcpp::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose approach_pose = target_pose_;
    approach_pose.position.z -= 0.095;
    std::vector<geometry_msgs::msg::Pose> waypoints{approach_pose};
    if (!moveGroupCartesianPath(waypoints)) {
      RCLCPP_ERROR(LOGGER, "Failed to approach object");
      return false;
    }

    publishGripperCommand(6, 10);  // Close gripper
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool moveToObservation()
  {
    RCLCPP_INFO(LOGGER, "Moving to observation position");
    if (!moveToJointPosition({-1.93, -1.58, 1.48, -1.49, -1.56, 2.74}, "Observation 0")) return false;
    // Move to first position
    if (!moveToJointPosition({-1.65, -1.42, 2.07, -2.24, -1.57, 1.53}, "Observation 1")) return false;
    
    // Open gripper
    if (!controlGripper(30, 10, "Open gripper")) return false;
    
    // Move to second position
    if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.52}, "Observation 2")) return false;
    
    // Move to third position
    if (!moveToJointPosition({-1.75, -1.54, 2.13, -2.18, -1.57, 2.92}, "Observation 3")) return false;
    
    RCLCPP_INFO(LOGGER, "Successfully moved to observation position");
    return true;
  }

  bool moveToFinalPosition()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(LOGGER, "Moving to final position based on orientation type: %d", orientation_type_);

    if (orientation_type_ < 0 || orientation_type_ > 3) {
      RCLCPP_ERROR(LOGGER, "Invalid orientation type: %d", orientation_type_);
      return false;
    }

    switch (orientation_type_) {
      case 0: // Front
        {

          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.52}, "Home 1")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, 1.52}, "Home 2")) return false;
          if (!controlGripper(30, 10, "Open gripper")) return false;
        }
        break;
      case 1: // Side-down
        {
          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 5")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      case 2: // Side-top
        {
          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, -1.3}, "Home 1")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, -1.62}, "Home 2")) return false;
          if (!controlGripper(6, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.3}, "Home 3")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, 1.52}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 5")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      case 3: // Back
        {

          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.57, 2.54, -3.20, -1.46, 2.91}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.35, 1.78, -1.94, -1.53, 2.91}, "Home 4.5")) return false;
          if (!moveToJointPosition({-1.69, -1.13, 1.57, -1.61, -1.53, 2.91}, "Home 5")) return false;
          if (!moveToJointPosition({-1.68, -1.03, 1.39, -1.31, -1.49, 2.91}, "Home 5.5")) return false;
          if (!moveToJointPosition({-1.60, -0.74, 1.15, -1.06, -1.63, 3.13}, "Home 6")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.13, 1.57, -1.61, -1.53, 2.91}, "Home 7")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 8")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 9")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 10")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 11")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Invalid orientation type");
        return false;
    }

    return true;
  }

  bool releaseObject()
  {
    RCLCPP_INFO(LOGGER, "Releasing object");
    publishGripperCommand(30, 10);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool executePlan()
  {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing plan");
      move_group_arm_->execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }

    return success;
  }

  bool moveGroupCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Path planned for %f of the total path.", fraction);

    if (fraction > 0.9) {
      move_group_arm_->execute(trajectory);
      return true;
    }
    return false;
  }

  void coordinatesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.position.x = msg->x;
      target_pose_.position.y = msg->y;
      target_pose_.position.z = 0.533;  // 固定高度
      coordinates_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%f, y=%f, z=%f", 
                  msg->x, msg->y, target_pose_.position.z);
      checkStateTransition();
    }
  }

  void orientationCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.orientation = *msg;
      orientation_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received orientation: x=%f, y=%f, z=%f, w=%f", 
                  msg->x, msg->y, msg->z, msg->w);
      checkStateTransition();
    }
  }

  void orientationTypeCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    orientation_type_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received orientation type: %d", orientation_type_);
  }

  void gripperStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    gripper_state_ = msg->data.substr(7);  // Remove "State: " prefix
    RCLCPP_INFO(this->get_logger(), "Gripper state: %s", gripper_state_.c_str());
  }

  void checkStateTransition()
  {
    if (coordinates_received_ && orientation_received_) {
      setState(State::MOVE_TO_OBJECT);
      coordinates_received_ = false;
      orientation_received_ = false;
    }
  }

  void publishGripperCommand(int width, int force)
  {
    std_msgs::msg::Int32MultiArray gripper_command;
    gripper_command.data = {width, force};
    RCLCPP_INFO(LOGGER, "Publishing Gripper Command: width=%d, force=%d", width, force);
    gripper_publisher->publish(gripper_command);
  }

  bool moveToJointPosition(const std::vector<double>& target_position, const std::string& position_name)
  {
    RCLCPP_INFO(LOGGER, "Moving to %s", position_name.c_str());
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setJointValueTarget(target_position);
    move_group_arm_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    int attempts = 0;
    const int max_attempts = 3;
    bool success_arm = false;

    while (!success_arm && attempts < max_attempts) {
      success_arm = (move_group_arm_->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success_arm) {
        success_arm = (move_group_arm_->execute(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      }
      attempts++;
      if (!success_arm && attempts < max_attempts) {
        RCLCPP_WARN(LOGGER, "Attempt %d failed, retrying...", attempts);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (success_arm) {
      RCLCPP_INFO(LOGGER, "Successfully moved to %s", position_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to move to %s after %d attempts", position_name.c_str(), max_attempts);
      return false;
    }
  }

  bool controlGripper(int width, int force, const std::string& operation)
  {
    RCLCPP_INFO(LOGGER, "Performing gripper operation: %s", operation.c_str());
    publishGripperCommand(width, force);
    
    // Wait for gripper operation to complete
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    return true;
  }

  std::string stateToString(State state)
  {
    switch (state) {
      case State::INIT: return "INIT";
      case State::MOVE_TO_HOME: return "MOVE_TO_HOME";
      case State::MOVE_TO_INTERMEDIATE: return "MOVE_TO_INTERMEDIATE";
      case State::WAIT_FOR_COORDINATES: return "WAIT_FOR_COORDINATES";
      case State::MOVE_TO_OBJECT: return "MOVE_TO_OBJECT";
      case State::GRASP_OBJECT: return "GRASP_OBJECT";
      case State::CHECK_GRASP: return "CHECK_GRASP";
      case State::MOVE_TO_OBSERVATION: return "MOVE_TO_OBSERVATION";
      case State::WAIT_FOR_ORIENTATION: return "WAIT_FOR_ORIENTATION";
      case State::MOVE_TO_FINAL_POSITION: return "MOVE_TO_FINAL_POSITION";
      case State::RELEASE_OBJECT: return "RELEASE_OBJECT";
      case State::FINISHED: return "FINISHED";
      default: return "UNKNOWN";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
<<<<<<< HEAD
}
*/

























































/*
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class RobotController : public rclcpp::Node
{
public:
  RobotController() : Node("robot_controller"), state_(State::INIT), orientation_type_(-1), gripper_state_("FALSE")
  {
    coordinates_subscription = this->create_subscription<geometry_msgs::msg::Point>(
      "/lego_coordinates", 10, 
      std::bind(&RobotController::coordinatesCallback, this, std::placeholders::_1));

    orientation_subscription = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/lego_orientation", 10, 
      std::bind(&RobotController::orientationCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    orientation_type_subscription = this->create_subscription<std_msgs::msg::Int8>(
      "/lego_orientation_type", 10,
      std::bind(&RobotController::orientationTypeCallback, this, std::placeholders::_1));
    
    gripper_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/gripper_state", 10, 
      std::bind(&RobotController::gripperStateCallback, this, std::placeholders::_1));   

    gripper_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("custom_gripper", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&RobotController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Robot controller initialized.");
  }

private:
  enum class State {
    INIT,
    MOVE_TO_HOME,
    MOVE_TO_INTERMEDIATE,
    WAIT_AFTER_INTERMEDIATE,
    WAIT_FOR_COORDINATES,
    MOVE_TO_OBJECT,
    GRASP_OBJECT,
    CHECK_GRASP,
    MOVE_TO_OBSERVATION,
    WAIT_FOR_ORIENTATION,
    MOVE_TO_FINAL_POSITION,
    RELEASE_OBJECT,
    FINISHED
  };

  State state_;
  int orientation_type_;
  rclcpp::Time state_start_time_;
  geometry_msgs::msg::Pose target_pose_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coordinates_subscription;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_subscription;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr orientation_type_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr gripper_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string gripper_state_;
  rclcpp::Time gripper_check_start_time_;
  bool coordinates_received_ = false;
  bool orientation_received_ = false;
  rclcpp::Time intermediate_move_time_;

  void timerCallback()
  {
    switch (state_) {
      case State::INIT:
        initializeMoveGroup();
        break;
      case State::MOVE_TO_HOME:
        if (goHome()) {
          setState(State::MOVE_TO_INTERMEDIATE);
        }
        break;
      case State::MOVE_TO_INTERMEDIATE:
        if (moveToIntermediate()) {
          intermediate_move_time_ = this->now();
          setState(State::WAIT_AFTER_INTERMEDIATE);
        }
        break;
      case State::WAIT_AFTER_INTERMEDIATE:
        if ((this->now() - intermediate_move_time_).seconds() >= 3.0) {
          setState(State::WAIT_FOR_COORDINATES);
        }
        break;
      case State::WAIT_FOR_COORDINATES:
        // 在這個狀態下，我們等待坐標和方向信息
        // 實際的處理在coordinatesCallback和orientationCallback中進行
        break;
      case State::MOVE_TO_OBJECT:
        if (moveToObject()) {
          setState(State::GRASP_OBJECT);
        }
        break;
      case State::GRASP_OBJECT:
        if (graspObject()) {
          gripper_check_start_time_ = this->now();
          setState(State::CHECK_GRASP);
        }
        break;
      case State::CHECK_GRASP:
        if ((this->now() - gripper_check_start_time_).seconds() >= 3.0) {
          if (gripper_state_ == "TRUE") {
            RCLCPP_INFO(this->get_logger(), "Grasp successful");
            setState(State::MOVE_TO_OBSERVATION);
          } else {
            RCLCPP_INFO(this->get_logger(), "Grasp failed, resetting to wait for coordinates");
            setState(State::WAIT_FOR_COORDINATES);
          }
        }
        break;
      case State::MOVE_TO_OBSERVATION:
        if (moveToObservation()) {
          setState(State::WAIT_FOR_ORIENTATION);
          state_start_time_ = this->now();
        }
        break;
      case State::WAIT_FOR_ORIENTATION:
        if (orientation_type_ != -1 || 
            (this->now() - state_start_time_).seconds() > 3.0) {
          setState(State::MOVE_TO_FINAL_POSITION);  
        }
        break;
      case State::MOVE_TO_FINAL_POSITION:
        if (moveToFinalPosition()) {
          setState(State::RELEASE_OBJECT);
        }
        break;
      case State::RELEASE_OBJECT:
        if (releaseObject()) {
          setState(State::FINISHED);
        }
        break;
      case State::FINISHED:
        RCLCPP_INFO(this->get_logger(), "Task completed.");
        break;
    }
  }

  void setState(State new_state)
  {
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "Entering state: %s", stateToString(state_).c_str());
  }

  void initializeMoveGroup()
  {
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_arm_->setMaxVelocityScalingFactor(0.1);
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    setState(State::MOVE_TO_HOME);
  }

  bool goHome()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    move_group_arm_->setStartStateToCurrentState();
    RCLCPP_INFO(this->get_logger(), "Going Home");

    std::vector<double> joint_group_positions = {-1.64, -2.27, 1.92, -1.22, -1.57, 1.57};
    move_group_arm_->setPlanningTime(20.0);
    move_group_arm_->setJointValueTarget(joint_group_positions);

    return executePlan();
  }

  bool moveToIntermediate()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to intermediate position");
    std::vector<double> intermediate_position = {-2.17, -1.24, 0.68, -1.01, -1.57, 2.52};
    move_group_arm_->setJointValueTarget(intermediate_position);
    move_group_arm_->setPlanningTime(10.0);

    return executePlan();
  }

  bool moveToObject()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to object");
    move_group_arm_->setPoseTarget(target_pose_);

    return executePlan();
  }

  bool graspObject()
  {
    RCLCPP_INFO(LOGGER, "Grasping object");
    publishGripperCommand(40, 10);  // Open gripper
    rclcpp::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose approach_pose = target_pose_;
    approach_pose.position.z -= 0.095;
    std::vector<geometry_msgs::msg::Pose> waypoints{approach_pose};
    if (!moveGroupCartesianPath(waypoints)) {
      RCLCPP_ERROR(LOGGER, "Failed to approach object");
      return false;
    }

    publishGripperCommand(6, 10);  // Close gripper
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool moveToObservation()
  {
    RCLCPP_INFO(LOGGER, "Moving to observation position");
    if (!moveToJointPosition({-1.93, -1.58, 1.48, -1.49, -1.56, 2.74}, "Observation 0")) return false;
    // Move to first position
    if (!moveToJointPosition({-1.65, -1.42, 2.07, -2.24, -1.57, 1.53}, "Observation 1")) return false;
    
    // Open gripper
    if (!controlGripper(30, 10, "Open gripper")) return false;
    
    // Move to second position
    if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.52}, "Observation 2")) return false;
    
    // Move to third position
    if (!moveToJointPosition({-1.75, -1.54, 2.13, -2.18, -1.57, 2.92}, "Observation 3")) return false;
    
    RCLCPP_INFO(LOGGER, "Successfully moved to observation position");
    return true;
  }

  bool moveToFinalPosition()
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    RCLCPP_INFO(LOGGER, "Moving to final position based on orientation type: %d", orientation_type_);

    if (orientation_type_ < 0 || orientation_type_ > 3) {
      RCLCPP_ERROR(LOGGER, "Invalid orientation type: %d", orientation_type_);
      return false;
    }

    switch (orientation_type_) {
      case 0: // Front
        {

          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.52}, "Home 1")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, 1.52}, "Home 2")) return false;
          if (!controlGripper(30, 10, "Open gripper")) return false;
        }
        break;
      case 1: // Side-down
        {
          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 5")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      case 2: // Side-top
        {
          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, -1.3}, "Home 1")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, -1.62}, "Home 2")) return false;
          if (!controlGripper(6, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.64, -1.49, 2.01, -2.11, -1.57, 1.3}, "Home 3")) return false;
          if (!moveToJointPosition({-1.64, -1.42, 2.07, -2.24, -1.57, 1.52}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 5")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      case 3: // Back
        {

          if (!moveToJointPosition({-1.64, -1.55, 1.93, -1.97, -1.57, 1.52}, "Home 1")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 2")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 3")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.57, 2.54, -3.20, -1.46, 2.91}, "Home 4")) return false;
          if (!moveToJointPosition({-1.69, -1.35, 1.78, -1.94, -1.53, 2.91}, "Home 4.5")) return false;
          if (!moveToJointPosition({-1.69, -1.13, 1.57, -1.61, -1.53, 2.91}, "Home 5")) return false;
          if (!moveToJointPosition({-1.68, -1.03, 1.39, -1.31, -1.49, 2.91}, "Home 5.5")) return false;
          if (!moveToJointPosition({-1.60, -0.74, 1.15, -1.06, -1.63, 3.13}, "Home 6")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.13, 1.57, -1.61, -1.53, 2.91}, "Home 7")) return false;
          if (!moveToJointPosition({-1.69, -1.48, 2.67, -3.55, -1.47, 2.98}, "Home 8")) return false;
          if (!moveToJointPosition({-1.70, -1.35, 2.68, -3.65, -1.50, 2.98}, "Home 9")) return false;
          if (!controlGripper(30, 10, "Close gripper")) return false;
          if (!moveToJointPosition({-1.69, -1.54, 2.65, -3.67, -1.31, 2.09}, "Home 10")) return false;
          if (!moveToJointPosition({-1.69, -1.33, 2.71, -3.72, -1.50, -0.11}, "Home 11")) return false;
          if (!controlGripper(60, 10, "Open gripper")) return false;
        }
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Invalid orientation type");
        return false;
    }

    return true;
  }

  bool releaseObject()
  {
    RCLCPP_INFO(LOGGER, "Releasing object");
    publishGripperCommand(30, 10);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool executePlan()
  {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing plan");
      move_group_arm_->execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }

    return success;
  }

  bool moveGroupCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Path planned for %f of the total path.", fraction);

    if (fraction > 0.9) {
      move_group_arm_->execute(trajectory);
      return true;
    }
    return false;
  }

  void coordinatesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.position.x = msg->x;
      target_pose_.position.y = msg->y;
      target_pose_.position.z = 0.533;  // 固定高度
      coordinates_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%f, y=%f, z=%f", 
                  msg->x, msg->y, target_pose_.position.z);
      checkStateTransition();
    }
  }

  void orientationCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    if (state_ == State::WAIT_FOR_COORDINATES) {
      target_pose_.orientation = *msg;
      orientation_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received orientation: x=%f, y=%f, z=%f, w=%f", 
                  msg->x, msg->y, msg->z, msg->w);
      checkStateTransition();
    }
  }

  void orientationTypeCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    orientation_type_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received orientation type: %d", orientation_type_);
  }

  void gripperStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    gripper_state_ = msg->data.substr(7);  // Remove "State: " prefix
    RCLCPP_INFO(this->get_logger(), "Gripper state: %s", gripper_state_.c_str());
  }

  void checkStateTransition()
  {
    if (coordinates_received_ && orientation_received_) {
      setState(State::MOVE_TO_OBJECT);
      coordinates_received_ = false;
      orientation_received_ = false;
    }
  }

  void publishGripperCommand(int width, int force)
  {
    std_msgs::msg::Int32MultiArray gripper_command;
    gripper_command.data = {width, force};
    RCLCPP_INFO(LOGGER, "Publishing Gripper Command: width=%d, force=%d", width, force);
    gripper_publisher->publish(gripper_command);
  }

  bool moveToJointPosition(const std::vector<double>& target_position, const std::string& position_name)
  {
    RCLCPP_INFO(LOGGER, "Moving to %s", position_name.c_str());
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setJointValueTarget(target_position);
    move_group_arm_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    int attempts = 0;
    const int max_attempts = 3;
    bool success_arm = false;

    while (!success_arm && attempts < max_attempts) {
      success_arm = (move_group_arm_->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success_arm) {
        success_arm = (move_group_arm_->execute(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      }
      attempts++;
      if (!success_arm && attempts < max_attempts) {
        RCLCPP_WARN(LOGGER, "Attempt %d failed, retrying...", attempts);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (success_arm) {
      RCLCPP_INFO(LOGGER, "Successfully moved to %s", position_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to move to %s after %d attempts", position_name.c_str(), max_attempts);
      return false;
    }
  }

  bool controlGripper(int width, int force, const std::string& operation)
  {
    RCLCPP_INFO(LOGGER, "Performing gripper operation: %s", operation.c_str());
    publishGripperCommand(width, force);
    
    // Wait for gripper operation to complete
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    return true;
  }

  std::string stateToString(State state)
  {
    switch (state) {
      case State::INIT: return "INIT";
      case State::MOVE_TO_HOME: return "MOVE_TO_HOME";
      case State::MOVE_TO_INTERMEDIATE: return "MOVE_TO_INTERMEDIATE";
      case State::WAIT_FOR_COORDINATES: return "WAIT_FOR_COORDINATES";
      case State::MOVE_TO_OBJECT: return "MOVE_TO_OBJECT";
      case State::GRASP_OBJECT: return "GRASP_OBJECT";
      case State::CHECK_GRASP: return "CHECK_GRASP";
      case State::MOVE_TO_OBSERVATION: return "MOVE_TO_OBSERVATION";
      case State::WAIT_FOR_ORIENTATION: return "WAIT_FOR_ORIENTATION";
      case State::MOVE_TO_FINAL_POSITION: return "MOVE_TO_FINAL_POSITION";
      case State::RELEASE_OBJECT: return "RELEASE_OBJECT";
      case State::FINISHED: return "FINISHED";
      default: return "UNKNOWN";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/

