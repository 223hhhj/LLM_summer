#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  //rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),16); //we don't need multithreadedexecutor
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator"; // MoveIt operates on sets of joints called "PLANNING_GROUP_ARM" and stores them in an object
  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  //**設定移動速度（0.1是預設，1.0最快）（不建議調太快，很危險）**
  move_group_arm.setMaxVelocityScalingFactor(0.5);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10); //需要10 seconds来获取最新的状态信息, 这意味着如果在10秒内无法获取状态信息, 函数将返回一个nullptr
  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  
  // Go Home //joint控制
  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going Home");
  
  // joint control特定格式
  // 都是弧度
  joint_group_positions_arm[0] = -1.64;  // base 
  joint_group_positions_arm[1] = -2.27;  // shoulder
  joint_group_positions_arm[2] = 1.92;  // elbow
  joint_group_positions_arm[3] = -1.22;  // Wrist 1s
  joint_group_positions_arm[4] = -1.57;  // Wrist 2
  joint_group_positions_arm[5] = 1.57;  // Wrist 3

  move_group_arm.setPlanningTime(10.0); // increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group_arm.setJointValueTarget(joint_group_positions_arm);//設定目標 指令
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm; //声明了一个变量my_plan_arm//這行寫一次就行
  bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);//規劃路徑 指令(my_plan_arm名稱為自取)>並且確認planning 是否 successful //bool寫一次就行

  // Execute
  int attempts1 = 0;
  int max_attempts =10;
  if (success_arm == true){
    move_group_arm.execute(my_plan_arm);
  }
  else {
    while(success_arm == false && attempts1 < max_attempts){
      success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      attempts1++;
    }
    move_group_arm.execute(my_plan_arm);
  }
  

  
  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
  // pose control特定格式, orientation都是Quaternion
  
  //可用這兩行將euler angle轉成quaternion, 
  //ref1: https://wiki.ros.org/tf2/Tutorials/Quaternions
  //ref2: https://wiki.ros.org/tf2/Tutorials/Quaternions
  //tf2::Quaternion quaternion; // 
  //quaternion.setRPY(roll, pitch, yaw); //再搭配下面的四行

  geometry_msgs::msg::Pose target_pose1; 

  target_pose1.orientation.w = 0.707;//quaternion.getW();
  target_pose1.orientation.x = 0.0;//quaternion.getX();
  target_pose1.orientation.y = 0.707;//quaternion.getY();
  target_pose1.orientation.z = 0.0;//quaternion.getZ();
  
  target_pose1.position.x = 0.2;
  target_pose1.position.y = -0.5;
  target_pose1.position.z = 0.0;

  move_group_arm.setPoseTarget(target_pose1);//設定目標 指令
  success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  int attempts2 = 0;
  if (success_arm == true) { 
  move_group_arm.execute(my_plan_arm);
  }
  else {
    while(success_arm == false && attempts2 < max_attempts){
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    attempts2++;
    }
    move_group_arm.execute(my_plan_arm);
  }


  /*
  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  geometry_msgs::msg::Pose target_pose2 = target_pose1;

  //依照編寫順序執行或同時執行
  target_pose2.position.x -= 0.1;
  target_pose2.position.y -= 0.1;
  target_pose2.position.z += 0.2;
  approach_waypoints.push_back(target_pose2); //push_back是特定語法
  
  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  //jump_threshold是在计算机器人臂执行笛卡尔路径（Cartesian Path）时使用的一个参数，
  //特别是在MoveIt中，这个参数用于控制路径规划时关节空间（joint space）中的最大跳跃
  //当设置为 0.0 时，意味着禁用了跳跃检测
  const double eef_step = 0.01; //We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian translation
  double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
  //fraction这个返回值表示函数尝试按照你指定的路径（通过一系列的目标位姿）进行规划，成功规划的部分占你期望的整个路径的比例。
  RCLCPP_INFO(LOGGER, "Path planned for %f of the total path.", fraction);
  
  // Execute 
  if(fraction != -1.0){ //滿足要求
  move_group_arm.execute(trajectory_approach);
  }
  */
  

  rclcpp::shutdown();
  return 0;
}