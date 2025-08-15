# dual_arm_collaboration/scripts/dual_arm_coordinator.py
#!/usr/bin/env python3
"""
雙臂協作控制器
整合 UR5 和 ARM0710 的協作運動控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
import threading
import time
import sys
from typing import List, Dict, Optional, Tuple
from enum import Enum

class CollaborationMode(Enum):
    """協作模式枚舉"""
    SEQUENTIAL = "sequential"      # 順序執行
    PARALLEL = "parallel"          # 平行執行  
    SYNCHRONIZED = "synchronized"  # 同步執行
    COORDINATED = "coordinated"    # 協調執行

class SafetyStatus(Enum):
    """安全狀態枚舉"""
    SAFE = "safe"
    WARNING = "warning"
    DANGER = "danger"
    EMERGENCY_STOP = "emergency_stop"

class DualArmCoordinator(Node):
    """雙臂協作控制器"""
    
    def __init__(self):
        super().__init__('dual_arm_coordinator')
        
        # 參數設定
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ur5_planning_group', 'ur5_manipulator'),
                ('arm0710_planning_group', 'arm0710_manipulator'),
                ('safety_distance', 0.2),
                ('warning_distance', 0.3),
                ('emergency_distance', 0.1),
                ('collaborative_speed', 0.3),
                ('normal_speed', 0.5),
            ]
        )
        
        # 獲取參數
        self.ur5_group_name = self.get_parameter('ur5_planning_group').value
        self.arm0710_group_name = self.get_parameter('arm0710_planning_group').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        self.collaborative_speed = self.get_parameter('collaborative_speed').value
        self.normal_speed = self.get_parameter('normal_speed').value

        # MoveIt 初始化
        self.init_moveit()
        
        # Action Clients
        self.init_action_clients()
        
        # 狀態變數
        self.ur5_current_pose = None
        self.arm0710_current_pose = None
        self.ur5_joint_states = None
        self.arm0710_joint_states = None
        self.safety_status = SafetyStatus.SAFE
        
        # 協作任務佇列
        self.task_queue = []
        self.current_task = None
        self.is_executing = False
        
        # 發布者
        self.safety_status_pub = self.create_publisher(Bool, '/dual_arm/safety_status', 10)
        self.task_status_pub = self.create_publisher(Int32MultiArray, '/dual_arm/task_status', 10)
        
        # 訂閱者
        self.ur5_joint_sub = self.create_subscription(
            JointState, '/joint_states', self.ur5_joint_callback, 10)
        self.arm0710_joint_sub = self.create_subscription(
            JointState, '/arm0710/joint_states', self.arm0710_joint_callback, 10)
        
        # 安全監控定時器
        self.safety_timer = self.create_timer(0.1, self.safety_monitor_callback)  # 10Hz
        
        # 任務執行定時器
        self.task_timer = self.create_timer(0.5, self.task_execution_callback)  # 2Hz
        
        self.get_logger().info("🤖 雙臂協作控制器已啟動")
        
    def init_moveit(self):
        """初始化 MoveIt"""
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            
            # 機器人模型
            self.robot = moveit_commander.RobotCommander()
            
            # 規劃場景
            self.scene = moveit_commander.PlanningSceneInterface()
            
            # 等待 MoveIt 初始化
            time.sleep(2.0)
            
            # UR5 規劃群組
            self.ur5_group = moveit_commander.MoveGroupCommander(self.ur5_group_name)
            self.ur5_group.set_max_velocity_scaling_factor(self.collaborative_speed)
            self.ur5_group.set_max_acceleration_scaling_factor(self.collaborative_speed)
            
            # ARM0710 規劃群組 (如果 MoveIt 支援)
            try:
                self.arm0710_group = moveit_commander.MoveGroupCommander(self.arm0710_group_name)
                self.arm0710_group.set_max_velocity_scaling_factor(self.collaborative_speed)
                self.arm0710_group.set_max_acceleration_scaling_factor(self.collaborative_speed)
                self.arm0710_moveit_available = True
            except:
                self.get_logger().warn("ARM0710 MoveIt 群組未找到，將使用 Action Client")
                self.arm0710_moveit_available = False
            
            self.get_logger().info("✅ MoveIt 初始化完成")
            
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt 初始化失敗: {e}")
            
    def init_action_clients(self):
        """初始化 Action Clients"""
        # UR5 Action Client (通過 MoveIt 使用)
        
        # ARM0710 Action Client
        self.arm0710_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm0710_follow_joint_trajectory')
        
        self.get_logger().info("⚡ Action Clients 已初始化")

    def ur5_joint_callback(self, msg: JointState):
        """UR5 關節狀態回調"""
        # 過濾出 UR5 關節
        ur5_indices = []
        for i, name in enumerate(msg.name):
            if name.startswith('ur5_'):
                ur5_indices.append(i)
        
        if ur5_indices:
            self.ur5_joint_states = {
                'names': [msg.name[i] for i in ur5_indices],
                'positions': [msg.position[i] for i in ur5_indices],
                'velocities': [msg.velocity[i] if msg.velocity else 0.0 for i in ur5_indices],
            }
            
            # 計算末端執行器位置
            try:
                self.ur5_current_pose = self.ur5_group.get_current_pose().pose
            except:
                pass

    def arm0710_joint_callback(self, msg: JointState):
        """ARM0710 關節狀態回調"""
        self.arm0710_joint_states = {
            'names': msg.name,
            'positions': msg.position,
            'velocities': msg.velocity if msg.velocity else [0.0] * len(msg.position),
        }
        
        # 計算末端執行器位置 (需要正向運動學)
        self.arm0710_current_pose = self.calculate_arm0710_end_effector_pose()

    def calculate_arm0710_end_effector_pose(self) -> Optional[Pose]:
        """計算 ARM0710 末端執行器位置 (簡化版)"""
        if not self.arm0710_joint_states:
            return None
        
        # 這裡需要實現正向運動學計算
        # 暫時返回一個估算位置
        pose = Pose()
        pose.position.x = 0.5  # 需要根據實際運動學計算
        pose.position.y = 0.8
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        
        return pose

    def safety_monitor_callback(self):
        """安全監控回調"""
        if not (self.ur5_current_pose and self.arm0710_current_pose):
            return
        
        # 計算兩個末端執行器之間的距離
        distance = self.calculate_distance(self.ur5_current_pose, self.arm0710_current_pose)
        
        # 更新安全狀態
        previous_status = self.safety_status
        
        if distance < self.emergency_distance:
            self.safety_status = SafetyStatus.EMERGENCY_STOP
        elif distance < self.safety_distance:
            self.safety_status = SafetyStatus.DANGER
        elif distance < self.warning_distance:
            self.safety_status = SafetyStatus.WARNING
        else:
            self.safety_status = SafetyStatus.SAFE
        
        # 狀態改變時記錄
        if self.safety_status != previous_status:
            self.get_logger().info(f"🛡️ 安全狀態變更: {previous_status.value} -> {self.safety_status.value}")
            
            # 緊急停止處理
            if self.safety_status == SafetyStatus.EMERGENCY_STOP:
                self.emergency_stop()
        
        # 發布安全狀態
        safety_msg = Bool()
        safety_msg.data = (self.safety_status == SafetyStatus.SAFE)
        self.safety_status_pub.publish(safety_msg)

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """計算兩個位置之間的歐氏距離"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def emergency_stop(self):
        """緊急停止"""
        self.get_logger().error("🚨 緊急停止！兩臂距離過近！")
        
        # 停止所有運動
        try:
            self.ur5_group.stop()
        except:
            pass
        
        # 清空任務佇列
        self.task_queue.clear()
        self.current_task = None
        self.is_executing = False

    def task_execution_callback(self):
        """任務執行回調"""
        # 安全檢查
        if self.safety_status in [SafetyStatus.DANGER, SafetyStatus.EMERGENCY_STOP]:
            return
        
        # 執行任務佇列
        if not self.is_executing and self.task_queue:
            self.execute_next_task()

    def add_collaborative_task(self, task_config: Dict):
        """添加協作任務"""
        required_keys = ['task_type', 'mode', 'ur5_target', 'arm0710_target']
        if not all(key in task_config for key in required_keys):
            self.get_logger().error("❌ 任務配置不完整")
            return False
        
        task_config['timestamp'] = time.time()
        task_config['id'] = len(self.task_queue)
        
        self.task_queue.append(task_config)
        self.get_logger().info(f"➕ 新增協作任務: {task_config['task_type']}")
        return True

    def execute_next_task(self):
        """執行下一個任務"""
        if not self.task_queue:
            return
        
        self.is_executing = True
        self.current_task = self.task_queue.pop(0)
        
        mode = CollaborationMode(self.current_task['mode'])
        self.get_logger().info(f"🚀 執行任務: {self.current_task['task_type']} ({mode.value})")
        
        # 根據模式執行任務
        if mode == CollaborationMode.SEQUENTIAL:
            self.execute_sequential_task()
        elif mode == CollaborationMode.PARALLEL:
            self.execute_parallel_task()
        elif mode == CollaborationMode.SYNCHRONIZED:
            self.execute_synchronized_task()
        elif mode == CollaborationMode.COORDINATED:
            self.execute_coordinated_task()

    def execute_sequential_task(self):
        """順序執行任務"""
        def sequential_execution():
            try:
                # 先執行 UR5
                if self.current_task['ur5_target']:
                    self.get_logger().info("🔄 執行 UR5 動作")
                    success = self.move_ur5_to_target(self.current_task['ur5_target'])
                    if not success:
                        self.get_logger().error("❌ UR5 動作失敗")
                        self.task_complete(False)
                        return
                
                # 再執行 ARM0710
                if self.current_task['arm0710_target']:
                    self.get_logger().info("🔄 執行 ARM0710 動作")
                    success = self.move_arm0710_to_target(self.current_task['arm0710_target'])
                    if not success:
                        self.get_logger().error("❌ ARM0710 動作失敗")
                        self.task_complete(False)
                        return
                
                self.task_complete(True)
                
            except Exception as e:
                self.get_logger().error(f"❌ 順序執行錯誤: {e}")
                self.task_complete(False)
        
        # 在新執行緒中執行
        threading.Thread(target=sequential_execution, daemon=True).start()

    def execute_parallel_task(self):
        """平行執行任務"""
        def parallel_execution():
            try:
                ur5_thread = None
                arm0710_thread = None
                
                # 同時啟動兩個執行緒
                if self.current_task['ur5_target']:
                    ur5_thread = threading.Thread(
                        target=lambda: self.move_ur5_to_target(self.current_task['ur5_target']))
                    ur5_thread.start()
                
                if self.current_task['arm0710_target']:
                    arm0710_thread = threading.Thread(
                        target=lambda: self.move_arm0710_to_target(self.current_task['arm0710_target']))
                    arm0710_thread.start()
                
                # 等待所有執行緒完成
                if ur5_thread:
                    ur5_thread.join()
                if arm0710_thread:
                    arm0710_thread.join()
                
                self.task_complete(True)
                
            except Exception as e:
                self.get_logger().error(f"❌ 平行執行錯誤: {e}")
                self.task_complete(False)
        
        threading.Thread(target=parallel_execution, daemon=True).start()

    def execute_synchronized_task(self):
        """同步執行任務"""
        def synchronized_execution():
            try:
                # 計算兩個軌跡的執行時間，確保同時完成
                ur5_duration = 5.0  # 預設時間
                arm0710_duration = 5.0
                
                # 使用較長的時間作為同步時間
                sync_duration = max(ur5_duration, arm0710_duration)
                
                self.get_logger().info(f"🔄 同步執行，預計時間: {sync_duration}秒")
                
                # 同時啟動
                threads = []
                
                if self.current_task['ur5_target']:
                    ur5_thread = threading.Thread(
                        target=lambda: self.move_ur5_to_target_with_duration(
                            self.current_task['ur5_target'], sync_duration))
                    threads.append(ur5_thread)
                    ur5_thread.start()
                
                if self.current_task['arm0710_target']:
                    arm0710_thread = threading.Thread(
                        target=lambda: self.move_arm0710_to_target_with_duration(
                            self.current_task['arm0710_target'], sync_duration))
                    threads.append(arm0710_thread)
                    arm0710_thread.start()
                
                # 等待所有完成
                for thread in threads:
                    thread.join()
                
                self.task_complete(True)
                
            except Exception as e:
                self.get_logger().error(f"❌ 同步執行錯誤: {e}")
                self.task_complete(False)
        
        threading.Thread(target=synchronized_execution, daemon=True).start()

    def execute_coordinated_task(self):
        """協調執行任務 - 需要實時協調的複雜任務"""
        self.get_logger().info("🤝 執行協調任務")
        # TODO: 實現更複雜的協調邏輯
        self.execute_synchronized_task()

    def move_ur5_to_target(self, target) -> bool:
        """移動 UR5 到目標位置"""
        try:
            if isinstance(target, dict):
                if 'joint_positions' in target:
                    # 關節空間目標
                    self.ur5_group.set_joint_value_target(target['joint_positions'])
                elif 'pose' in target:
                    # 笛卡爾空間目標
                    self.ur5_group.set_pose_target(target['pose'])
                else:
                    self.get_logger().error("❌ UR5 目標格式錯誤")
                    return False
            else:
                # 直接是關節位置列表
                self.ur5_group.set_joint_value_target(target)
            
            # 規劃和執行
            plan = self.ur5_group.plan()
            if plan[0]:  # plan[0] 是成功標誌
                success = self.ur5_group.execute(plan[1], wait=True)
                self.get_logger().info("✅ UR5 移動完成")
                return success
            else:
                self.get_logger().error("❌ UR5 路徑規劃失敗")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ UR5 移動錯誤: {e}")
            return False

    def move_arm0710_to_target(self, target) -> bool:
        """移動 ARM0710 到目標位置"""
        try:
            if isinstance(target, dict):
                if 'joint_positions' in target:
                    return self.send_arm0710_joint_trajectory(target['joint_positions'])
                elif 'pose' in target:
                    # 需要逆運動學求解
                    joint_positions = self.arm0710_inverse_kinematics(target['pose'])
                    if joint_positions:
                        return self.send_arm0710_joint_trajectory(joint_positions)
                    else:
                        self.get_logger().error("❌ ARM0710 逆運動學求解失敗")
                        return False
            else:
                # 直接是關節位置列表
                return self.send_arm0710_joint_trajectory(target)
                
        except Exception as e:
            self.get_logger().error(f"❌ ARM0710 移動錯誤: {e}")
            return False

    def send_arm0710_joint_trajectory(self, joint_positions: List[float], duration: float = 5.0) -> bool:
        """發送 ARM0710 關節軌跡"""
        try:
            # 等待 Action Server
            if not self.arm0710_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ ARM0710 Action Server 未響應")
                return False
            
            # 建立軌跡
            goal_msg = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = [
                'arm0710_joint1', 'arm0710_joint2', 'arm0710_joint3',
                'arm0710_joint4', 'arm0710_joint5', 'arm0710_joint6'
            ]
            
            # 軌跡點
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            
            trajectory.points = [point]
            goal_msg.trajectory = trajectory
            
            # 發送目標
            self.get_logger().info(f"🚀 發送 ARM0710 軌跡: {joint_positions}")
            future = self.arm0710_action_client.send_goal_async(goal_msg)
            
            # 等待結果 (簡化版)
            time.sleep(duration + 1.0)
            
            self.get_logger().info("✅ ARM0710 移動完成")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ ARM0710 軌跡發送錯誤: {e}")
            return False

    def move_ur5_to_target_with_duration(self, target, duration: float) -> bool:
        """以指定時間移動 UR5"""
        # 設定時間參數
        self.ur5_group.set_planning_time(min(duration * 0.8, 10.0))
        return self.move_ur5_to_target(target)

    def move_arm0710_to_target_with_duration(self, target, duration: float) -> bool:
        """以指定時間移動 ARM0710"""
        if isinstance(target, dict) and 'joint_positions' in target:
            return self.send_arm0710_joint_trajectory(target['joint_positions'], duration)
        else:
            return self.send_arm0710_joint_trajectory(target, duration)

    def arm0710_inverse_kinematics(self, pose: Pose) -> Optional[List[float]]:
        """ARM0710 逆運動學求解 (簡化版)"""
        # TODO: 實現真正的逆運動學
        # 這裡返回一個示例解
        self.get_logger().warn("⚠️ 使用簡化逆運動學解")
        return [0.0, 0.5, -0.5, 0.0, -1.0, 0.0]

    def task_complete(self, success: bool):
        """任務完成處理"""
        if self.current_task:
            status = "成功" if success else "失敗"
            self.get_logger().info(f"✅ 任務完成: {self.current_task['task_type']} - {status}")
            
            # 發布任務狀態
            status_msg = Int32MultiArray()
            status_msg.data = [self.current_task['id'], 1 if success else 0]
            self.task_status_pub.publish(status_msg)
        
        self.current_task = None
        self.is_executing = False

    def get_system_status(self) -> Dict:
        """獲取系統狀態"""
        return {
            'safety_status': self.safety_status.value,
            'is_executing': self.is_executing,
            'task_queue_length': len(self.task_queue),
            'current_task': self.current_task['task_type'] if self.current_task else None,
            'ur5_connected': self.ur5_current_pose is not None,
            'arm0710_connected': self.arm0710_current_pose is not None,
        }

    def emergency_stop_all(self):
        """緊急停止所有動作"""
        self.get_logger().error("🚨 執行緊急停止！")
        self.emergency_stop()

def main(args=None):
    rclpy.init(args=args)
    coordinator = DualArmCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info("🔚 雙臂協作控制器關閉")
    finally:
        # 清理 MoveIt
        try:
            moveit_commander.roscpp_shutdown()
        except:
            pass
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()