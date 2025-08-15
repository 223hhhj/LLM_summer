#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import socket
import time
import re

class IntegratedArmGripperController(Node):
    def __init__(self):
        super().__init__('integrated_arm_gripper_controller')
        
        # Publishers
        self.joint_angles_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.end_effector_publisher = self.create_publisher(JointState, '/robot_end_effector', 10)
        self.gripper_publisher = self.create_publisher(Int32MultiArray, '/robotiq_gripper', 10)
        
        # Socket 設定
        self.host, self.port, self.target_ip = '192.168.1.10', 4000, '192.168.1.20'
        self.connect_to_robot()
        
        # Timer 設定
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 關節和末端執行器名稱
        self.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.end_effector_names = [f'end_effector_{axis}' for axis in ['x_mm', 'y_mm', 'z_mm', 'rx_deg', 'ry_deg', 'rz_deg']]
        
        # 狀態變數
        self.latest_joint_angles = None
        self.latest_position_data = None
        self.request_toggle = True
        
        self.is_cartesian_monitoring = False
        self.target_position = None  # 用於儲存笛卡爾目標位置
        # 序列控制
        self.action_sequence = [
            ["speed_config", [5, 40], 1.0],
            ["move", [0, 0, 0, 0, 0, 0], 10.0],
            # ["move_cartesian", [0, 400, 300, 170,-80 ,100 ], 10.0],  # MOVL 到末端位置
            ["gripper", [10, 10, 10, 250, 200, 3, 1], 10.0],
            ["move", [18, 20, -30, 0, -75, 25], 10.0],
            # ["move_cartesian", [50, 400, 300, 170,-80 ,100 ], 10.0],
            ["gripper", [80, 80, 80, 250, 200, 1, 1], 10.0],
            ["move", [-25, -20, 16, 0, -85, 25], 10.0],
            # ["move_cartesian", [80, 400, 300, 170,-80 ,100 ], 10.0],
            ["gripper", [30, 30, 30, 250, 200, 1, 1], 10.0],
        ]
        
        self.current_action_index = 0
        self.action_in_progress = False
        self.sequence_completed = False
        self.target_joint_angles = None
        self.position_tolerance = 2.0
        self.position_check_timer = None
        self.max_wait_time = 10.0
        self.wait_start_time = None
        
        self.start_action_sequence()

    def connect_to_robot(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((self.host, self.port))
        self.get_logger().info('🔗 已連線到機械手臂控制器')

    def clear_socket_buffer(self):
        """清空 socket 緩衝區"""
        try:
            self.sock.setblocking(False)
            while True:
                try:
                    old_data = self.sock.recv(1024)
                    if not old_data:
                        break
                except socket.error:
                    break
        except:
            pass
        finally:
            self.sock.setblocking(True)

    def send_command(self, command):
        """統一的指令發送函數"""
        try:
            self.sock.sendall(command.encode('utf-8'))
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 指令發送失敗: {e}')
            return False

    def request_data(self, command, data_type="angle"):
        self.clear_socket_buffer()
        
        if not self.send_command(command):
            return False
        
        try:
            data = self.sock.recv(1024).decode('utf-8').strip()
            
            if data in ['IRA', ''] or 'error' in data.lower():
                return False
            
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) != 6:
                return False
            
            values = [float(x) for x in matches]
            
            if data_type == "angle":
                if all(-360 <= angle <= 360 for angle in values):
                    self.latest_joint_angles = values
                    self.publish_joint_angles()
                    return True
            else:  # position
                # 驗證笛卡爾位置範圍（範例：±1000 mm, ±180°）
                if all(-1000 <= values[i] <= 1000 for i in range(3)) and all(-180 <= values[i] <= 180 for i in range(3, 6)):
                    self.latest_position_data = values
                    self.publish_end_effector_data()
                    return True
            
            return False
        except Exception as e:
            self.get_logger().error(f"❌ 資料解析失敗: {e}")
            return False

    def timer_callback(self):
        """定時回調函數"""
        if self.request_toggle:
            self.request_data(f'NETS_GETPOS {self.target_ip}\0', "angle")
        else:
            self.request_data(f'NETS_GETDEG {self.target_ip}\0', "position")
        
        self.request_toggle = not self.request_toggle
        time.sleep(0.01)

    def publish_joint_angles(self):
        """發布關節角度"""
        if self.latest_joint_angles is None:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        msg.position = self.latest_joint_angles
        self.joint_angles_publisher.publish(msg)

    def publish_end_effector_data(self):
        """發布末端執行器資料"""
        if self.latest_position_data is None:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.end_effector_names
        msg.position = self.latest_position_data
        self.end_effector_publisher.publish(msg)

    # ==================== 序列控制系統 ====================
    
    def start_action_sequence(self):
        """啟動序列執行"""
        if self.current_action_index < len(self.action_sequence):
            self.execute_next_action()
        else:
            self.sequence_completed = True
            self.get_logger().info("🎉 所有動作序列執行完成！")

    def execute_next_action(self):
        if self.current_action_index >= len(self.action_sequence):
            self.sequence_completed = True
            return
        
        action_type, params, delay = self.action_sequence[self.current_action_index]
        self.action_in_progress = True
        
        self.get_logger().info(f"🚀 執行動作 {self.current_action_index + 1}/{len(self.action_sequence)}: {action_type}")
        
        action_handlers = {
            "move": self.handle_move_action,
            "move_cartesian": self.handle_move_cartesian_action,  # 新增 MOVL 處理
            "gripper": self.handle_gripper_action,
            "speed_config": self.handle_speed_action
        }
        
        handler = action_handlers.get(action_type)
        if handler:
            handler(params, delay)
        else:
            self.get_logger().error(f"❌ 未知動作類型: {action_type}")
            self.action_complete_callback()

    def handle_move_cartesian_action(self, params, delay):
        """處理笛卡爾空間移動動作 (MOVL)"""
        if len(params) != 6:
            self.get_logger().error(f"❌ MOVL 參數數量錯誤: 應為6個，實際為{len(params)}")
            self.action_complete_callback()
            return
        
        cmd = f'MOVL {" ".join(map(str, params))}\0'
        if self.send_command(cmd):
            self.get_logger().info(f'🦾 發送末端移動指令: {cmd.strip()}')
            self.target_position = params.copy()  # 儲存目標笛卡爾位置
            self.wait_start_time = time.time()
            self.start_position_monitoring(cartesian=True)  # 啟動笛卡爾位置監控
        else:
            self.get_logger().error("❌ 發送 MOVL 指令失敗")
            self.action_complete_callback()

    def handle_move_action(self, params, delay):
        """處理移動動作"""
        cmd = f'MOVJ {" ".join(map(str, params))}\0'
        if self.send_command(cmd):
            self.get_logger().info(f'🦾 發送手臂指令: {cmd.strip()}')
            self.target_joint_angles = params.copy()
            self.wait_start_time = time.time()
            self.start_position_monitoring()

    def handle_gripper_action(self, params, delay):
        """處理夾爪動作"""
        self.get_logger().warning("🚨 === 準備發送夾爪指令 ===")
        if self.latest_joint_angles:
            self.get_logger().warning(f"🚨 當前關節角度: {[f'{x:.1f}' for x in self.latest_joint_angles]}")
        
        gripper_msg = Int32MultiArray()
        gripper_msg.data = params
        self.gripper_publisher.publish(gripper_msg)
        
        self.get_logger().warning(f'🤖 === 夾爪指令已發送 ===: {gripper_msg.data}')
        
        # 顯示夾爪詳細資訊
        if len(params) >= 7:
            pos, speed, force, mode_id = params[0:3], params[3], params[4], params[5]
            modes = ["Basic", "Pinch", "Wide", "Scissor"]
            mode_name = modes[mode_id] if 0 <= mode_id < len(modes) else "Unknown"
            self.get_logger().warning(f'📌 夾爪設定: 位置={pos}, 速度={speed}, 力量={force}, 模式={mode_name}')
        
        self.action_timer = self.create_timer(delay, self.action_complete_callback)

    def handle_speed_action(self, params, delay):
        """處理速度設定動作"""
        ptp_speed, line_speed = params
        
        commands = [
            (f'SETPTPSPEED {ptp_speed}\0', f'⚡ PTP速度設定: {ptp_speed}%'),
            (f'SETLINESPEED {line_speed}\0', f'⚡ 直線速度設定: {line_speed}mm/s')
        ]
        
        for cmd, log_msg in commands:
            if self.send_command(cmd):
                self.get_logger().info(log_msg)
            time.sleep(0.1)
        
        self.get_logger().info(f'📊 速度配置完成 - PTP: {ptp_speed}%, 直線: {line_speed}mm/s')
        self.action_timer = self.create_timer(delay, self.action_complete_callback)

    def start_position_monitoring(self, cartesian=False):
        """開始位置監控，支援關節角度或笛卡爾位置"""
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
        
        self.is_cartesian_monitoring = cartesian  # 記錄監控類型
        self.position_check_timer = self.create_timer(0.2, self.check_position_callback)
        self.get_logger().info(f"🎯 開始監控{'笛卡爾位置' if cartesian else '關節角度'}: 目標={self.target_position if cartesian else self.target_joint_angles}")

    def check_position_callback(self):
        """檢查位置到達，支援關節角度和笛卡爾位置"""
        if self.is_cartesian_monitoring:
            if self.latest_position_data is None:
                return
            target = self.target_position
            current = self.latest_position_data
            tolerance = 2.0  # 笛卡爾位置容差 (mm 或 度)
            unit = "mm/deg"
        else:
            if self.latest_joint_angles is None:
                return
            target = self.target_joint_angles
            current = self.latest_joint_angles
            tolerance = self.position_tolerance  # 關節角度容差 (2.0°)
            unit = "deg"
        
        elapsed_time = time.time() - self.wait_start_time
        if elapsed_time > self.max_wait_time:
            self.get_logger().warning(f"⏰ 位置等待超時！")
            self.position_reached()
            return
        
        differences = [abs(current_val - target_val) for current_val, target_val in zip(current, target)]
        max_diff = max(differences)
        
        self.get_logger().info(f"🔍 位置檢查 - 時間:{elapsed_time:.1f}s, 最大誤差:{max_diff:.2f}{unit}")
        
        if max_diff <= tolerance:
            self.get_logger().info(f"✅ {'末端' if self.is_cartesian_monitoring else '手臂'}已到達目標位置！總耗時:{elapsed_time:.1f}s")
            self.position_reached()

    def position_reached(self):
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
            self.position_check_timer = None
        
        self.is_cartesian_monitoring = False
        self.action_timer = self.create_timer(0.5, self.action_complete_callback)
        self.get_logger().info("🛑 手臂已穩定，準備執行下一動作")

    def action_complete_callback(self):
        """動作完成回調"""
        if hasattr(self, 'action_timer'):
            self.destroy_timer(self.action_timer)
        
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
            self.position_check_timer = None
        
        self.action_in_progress = False
        self.current_action_index += 1
        
        self.get_logger().info(f"✅ 動作 {self.current_action_index}/{len(self.action_sequence)} 完成")
        self.start_action_sequence()

    def add_action_to_sequence(self, action_type, params, delay=2.0):
        if action_type not in ["move", "move_cartesian", "gripper", "speed_config"]:
            self.get_logger().error(f"❌ 無效動作類型: {action_type}")
            return
        if not self.sequence_completed:
            self.action_sequence.append([action_type, params, delay])
            self.get_logger().info(f"➕ 新增動作: {action_type} - {params}")

    def get_current_status(self):
        """獲取當前狀態"""
        if self.sequence_completed:
            return "序列執行完成"
        elif self.action_in_progress:
            action_type, params, _ = self.action_sequence[self.current_action_index]
            return f"執行中: {action_type} - {params}"
        else:
            return f"準備執行動作 {self.current_action_index + 1}/{len(self.action_sequence)}"

    def __del__(self):
        """清理資源"""
        try:
            if hasattr(self, 'position_check_timer') and self.position_check_timer is not None:
                self.destroy_timer(self.position_check_timer)
            if hasattr(self, 'action_timer'):
                self.destroy_timer(self.action_timer)
            if hasattr(self, 'sock'):
                self.sock.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedArmGripperController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
































































'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

import socket
import time
import re
import math

class IntegratedArmGripperController(Node):
    def __init__(self):
        super().__init__('integrated_arm_gripper_controller')

        # 宣告兩個獨立的 JointState publisher
        self.joint_angles_publisher = self.create_publisher(JointState, '/robot_joint_angles', 10)
        self.end_effector_publisher = self.create_publisher(JointState, '/robot_end_effector', 10)
        
        # 新增夾爪控制 publisher
        self.gripper_publisher = self.create_publisher(Int32MultiArray, '/robotiq_gripper', 10)

        # 設定 socket
        self.host = '192.168.1.10'
        self.port = 4000
        self.target_ip = '192.168.1.20'

        self.connect_to_robot()

        # 建立 timer，降低頻率避免過於頻繁的請求
        self.timer = self.create_timer(0.05, self.timer_callback)  # 改為 20Hz

        # 關節名稱（只包含 6 個關節）
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # 末端執行器位置和姿態名稱
        self.end_effector_names = [
            'end_effector_x_mm', 'end_effector_y_mm', 'end_effector_z_mm',
            'end_effector_rx_deg', 'end_effector_ry_deg', 'end_effector_rz_deg'
        ]

        # 用於儲存最新的角度和位置數據
        self.latest_joint_angles = None
        self.latest_position_data = None
        self.request_toggle = True  # True=請求角度, False=請求位置
        
        # ==================== 序列控制系統 ====================
        # 定義動作序列：每個元素包含 [動作類型, 參數, 延遲時間(秒)]
        self.action_sequence = [
            ["speed_config", [5, 20], 1.0],              # 設定速度: PTP=3%, Line=20mm/s, 等待1秒
            ["move", [0, 0, 0, 0, 0, 0], 10.0],           # 移動到位置1, 等待3秒
            ["gripper", [10, 10, 10, 250, 200, 3, 1], 10.0],  # 夾爪打開, 等待2秒
            ["move", [0, 0, 0, 0, 0, 10], 10.0],         # 移動到位置2, 等待3秒
            ["gripper", [80, 80, 80, 250, 200, 1, 1], 10.0],  # 夾爪夾取, 等待2秒
            ["move", [0, 0, 0, 0, 0, -10], 10.0],           # 移動回位置1, 等待3秒
            ["gripper", [200, 200, 200, 200, 150, 0, 1], 10.0],  # 夾爪放開, 等待2秒
        ]
        
        self.current_action_index = 0
        self.action_in_progress = False
        self.sequence_completed = False
        
        # 位置到達確認相關變數
        self.target_joint_angles = None
        self.position_tolerance = 2.0  # 角度誤差容忍度（度）
        self.position_check_timer = None
        self.max_wait_time = 10.0  # 最大等待時間（秒）
        self.wait_start_time = None
        
        # 啟動序列控制
        self.start_action_sequence()

    def start_action_sequence(self):
        """啟動動作序列執行"""
        if self.current_action_index < len(self.action_sequence):
            self.execute_next_action()
        else:
            self.sequence_completed = True
            self.get_logger().info("🎉 所有動作序列執行完成！")

    def execute_next_action(self):
        """執行下一個動作"""
        if self.current_action_index >= len(self.action_sequence):
            self.sequence_completed = True
            return
            
        action_type, params, delay = self.action_sequence[self.current_action_index]
        self.action_in_progress = True
        
        self.get_logger().info(f"🚀 執行動作 {self.current_action_index + 1}/{len(self.action_sequence)}: {action_type}")
        
        if action_type == "move":
            self.send_arm_command(params)
            # 設定目標位置並開始位置確認
            self.target_joint_angles = params.copy()
            self.wait_start_time = time.time()
            self.start_position_monitoring()
        elif action_type == "gripper":
            self.send_gripper_command(params)
            # 夾爪動作使用固定延遲
            self.action_timer = self.create_timer(delay, self.action_complete_callback)
        elif action_type == "speed_config":
            self.send_speed_config(params)
            # 速度設定使用固定延遲
            self.action_timer = self.create_timer(delay, self.action_complete_callback)

    def start_position_monitoring(self):
        """開始監控手臂位置是否到達目標"""
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
        
        # 每0.2秒檢查一次位置
        self.position_check_timer = self.create_timer(0.2, self.check_position_callback)
        self.get_logger().info(f"🎯 開始監控位置到達: 目標={self.target_joint_angles}")

    def check_position_callback(self):
        """檢查手臂是否到達目標位置"""
        if self.latest_joint_angles is None:
            self.get_logger().debug("⚠️ 尚未收到關節角度資料")
            return
            
        # 檢查是否超時
        elapsed_time = time.time() - self.wait_start_time
        if elapsed_time > self.max_wait_time:
            self.get_logger().warning(f"⏰ 位置等待超時！當前位置: {[f'{x:.2f}' for x in self.latest_joint_angles]}")
            self.get_logger().warning(f"⏰ 目標位置: {[f'{x:.2f}' for x in self.target_joint_angles]}")
            self.position_reached()
            return
        
        # 檢查每個關節是否都在容忍範圍內
        position_reached = True
        differences = []
        
        for i, (current, target) in enumerate(zip(self.latest_joint_angles, self.target_joint_angles)):
            diff = abs(current - target)
            differences.append(diff)
            if diff > self.position_tolerance:
                position_reached = False
        
        # 每次都記錄詳細信息用於調試
        max_diff = max(differences)
        self.get_logger().info(f"🔍 位置檢查 - 時間:{elapsed_time:.1f}s, 最大誤差:{max_diff:.2f}°, 所有誤差:{[f'{x:.1f}' for x in differences]}")
        self.get_logger().info(f"📍 當前位置: {[f'{x:.1f}' for x in self.latest_joint_angles]}")
        self.get_logger().info(f"🎯 目標位置: {[f'{x:.1f}' for x in self.target_joint_angles]}")
        
        if position_reached:
            self.get_logger().info(f"✅ 手臂已到達目標位置！總耗時:{elapsed_time:.1f}s, 最大誤差:{max_diff:.2f}°")
            self.position_reached()
        else:
            self.get_logger().info(f"🔄 繼續等待到達位置... 剩餘最大誤差:{max_diff:.2f}°")

    def position_reached(self):
        """位置到達處理"""
        # 停止位置監控
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
            self.position_check_timer = None
        
        # 位置到達後，等待0.5秒讓手臂穩定
        self.action_timer = self.create_timer(0.5, self.action_complete_callback)
        self.get_logger().info("🛑 手臂已穩定，準備執行下一動作")

    def action_complete_callback(self):
        """動作完成回調"""
        # 銷毀當前的動作定時器
        if hasattr(self, 'action_timer'):
            self.destroy_timer(self.action_timer)
        
        # 清理位置監控定時器（如果存在）
        if self.position_check_timer is not None:
            self.destroy_timer(self.position_check_timer)
            self.position_check_timer = None
        
        self.action_in_progress = False
        self.current_action_index += 1
        
        self.get_logger().info(f"✅ 動作 {self.current_action_index}/{len(self.action_sequence)} 完成")
        
        # 執行下一個動作
        self.start_action_sequence()

    def send_arm_command(self, joint_angles):
        """發送手臂移動指令"""
        cmd = f'MOVJ {joint_angles[0]} {joint_angles[1]} {joint_angles[2]} {joint_angles[3]} {joint_angles[4]} {joint_angles[5]}\0'
        try:
            self.sock.sendall(cmd.encode('utf-8'))
            self.get_logger().info(f'🦾 發送手臂指令: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ 發送手臂指令失敗: {e}')

    def send_speed_config(self, speed_params):
        """發送速度設定指令"""
        ptp_speed, line_speed = speed_params
        
        # 發送PTP速度設定
        ptp_cmd = f'SETPTPSPEED {ptp_speed}\0'
        try:
            self.sock.sendall(ptp_cmd.encode('utf-8'))
            self.get_logger().info(f'⚡ 發送PTP速度設定: {ptp_cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ 發送PTP速度設定失敗: {e}')
        
        time.sleep(0.1)  # 小延遲確保指令處理
        
        # 發送直線速度設定
        line_cmd = f'SETLINESPEED {line_speed}\0'
        try:
            self.sock.sendall(line_cmd.encode('utf-8'))
            self.get_logger().info(f'⚡ 發送直線速度設定: {line_cmd.strip()}')
            self.get_logger().info(f'📊 速度配置完成 - PTP速度: {ptp_speed}%, 直線速度: {line_speed}mm/s')
        except Exception as e:
            self.get_logger().error(f'❌ 發送直線速度設定失敗: {e}')

    def send_gripper_command(self, gripper_data):
        """發送夾爪控制指令"""
        self.get_logger().warning("🚨 === 準備發送夾爪指令 ===")
        self.get_logger().warning(f"🚨 當前手臂狀態: 動作進行中={self.action_in_progress}")
        if self.latest_joint_angles:
            self.get_logger().warning(f"🚨 當前關節角度: {[f'{x:.1f}' for x in self.latest_joint_angles]}")
        
        gripper_msg = Int32MultiArray()
        gripper_msg.data = gripper_data
        
        self.gripper_publisher.publish(gripper_msg)
        self.get_logger().warning(f'🤖 === 夾爪指令已發送 ===: {gripper_msg.data}')
        
        # 解析夾爪指令並顯示詳細資訊
        if len(gripper_data) >= 7:
            pos = gripper_data[0:3]
            speed = gripper_data[3]
            force = gripper_data[4]
            mode_id = gripper_data[5]
            individual = gripper_data[6]
            
            modes = ["Basic", "Pinch", "Wide", "Scissor"]
            mode_name = modes[mode_id] if 0 <= mode_id < len(modes) else "Unknown"
            
            self.get_logger().warning(f'📌 夾爪設定: 位置={pos}, 速度={speed}, 力量={force}, 模式={mode_name}, 獨立控制={"開啟" if individual else "關閉"}')

    def add_action_to_sequence(self, action_type, params, delay=2.0):
        """動態新增動作到序列中（必須在序列執行前調用）"""
        if not self.sequence_completed:
            self.action_sequence.append([action_type, params, delay])
            self.get_logger().info(f"➕ 新增動作到序列: {action_type} - {params}")

    def clear_socket_buffer(self):
        """清空 socket 接收緩衝區中的舊資料"""
        try:
            # 設定非阻塞模式
            self.sock.setblocking(False)
            while True:
                try:
                    old_data = self.sock.recv(1024)
                    if not old_data:
                        break
                    self.get_logger().debug(f'清除舊資料: {old_data.decode("utf-8", errors="ignore").strip()}')
                except socket.error:
                    break
        except Exception:
            pass
        finally:
            # 恢復阻塞模式
            self.sock.setblocking(True)

    def connect_to_robot(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((self.host, self.port))
        self.get_logger().info('🔗 已連線到機械手臂控制器')

    def request_joint_angles(self):
        """請求關節角度 - NETS_GETPOS 回傳角度資料"""
        # 清空接收緩衝區，避免讀取到舊資料
        self.clear_socket_buffer()
        
        cmd = f'NETS_GETPOS {self.target_ip}\0'
        self.sock.sendall(cmd.encode('utf-8'))
        
        try:
            data = self.sock.recv(1024).decode('utf-8').strip()
            
            # 檢查是否為錯誤回應
            if data == 'IRA' or data == '' or 'error' in data.lower():
                return False
            
            # 抓取 6 個浮點數（支援科學記號）
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                angles_deg = [float(x) for x in matches]
                
                # 驗證是否為合理的角度範圍（通常 -360 到 360 度）
                if all(-360 <= angle <= 360 for angle in angles_deg):
                    self.latest_joint_angles = angles_deg
                    
                    # 立即發布關節角度資料
                    self.publish_joint_angles()
                    return True
                else:
                    return False
            else:
                return False
        except socket.timeout:
            return False
        except Exception as e:
            return False

    def request_position_data(self):
        """請求末端執行器位置 - NETS_GETDEG 回傳位置資料"""
        # 清空接收緩衝區，避免讀取到舊資料
        self.clear_socket_buffer()
        
        cmd = f'NETS_GETDEG {self.target_ip}\0'
        self.sock.sendall(cmd.encode('utf-8'))

        try:
            data = self.sock.recv(1024).decode('utf-8').strip()

            # 檢查是否為錯誤回應
            if data == 'IRA' or data == '' or 'error' in data.lower():
                return False

            # 抓取6個浮點數，檢查數量是否等於6
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                position_data = [float(x) for x in matches]
                
                # 驗證是否為合理的位置範圍（位置通常有較大的數值範圍）
                # 檢查是否至少有一個數值的絕對值大於 10（位置特徵）
                if any(abs(val) > 10 for val in position_data):
                    self.latest_position_data = position_data
                    
                    # 立即發布末端執行器資料
                    self.publish_end_effector_data()
                    return True
                else:
                    return False
            else:
                return False
        except socket.timeout:
            return False
        except Exception as e:
            return False

    def timer_callback(self):
        """定時回調函數，輪流請求角度和位置，加入延遲避免衝突"""
        
        if self.request_toggle:
            # 請求關節角度
            success = self.request_joint_angles()
            self.request_toggle = False  # 下次請求位置
        else:
            # 請求位置資料
            success = self.request_position_data()
            self.request_toggle = True   # 下次請求角度
        
        # 加入小延遲，讓控制器有時間處理
        time.sleep(0.01)

    def publish_joint_angles(self):
        """發布關節角度到獨立的 topic"""
        if self.latest_joint_angles is None:
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        msg.position = self.latest_joint_angles
        
        self.joint_angles_publisher.publish(msg)

    def publish_end_effector_data(self):
        """發布末端執行器資料到獨立的 topic"""
        if self.latest_position_data is None:
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.end_effector_names
        msg.position = self.latest_position_data
        
        self.end_effector_publisher.publish(msg)

    def get_current_status(self):
        """獲取當前系統狀態"""
        if self.sequence_completed:
            return "序列執行完成"
        elif self.action_in_progress:
            action_type, params, _ = self.action_sequence[self.current_action_index]
            return f"執行中: {action_type} - {params}"
        else:
            return f"準備執行動作 {self.current_action_index + 1}/{len(self.action_sequence)}"

    def __del__(self):
        """確保關閉socket連接"""
        try:
            # 清理所有定時器
            if hasattr(self, 'position_check_timer') and self.position_check_timer is not None:
                self.destroy_timer(self.position_check_timer)
            if hasattr(self, 'action_timer'):
                self.destroy_timer(self.action_timer)
            if hasattr(self, 'sock'):
                self.sock.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedArmGripperController()
    
    # 可以在這裡動態新增更多動作（在 rclpy.spin 之前）
    # node.add_action_to_sequence("move", [10, 20, 30, 0, 0, 0], 3.0)
    # node.add_action_to_sequence("gripper", [100, 100, 100, 200, 150, 0, 0], 2.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

import socket
import time
import re
import math


class JointAngleReader(Node):
    def __init__(self):
        super().__init__('joint_angle_reader')

        # 宣告 JointState publisher (包含關節角度和末端執行器位置)
        self.publisher_ = self.create_publisher(JointState, '/robot_states', 10)

        # 宣告夾爪控制 publisher
        self.gripper_publisher_ = self.create_publisher(Int32MultiArray, 'robotiq_gripper', 10)

        # 設定 socket
        self.host = '192.168.1.10'
        self.port = 4000
        self.target_ip = '192.168.1.20'

        self.connect_to_robot()
        
        # 順序移動控制
        self.movement_sequence = [
            'MOVJ 0 0 0 0 0 0',      # 第一個位置（歸零位置）
            'MOVJ 0 0 0 0 0 30',     # 第二個位置
            'MOVL 0.8 430 350 # # #' # 第三個位置
        ]
        
        # 對應的夾爪命令序列
        self.gripper_commands = [
            [100, 100, 100, 250, 200, 2, 1],  # MOVJ 0 0 0 0 0 0 後：剪刀模式，獨立控制
            [10, 10, 10, 250, 200, 3, 1],  # MOVJ 0 0 0 0 0 30 後：寬模式，獨立控制
            [80, 80, 80, 250, 200, 1, 1],  # MOVL 0.8 430 350 # # # 後：夾緊模式，獨立控制
        ]
        
        self.current_move_index = 0
        self.is_moving = True
        self.move_start_time = None
        
        # 開始執行移動序列
        self.execute_next_movement()

        # 關節名稱 + 末端執行器位置和姿態
        self.joint_names = [
            # 6個關節 (角度以度數為單位)
            'joint_1_deg', 'joint_2_deg', 'joint_3_deg',
            'joint_4_deg', 'joint_5_deg', 'joint_6_deg',
            # 末端執行器位置和姿態 (以 mm 和 degree 為單位的原始數據)
            'end_effector_x_mm', 'end_effector_y_mm', 'end_effector_z_mm',
            'end_effector_rx_deg', 'end_effector_ry_deg', 'end_effector_rz_deg'
        ]

        # 用於儲存最新的角度和位置數據
        self.latest_joint_angles = None
        self.latest_position_data = None
        
        # 建立 timer，每秒觸發一次 - 從開始就運行監控
        self.timer = self.create_timer(1.0, self.timer_callback)

    def connect_to_robot(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((self.host, self.port))
        self.get_logger().info('已連線到機械手臂控制器')

    def execute_next_movement(self):
        """執行下一個移動指令"""
        if self.current_move_index < len(self.movement_sequence):
            cmd = self.movement_sequence[self.current_move_index] + '\0'
            self.sock.sendall(cmd.encode('utf-8'))
            self.get_logger().info(f'🤖 執行移動 {self.current_move_index + 1}: {cmd.strip()}')
            
            self.is_moving = True
            self.move_start_time = time.time()
            self.current_move_index += 1
        else:
            self.get_logger().info('✅ 所有移動序列完成，開始狀態監控')
            self.is_moving = False

    def publish_gripper_command(self, command):
        """發布夾爪控制命令"""
        msg = Int32MultiArray()
        msg.data = command
        self.gripper_publisher_.publish(msg)
        self.get_logger().info(f'🦾 發布夾爪命令: {command}')

    def check_movement_completion(self):
        """檢查移動是否完成並觸發夾爪命令"""
        if self.is_moving and self.move_start_time:
            # 假設每個移動需要 3 秒完成（可根據實際情況調整）
            if time.time() - self.move_start_time > 10.0:
                self.get_logger().info(f'✅ 移動 {self.current_move_index} 完成')
                
                # 發布對應的夾爪命令（如果存在）
                gripper_cmd_index = self.current_move_index - 1
                if gripper_cmd_index < len(self.gripper_commands):
                    self.publish_gripper_command(self.gripper_commands[gripper_cmd_index])
                
                # 如果還有下一個移動，繼續執行
                if self.current_move_index < len(self.movement_sequence):
                    time.sleep(10)  # 稍微等待確保穩定
                    self.execute_next_movement()
                else:
                    self.get_logger().info('🎯 所有移動完成，開始正常狀態監控')
                    self.is_moving = False

    def request_both_data_sync(self):
        """同步請求關節角度和位置資料"""
        angles_success = False
        position_success = False
        
        # 同時請求角度資料
        try:
            cmd = f'NETS_GETPOS {self.target_ip}\0'
            self.sock.sendall(cmd.encode('utf-8'))
            
            data = self.sock.recv(1024).decode('utf-8')
            # self.get_logger().info(f'角度回應: {data.strip()}')
            
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                angles_deg = [float(x) for x in matches]
                self.latest_joint_angles = angles_deg
                # self.get_logger().info(f'✅ 關節角度 (度): {angles_deg}')
                angles_success = True
            else:
                self.get_logger().warn('⚠️ 角度回應格式錯誤')
        except Exception as e:
            self.get_logger().warn(f'⚠️ 獲取角度失敗: {e}')
        
        # 稍微等待後請求位置資料
        time.sleep(0.1)
        
        try:
            cmd = f'NETS_GETDEG {self.target_ip}\0'
            self.sock.sendall(cmd.encode('utf-8'))
            
            data = self.sock.recv(1024).decode('utf-8')
            # self.get_logger().info(f'位置回應: {data.strip()}')
            
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                position_data = [float(x) for x in matches]
                self.latest_position_data = position_data
                # self.get_logger().info(f'✅ 位置資料: X={position_data[0]:.2f}, Y={position_data[1]:.2f}, Z={position_data[2]:.2f}')
                position_success = True
            else:
                self.get_logger().warn('⚠️ 位置回應格式錯誤')
        except Exception as e:
            self.get_logger().warn(f'⚠️ 獲取位置失敗: {e}')
        
        return angles_success, position_success

    def timer_callback(self):
        """定時回調函數，處理移動序列和持續監控"""
        
        # 檢查移動是否完成（如果正在移動中）
        if self.is_moving:
            self.check_movement_completion()
        
        # 無論是否在移動，都持續監控狀態
        angles_success, position_success = self.request_both_data_sync()
        
        # 根據獲取的資料發布對應的訊息
        if angles_success and position_success:
            self.publish_combined_data()
        elif angles_success:
            self.publish_joint_only()
        elif position_success:
            self.publish_position_only()
        else:
            self.get_logger().warn('⚠️ 本次監控週期未獲取到任何有效資料')

    def publish_combined_data(self):
        """發布完整的關節角度和位置資料"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        
        # 合併關節角度 (度數) 和位置資料 (原始單位)
        combined_data = self.latest_joint_angles + self.latest_position_data
        msg.position = combined_data
        
        self.publisher_.publish(msg)
        
        # 詳細日誌
        # self.get_logger().info('📊 發布完整機械手臂狀態:')
        # self.get_logger().info(f'   關節角度 (度): {[f"{x:.4f}" for x in self.latest_joint_angles]}')
        # self.get_logger().info(f'   位置 (mm): X={self.latest_position_data[0]:.2f}, Y={self.latest_position_data[1]:.2f}, Z={self.latest_position_data[2]:.2f}')
        # self.get_logger().info(f'   姿態 (度): RX={self.latest_position_data[3]:.2f}, RY={self.latest_position_data[4]:.2f}, RZ={self.latest_position_data[5]:.2f}')

    def publish_joint_only(self):
        """只發布關節角度"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names[:6]  # 只包含關節名稱
        msg.position = self.latest_joint_angles
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'📐 發布關節角度: {[f"{x:.4f}" for x in self.latest_joint_angles]} (度)')

    def publish_position_only(self):
        """只發布位置資料"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names[6:]  # 只包含位置名稱
        msg.position = self.latest_position_data
        
        self.publisher_.publish(msg)
        # self.get_logger().info(f'📍 發布位置資料: {self.latest_position_data}')


def main(args=None):
    rclpy.init(args=args)
    node = JointAngleReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''