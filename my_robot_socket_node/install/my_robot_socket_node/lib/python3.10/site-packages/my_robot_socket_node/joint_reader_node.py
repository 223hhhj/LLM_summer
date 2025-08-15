import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import socket
import time
import re
import math


class JointAngleReader(Node):
    def __init__(self):
        super().__init__('joint_angle_reader')

        # 宣告 JointState publisher (包含關節角度和末端執行器位置)
        self.publisher_ = self.create_publisher(JointState, '/robot_states', 10)

        # 設定 socket
        self.host = '192.168.1.10'
        self.port = 4000
        self.target_ip = '192.168.1.20'

        self.connect_to_robot()
        
        # 順序移動控制
        self.movement_sequence = [
            'MOVJ 0 0 0 0 0 0',      # 第一個位置（原本的歸零位置）
            'MOVJ 0 0 0 0 0 30',
            'MOVL 0.8 430 350 # # #'      # 第二個位置
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

    def check_movement_completion(self):
        """檢查移動是否完成 - 這裡使用簡單的時間等待"""
        if self.is_moving and self.move_start_time:
            # 假設每個移動需要 3 秒完成（你可以根據實際情況調整）
            if time.time() - self.move_start_time > 3.0:
                self.get_logger().info(f'✅ 移動 {self.current_move_index} 完成')
                
                # 如果還有下一個移動，繼續執行
                if self.current_move_index < len(self.movement_sequence):
                    time.sleep(0.5)  # 稍微等待確保穩定
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
            self.get_logger().info(f'角度回應: {data.strip()}')
            
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                angles_deg = [float(x) for x in matches]
                # 保持度數單位，不轉換為弧度
                self.latest_joint_angles = angles_deg
                self.get_logger().info(f'✅ 關節角度 (度): {angles_deg}')
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
            self.get_logger().info(f'位置回應: {data.strip()}')
            
            matches = re.findall(r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?', data)
            if len(matches) == 6:
                position_data = [float(x) for x in matches]
                self.latest_position_data = position_data
                self.get_logger().info(f'✅ 位置資料: X={position_data[0]:.2f}, Y={position_data[1]:.2f}, Z={position_data[2]:.2f}')
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
        self.get_logger().info('📊 發布完整機械手臂狀態:')
        self.get_logger().info(f'   關節角度 (度): {[f"{x:.4f}" for x in self.latest_joint_angles]}')
        self.get_logger().info(f'   位置 (mm): X={self.latest_position_data[0]:.2f}, Y={self.latest_position_data[1]:.2f}, Z={self.latest_position_data[2]:.2f}')
        self.get_logger().info(f'   姿態 (度): RX={self.latest_position_data[3]:.2f}, RY={self.latest_position_data[4]:.2f}, RZ={self.latest_position_data[5]:.2f}')

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
        self.get_logger().info(f'📍 發布位置資料: {self.latest_position_data}')


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