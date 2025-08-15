#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
import time

class GripperControlClient(Node):
    def __init__(self):
        super().__init__('gripper_control_client')
        
        # 發布夾爪指令的Publisher（給你現有的夾爪節點）
        self.gripper_pub = self.create_publisher(
            Int32MultiArray, 
            'custom_gripper', 
            10
        )
        
        # 建立Service Server來接收控制指令
        self.open_service = self.create_service(
            Trigger, 
            'open_gripper', 
            self.open_gripper_callback
        )
        
        self.close_service = self.create_service(
            Trigger, 
            'close_gripper', 
            self.close_gripper_callback
        )
        
        self.get_logger().info('夾爪控制服務已啟動！')
        self.get_logger().info('可用服務：')
        self.get_logger().info('  - /open_gripper  : 張開夾爪')
        self.get_logger().info('  - /close_gripper : 關閉夾爪')

    def open_gripper_callback(self, request, response):
        """張開夾爪的Service回調函數"""
        self.get_logger().info('收到張開夾爪指令')
        
        try:
            # 發送張開夾爪的指令（寬度=110, 力量=40）
            msg = Int32MultiArray()
            msg.data = [110, 40]  # [寬度, 力量]
            
            self.gripper_pub.publish(msg)
            self.get_logger().info('夾爪張開指令已發送')
            
            # 等待一下讓夾爪動作完成
            time.sleep(2)
            
            response.success = True
            response.message = '夾爪已張開'
            
        except Exception as e:
            self.get_logger().error(f'張開夾爪失敗: {str(e)}')
            response.success = False
            response.message = f'張開失敗: {str(e)}'
        
        return response

    def close_gripper_callback(self, request, response):
        """關閉夾爪的Service回調函數"""
        self.get_logger().info('收到關閉夾爪指令')
        
        try:
            # 發送關閉夾爪的指令（寬度=0, 力量=40）
            msg = Int32MultiArray()
            msg.data = [0, 40]  # [寬度, 力量]
            
            self.gripper_pub.publish(msg)
            self.get_logger().info('夾爪關閉指令已發送')
            
            # 等待一下讓夾爪動作完成
            time.sleep(2)
            
            response.success = True
            response.message = '夾爪已關閉'
            
        except Exception as e:
            self.get_logger().error(f'關閉夾爪失敗: {str(e)}')
            response.success = False
            response.message = f'關閉失敗: {str(e)}'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GripperControlClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()