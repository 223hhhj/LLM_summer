#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class ItriArmTestController(Node):
    def __init__(self):
        super().__init__('itri_arm_test_controller')
        
        # 建立發布器
        self.position_publisher = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10
        )
        
        # 建立計時器
        self.timer = self.create_timer(2.0, self.send_test_positions)
        
        # 測試位置序列 (弧度)
        self.test_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                    # 歸零位置
            [0.5, 0.3, -0.2, 0.1, -0.1, 0.2],                  # 測試位置 1
            [-0.3, -0.5, 0.4, -0.2, 0.3, -0.1],                # 測試位置 2
            [0.2, 0.1, 0.1, 0.3, -0.2, 0.4],                   # 測試位置 3
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                    # 回到歸零
        ]
        
        self.current_position_index = 0
        
        self.get_logger().info('ITRI Arm Test Controller started')
        self.get_logger().info('Will cycle through test positions every 2 seconds')

    def send_test_positions(self):
        msg = Float64MultiArray()
        msg.data = self.test_positions[self.current_position_index]
        
        self.position_publisher.publish(msg)
        
        self.get_logger().info(
            f'Sent position {self.current_position_index + 1}/{len(self.test_positions)}: '
            f'{[f"{pos:.3f}" for pos in msg.data]}'
        )
        
        # 移動到下一個位置
        self.current_position_index = (self.current_position_index + 1) % len(self.test_positions)

def main(args=None):
    rclpy.init(args=args)
    
    test_controller = ItriArmTestController()
    
    try:
        rclpy.spin(test_controller)
    except KeyboardInterrupt:
        test_controller.get_logger().info('Test controller stopped by user')
    finally:
        test_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()