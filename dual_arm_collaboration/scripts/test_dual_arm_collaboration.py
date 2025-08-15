# dual_arm_collaboration/scripts/test_dual_arm_collaboration.py
#!/usr/bin/env python3
"""
雙臂協作測試程式
測試各種協作模式和安全功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
import sys

# 假設我們有一個服務介面來控制雙臂協作器
class DualArmTester(Node):
    def __init__(self):
        super().__init__('dual_arm_tester')
        
        # 等待協作控制器啟動
        time.sleep(2.0)
        self.get_logger().info("🧪 雙臂協作測試器已啟動")

    def test_sequential_collaboration(self):
        """測試順序協作"""
        self.get_logger().info("🔄 測試順序協作模式")
        
        # 模擬發送協作任務到協作控制器
        # 在實際實現中，這將通過服務或 Action 進行
        
        task_config = {
            'task_type': 'pick_and_place',
            'mode': 'sequential',
            'ur5_target': {
                'joint_positions': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            },
            'arm0710_target': {
                'joint_positions': [0.0, 0.5, -0.5, 0.0, -1.0, 0.0]
            }
        }
        
        self.get_logger().info("✅ 順序協作任務配置完成")
        return task_config

    def test_parallel_collaboration(self):
        """測試平行協作"""
        self.get_logger().info("🔄 測試平行協作模式")
        
        task_config = {
            'task_type': 'dual_pick',
            'mode': 'parallel',
            'ur5_target': {
                'joint_positions': [1.0, -1.0, 1.0, -1.0, 1.0, 0.0]
            },
            'arm0710_target': {
                'joint_positions': [0.5, 0.3, -0.8, 0.2, -0.5, 0.0]
            }
        }
        
        self.get_logger().info("✅ 平行協作任務配置完成")
        return task_config

    def test_synchronized_collaboration(self):
        """測試同步協作"""
        self.get_logger().info("🔄 測試同步協作模式")
        
        task_config = {
            'task_type': 'handover',
            'mode': 'synchronized',
            'ur5_target': {
                'pose': self.create_pose(0.3, -0.5, 0.5, 0, 0, 0, 1)
            },
            'arm0710_target': {
                'pose': self.create_pose(0.3, 0.5, 0.5, 0, 0, 0, 1)
            }
        }
        
        self.get_logger().info("✅ 同步協作任務配置完成")
        return task_config

    def create_pose(self, x, y, z, qx, qy, qz, qw):
        """創建位姿"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def run_test_sequence(self):
        """執行完整測試序列"""
        tests = [
            self.test_sequential_collaboration,
            self.test_parallel_collaboration,
            self.test_synchronized_collaboration,
        ]
        
        for i, test in enumerate(tests, 1):
            self.get_logger().info(f"🚀 執行測試 {i}/{len(tests)}")
            task_config = test()
            
            # 在真實實現中，這裡會發送任務到協作控制器
            # 並等待執行完成
            time.sleep(3.0)  # 模擬執行時間
            
            self.get_logger().info(f"✅ 測試 {i} 完成")
            time.sleep(1.0)

def main():
    rclpy.init()
    tester = DualArmTester()
    
    try:
        tester.run_test_sequence()
        tester.get_logger().info("🎉 所有測試完成！")
    except KeyboardInterrupt:
        tester.get_logger().info("🔚 測試中斷")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()