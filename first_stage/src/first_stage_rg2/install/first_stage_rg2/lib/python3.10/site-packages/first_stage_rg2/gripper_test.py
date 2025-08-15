#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # 建立Service Clients
        self.open_client = self.create_client(Trigger, 'open_gripper')
        self.close_client = self.create_client(Trigger, 'close_gripper')
        
        self.get_logger().info('夾爪測試程式啟動')

    def wait_for_services(self):
        """等待服務可用"""
        self.get_logger().info('等待夾爪服務...')
        
        while not self.open_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('張開夾爪服務尚未可用，等待中...')
        
        while not self.close_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('關閉夾爪服務尚未可用，等待中...')
        
        self.get_logger().info('所有服務已就緒！')

    def open_gripper(self):
        """呼叫張開夾爪服務"""
        self.get_logger().info('=== 執行張開夾爪 ===')
        
        request = Trigger.Request()
        future = self.open_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            if result.success:
                self.get_logger().info(f'✓ 成功: {result.message}')
            else:
                self.get_logger().error(f'✗ 失敗: {result.message}')
        else:
            self.get_logger().error('服務呼叫失敗')

    def close_gripper(self):
        """呼叫關閉夾爪服務"""
        self.get_logger().info('=== 執行關閉夾爪 ===')
        
        request = Trigger.Request()
        future = self.close_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            if result.success:
                self.get_logger().info(f'✓ 成功: {result.message}')
            else:
                self.get_logger().error(f'✗ 失敗: {result.message}')
        else:
            self.get_logger().error('服務呼叫失敗')

    def run_test_sequence(self):
        """執行測試序列"""
        self.wait_for_services()
        
        self.get_logger().info('開始夾爪測試序列...')
        
        # 測試序列：張開 -> 關閉 -> 張開
        import time
        
        self.open_gripper()
        time.sleep(3)
        
        self.close_gripper()
        time.sleep(3)
        
        self.open_gripper()
        
        self.get_logger().info('測試序列完成！')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = GripperTester()
        tester.run_test_sequence()
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()