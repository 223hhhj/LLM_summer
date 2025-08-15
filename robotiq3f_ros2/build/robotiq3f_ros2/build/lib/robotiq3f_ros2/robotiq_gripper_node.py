#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger
from pyModbusTCP.client import ModbusClient
import time
import threading
import warnings
import json

class GripperController:
    def __init__(self, server_ip, port=502, unit_id=9, update_interval=0.1):
        # Initialize Modbus client and update thread
        self.client = ModbusClient(host=server_ip, port=port, auto_open=True, unit_id=unit_id)
        self.client.open()
        self.update_interval = update_interval
        self._running = True
        self._thread = threading.Thread(target=self._update_loop)
        self._thread.start()

    def _update_loop(self):
        """Continuously update gripper status."""
        while self._running:
            self._update_status()
            time.sleep(self.update_interval)

    def _update_status(self):
        """Update and store the gripper status from Modbus server."""
        readData = self.client.read_input_registers(0,8)
        if readData:
            self.gSTA, self.gIMC, self.gGTO, self.gMOD, self.gACT, self.gDTS, self.gDTC, self.gDTB, self.gDTA = self.stat(self.add_leading_zeros(bin(readData[0])))
            self.FaultStatus, self.FingerA_PositionReqEcho = self.Byte_status(self.add_leading_zeros(bin(readData[1])))
            self.FingerA_Position, self.FingerA_Current = self.Byte_status(self.add_leading_zeros(bin(readData[2])))
            self.FingerB_PositionReqEcho, self.FingerB_Position = self.Byte_status(self.add_leading_zeros(bin(readData[3])))
            self.FingerB_Current, self.FingerC_PositionReqEcho = self.Byte_status(self.add_leading_zeros(bin(readData[4])))
            self.FingerC_Position, self.FingerC_Current = self.Byte_status(self.add_leading_zeros(bin(readData[5])))
            self.Scissor_PositionReqEcho, self.Scissor_Position = self.Byte_status(self.add_leading_zeros(bin(readData[6])))
            self.Scissor_Current, RES = self.Byte_status(self.add_leading_zeros(bin(readData[7])))
        else:
            print("Error reading data in update_status.")
        
    @staticmethod
    def add_leading_zeros(bin_num, total_length=16):
        """Ensure binary number string has correct number of digits."""
        bin_str = str(bin_num)[2:]  # remove '0b' prefix
        return bin_str.zfill(total_length)

    @staticmethod
    def Byte_status(variable: int) -> str:
        """Split and parse byte status."""
        B1 = int(variable[0:8],2)
        B2 = int(variable[8:16],2)
        return B1, B2

    @staticmethod
    def stat(variable: int) -> str:
        """Split and parse status."""
        # Define gripper modes
        modes = ["Basic Mode", "Pinch Mode", "Wide Mode", "Scissor Mode"]

        # Split and parse status bits
        gSTA = int(variable[0:2],2)
        gIMC = int(variable[2:4],2)
        gGTO = int(variable[4],2)
        gMOD = int(variable[5:7],2)
        gMOD = modes[gMOD]  # Translate mode code to string
        gACT = int(variable[7],2)
        gDTS = int(variable[8:10],2)
        gDTC = int(variable[10:12],2)
        gDTB = int(variable[12:14],2)
        gDTA = int(variable[14:16],2)
        
        return gSTA, gIMC, gGTO, gMOD, gACT, gDTS, gDTC, gDTB, gDTA
    
    def activate(self):
        """Activate the gripper."""
        self.client.write_multiple_registers(
                0,
                [self._action_req_variable(rACT=1, rGTO=1, rMOD=0, rICF=0),
                self._position_req_variable(0),
                self._write_req_variable(0, 0)]
            )
        print("Gripper activate")
        time.sleep(1)

    def command_gripper(self, rPRA=[1, 1, 1], rSP=[250, 250, 250], rFR=[250, 250, 250], rMOD="Basic", rICF=False):
        """Send a command to the gripper."""
        modes = {"Basic": 0, "Pinch": 1, "Wide": 2, "Scissor": 3}
        rMOD = modes[rMOD]
        if rICF:
            for var in [rPRA, rSP, rFR]:
                if isinstance(var, int):
                    self.close()
                    raise ValueError("Input variables must be 3d vectors when using Individual Control Flag.")
            response = self.client.write_multiple_registers(
                0,
                [self._action_req_variable(rACT=1, rGTO=1, rMOD=rMOD, rICF=1),
                self._position_req_variable(rPRA[0]),
                self._write_req_variable(rSP[0], rFR[0]),
                self._write_req_variable(rPRA[1],rSP[1]),
                self._write_req_variable(rFR[1],rPRA[2]),
                self._write_req_variable(rSP[2], rFR[2]),]
            )
        else:
            for var in [rPRA, rSP, rFR]:
                if isinstance(var, list):
                    warnings.warn("only first value of 3d vector will be used when not using Individual Control Flag.") 
            response = self.client.write_multiple_registers(
                0,
                [self._action_req_variable(rACT=1, rGTO=1, rMOD=rMOD, rICF=0),
                self._position_req_variable(rPRA[0]),
                self._write_req_variable(rSP[0], rFR[0])]
            )

    def _action_req_variable(self, rARD: int = 0, rATR: int = 0, rGTO: int = 0, rACT: int = 0, rMOD:int = 0, rICS:int = 0, rICF:int = 0 ) -> str:
        """Build action request variable."""
        for var in [rARD, rATR, rGTO, rACT]:
            if var not in [0, 1]:
                raise ValueError("Input variables must be either 0 or 1.")
        rMOD = bin(rMOD).replace("0b", "").zfill(2)
        string_variable = f"0b00{rARD}{rATR}{rGTO}{rMOD}{rACT}0000{rICS}{rICF}00" 
        return int(string_variable,2)
    
    def _position_req_variable(self, rPR: int = 0) -> str:
        """Build position request variable."""
        for var in [rPR]:
            if var not in range(0,256):
                raise ValueError("Input variables must be between 0 and 255.")
        rPR = format(rPR, '08b')
        string_variable = f"0b00000000{rPR}"
        return int(string_variable,2) 
    
    def _write_req_variable(self, X: int = 0, Y: int = 0) -> str:
        """Build write request variable."""
        for var in [X, Y]:
            if var not in range(0,256):
                raise ValueError("Input variables must be between 0 and 255.")
        X = format(X, '08b')
        Y = format(Y, '08b')
        string_variable = f"0b{X}{Y}"
        return int(string_variable,2)
    
    def close(self):
        """Stop the update thread and close the Modbus client."""
        self._running = False
        self._thread.join()
        self.client.close()
        print("Connection closed.")

class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__("robotiq_gripper_node")
        self.get_logger().info("Hello Robotiq Gripper Node")
        
        # 宣告參數
        self.declare_parameter('gripper_ip', '192.168.1.11')
        self.declare_parameter('modbus_port', 502)
        self.declare_parameter('unit_id', 9)
        self.declare_parameter('update_interval', 0.1)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('status_publish_rate', 1.0)  # 狀態發布頻率 (Hz)
        
        # 取得參數值
        gripper_ip = self.get_parameter('gripper_ip').get_parameter_value().string_value
        modbus_port = self.get_parameter('modbus_port').get_parameter_value().integer_value
        unit_id = self.get_parameter('unit_id').get_parameter_value().integer_value
        update_interval = self.get_parameter('update_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        status_publish_rate = self.get_parameter('status_publish_rate').get_parameter_value().double_value
        
        # 訂閱 robotiq_gripper topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'robotiq_gripper',
            self.listener_callback,
            10)
        
        # 建立狀態發布器
        self.status_publisher = self.create_publisher(
            String,
            'robotiq_gripper_status',
            10)
        
        # 建立位置發布器 (方便其他節點使用)
        self.position_publisher = self.create_publisher(
            Int32MultiArray,
            'robotiq_gripper_position',
            10)
        
        # 建立狀態發布定時器
        self.status_timer = self.create_timer(
            1.0 / status_publish_rate,
            self.publish_status_callback)
           
        try:
            # 初始化夾爪控制器
            self.gripper = GripperController(gripper_ip, modbus_port, unit_id, update_interval)
            self.gripper.activate()
            self.get_logger().info(f"Connected to gripper at {gripper_ip}:{modbus_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to gripper: {str(e)}")
            return
        
        # 初始化服務客戶端（如果需要的話）
        # self.minimal_client = MinimalClientAsync()
    
    def listener_callback(self, msg):
        """處理接收到的夾爪指令"""
        try:
            # 解析指令格式
            # 期望格式: [position1, position2, position3, speed, force, mode_id, individual_control]
            # 或簡化格式: [position, speed, force]
            
            data = msg.data
            
            if len(data) >= 7:
                # 完整格式 - 三指獨立控制
                target_position = [data[0], data[1], data[2]]
                speed = [data[3], data[3], data[3]]  # 使用相同速度
                force = [data[4], data[4], data[4]]  # 使用相同力量
                mode_id = data[5]
                individual_control = bool(data[6])
                
                # 模式對應
                modes = ["Basic", "Pinch", "Wide", "Scissor"]
                mode = modes[mode_id] if 0 <= mode_id < len(modes) else "Basic"
                
            elif len(data) >= 3:
                # 簡化格式 - 統一控制
                target_position = [data[0], data[0], data[0]]  # 三指使用相同位置
                speed = [data[1], data[1], data[1]]
                force = [data[2], data[2], data[2]]
                mode = "Basic"
                individual_control = False
                
            else:
                self.get_logger().error("Invalid command format. Need at least 3 values.")
                return
            
            # 檢查數值範圍
            for pos in target_position:
                if not 0 <= pos <= 255:
                    self.get_logger().error(f"Position {pos} out of range (0-255)")
                    return
            
            for spd in speed:
                if not 0 <= spd <= 255:
                    self.get_logger().error(f"Speed {spd} out of range (0-255)")
                    return
                    
            for frc in force:
                if not 0 <= frc <= 255:
                    self.get_logger().error(f"Force {frc} out of range (0-255)")
                    return
            
            # 發送夾爪指令
            self.gripper.command_gripper(
                rPRA=target_position,
                rSP=speed,
                rFR=force,
                rMOD=mode,
                rICF=individual_control
            )
            
            self.get_logger().info(f"Gripper command sent - Position: {target_position}, Speed: {speed}, Force: {force}, Mode: {mode}, Individual: {individual_control}")
            
            # 顯示當前狀態
            self.get_logger().info(f"Current Status: {self.get_status_summary()}")
            
            # 如果需要，可以等待夾爪到達目標位置
            # self.wait_for_position(target_position)
            
        except Exception as e:
            self.get_logger().error(f"Error processing gripper command: {str(e)}")
    
    def wait_for_position(self, target_position):
        """等待夾爪到達目標位置"""
        start_time = time.time()
        
        while time.time() - start_time < self.timeout:
            try:
                current_pos = [
                    self.gripper.FingerA_Position,
                    self.gripper.FingerB_Position,
                    self.gripper.FingerC_Position
                ]
                
                # 檢查是否到達目標位置（允許小誤差）
                if all(abs(current - target) <= 2 for current, target in zip(current_pos, target_position)):
                    self.get_logger().info(f"Gripper reached target position: {current_pos}")
                    break
                    
                time.sleep(0.1)
            except AttributeError:
                # 如果還沒有位置資料，繼續等待
                time.sleep(0.1)
                continue
        else:
            self.get_logger().warning("Gripper position timeout")
    
    def publish_status_callback(self):
        """定期發布夾爪狀態資訊"""
        try:
            if not hasattr(self, 'gripper'):
                return
                
            # 取得夾爪狀態
            status_data = {
                "timestamp": time.time(),
                "finger_positions": {
                    "finger_a": getattr(self.gripper, 'FingerA_Position', 0),
                    "finger_b": getattr(self.gripper, 'FingerB_Position', 0),
                    "finger_c": getattr(self.gripper, 'FingerC_Position', 0)
                },
                "finger_currents": {
                    "finger_a": getattr(self.gripper, 'FingerA_Current', 0),
                    "finger_b": getattr(self.gripper, 'FingerB_Current', 0),
                    "finger_c": getattr(self.gripper, 'FingerC_Current', 0)
                },
                "scissor_position": getattr(self.gripper, 'Scissor_Position', 0),
                "scissor_current": getattr(self.gripper, 'Scissor_Current', 0),
                "gripper_status": {
                    "activation_status": getattr(self.gripper, 'gSTA', 0),
                    "initialization_status": getattr(self.gripper, 'gIMC', 0),
                    "action_status": getattr(self.gripper, 'gGTO', 0),
                    "gripper_mode": getattr(self.gripper, 'gMOD', 'Unknown'),
                    "activation_bit": getattr(self.gripper, 'gACT', 0),
                    "finger_a_status": getattr(self.gripper, 'gDTA', 0),
                    "finger_b_status": getattr(self.gripper, 'gDTB', 0),
                    "finger_c_status": getattr(self.gripper, 'gDTC', 0),
                    "scissor_status": getattr(self.gripper, 'gDTS', 0)
                },
                "fault_status": getattr(self.gripper, 'FaultStatus', 0)
            }
            
            # 發布詳細狀態 (JSON格式)
            status_msg = String()
            status_msg.data = json.dumps(status_data, indent=2)
            self.status_publisher.publish(status_msg)
            
            # 發布簡單的位置資訊
            position_msg = Int32MultiArray()
            position_msg.data = [
                status_data["finger_positions"]["finger_a"],
                status_data["finger_positions"]["finger_b"],
                status_data["finger_positions"]["finger_c"],
                status_data["scissor_position"]
            ]
            self.position_publisher.publish(position_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing gripper status: {str(e)}")
    
    def get_status_summary(self):
        """取得夾爪狀態摘要"""
        try:
            if not hasattr(self, 'gripper'):
                return "Gripper not connected"
                
            finger_a_pos = getattr(self.gripper, 'FingerA_Position', 0)
            finger_b_pos = getattr(self.gripper, 'FingerB_Position', 0)
            finger_c_pos = getattr(self.gripper, 'FingerC_Position', 0)
            mode = getattr(self.gripper, 'gMOD', 'Unknown')
            activation = getattr(self.gripper, 'gACT', 0)
            
            status_text = f"Mode: {mode} | Active: {'Yes' if activation else 'No'} | "
            status_text += f"Positions: A={finger_a_pos}, B={finger_b_pos}, C={finger_c_pos}"
            
            return status_text
            
        except Exception as e:
            return f"Error reading status: {str(e)}"
    
    def __del__(self):
        """確保在節點銷毀時關閉夾爪連接"""
        try:
            if hasattr(self, 'gripper'):
                self.gripper.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotiqGripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 確保正確關閉夾爪連接
        if 'node' in locals():
            try:
                node.gripper.close()
            except:
                pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()

