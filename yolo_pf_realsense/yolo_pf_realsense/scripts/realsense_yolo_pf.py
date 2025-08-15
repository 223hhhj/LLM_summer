#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
import random
import time
import threading
from ultralytics import YOLO
from scipy.ndimage import uniform_filter1d

# 添加 TF2 相關導入
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

class AdvancedParticleFilter:
    """高級粒子濾波器，整合3D版本的先進功能"""
    def __init__(self, num_particles=500, init_range=(0, 640, 0, 480)):
        self.num_particles = num_particles
        x_min, x_max, y_min, y_max = init_range
        
        # 粒子狀態：[x, y, vx, vy, weight] - 增加速度狀態
        self.particles = np.zeros((num_particles, 5))
        
        # 初始化位置
        self.particles[:, 0] = np.random.uniform(x_min, x_max, num_particles)  # x
        self.particles[:, 1] = np.random.uniform(y_min, y_max, num_particles)  # y
        self.particles[:, 2] = np.random.normal(0, 5, num_particles)           # vx
        self.particles[:, 3] = np.random.normal(0, 5, num_particles)           # vy
        self.particles[:, 4] = 1.0 / num_particles                            # weight
        
        # 高級參數設定
        self.state_std = 8.0           # 狀態雜訊標準差
        self.measurement_std = 20.0    # 測量雜訊標準差  
        self.velocity_std = 3.0        # 速度雜訊標準差
        
        # 平滑參數
        self.smoothing_window = 5
        self.position_history = []
        self.smoothed_positions = []
        
        # 加速度記憶
        self.previous_acceleration = None
        
        # 自適應重採樣
        self.resample_threshold = 0.5
        
    def predict(self, dt=1.0, motion=(0, 0)):
        """預測階段 - 整合3D版本的高級預測模型"""
        # 1. 加速度記憶機制
        if self.previous_acceleration is None:
            self.previous_acceleration = np.random.normal(0, 0.5, (self.num_particles, 2))
        else:
            # 加速度衰減
            self.previous_acceleration *= 0.9
            # 應用加速度到速度
            self.particles[:, 2:4] += self.previous_acceleration * dt
        
        # 2. 運動學更新：位置 = 位置 + 速度 * 時間
        self.particles[:, 0:2] += self.particles[:, 2:4] * dt
        
        # 3. 添加外部運動向量
        dx, dy = motion
        self.particles[:, 0] += dx
        self.particles[:, 1] += dy
        
        # 4. 自適應雜訊 - 根據速度調整雜訊大小
        velocities = np.linalg.norm(self.particles[:, 2:4], axis=1)
        adaptive_noise = self.state_std * (1 + 0.3 * np.exp(-velocities / 10.0))
        
        # 5. 添加過程雜訊
        position_noise = np.random.normal(0, adaptive_noise[:, None], (self.num_particles, 2))
        velocity_noise = np.random.normal(0, self.velocity_std, (self.num_particles, 2))
        
        self.particles[:, 0:2] += position_noise
        self.particles[:, 2:4] += velocity_noise
        
        # 6. 邊界約束
        x_min, x_max, y_min, y_max = 0, 640, 0, 480
        self.particles[:, 0] = np.clip(self.particles[:, 0], x_min, x_max)
        self.particles[:, 1] = np.clip(self.particles[:, 1], y_min, y_max)

    def update(self, measurement):
        """更新階段 - 整合多重權重機制"""
        if measurement is None:
            return
            
        x_meas, y_meas = measurement
        
        # 1. 基於距離的權重計算
        dist = np.linalg.norm(self.particles[:, 0:2] - np.array([x_meas, y_meas]), axis=1)
        self.particles[:, 4] = np.exp(-0.5 * (dist ** 2) / (self.measurement_std ** 2))
        
        # 2. 時間平滑權重 - 考慮與前一位置的連續性
        if len(self.position_history) > 0:
            prev_pos = self.position_history[-1]
            prev_dist = np.linalg.norm(self.particles[:, 0:2] - prev_pos, axis=1)
            smoothing_weights = np.exp(-0.5 * (prev_dist ** 2) / (self.state_std ** 2))
            self.particles[:, 4] *= (0.9 * smoothing_weights + 0.1)
        
        # 3. 權重正規化
        total_weight = np.sum(self.particles[:, 4])
        if total_weight > 0:
            self.particles[:, 4] /= total_weight
        else:
            self.particles[:, 4] = 1.0 / self.num_particles

    def get_estimate(self):
        """估計階段 - 整合多重平滑機制"""
        # 1. 加權平均估計
        weights = self.particles[:, 4]
        x_estimate = np.sum(self.particles[:, 0] * weights)
        y_estimate = np.sum(self.particles[:, 1] * weights)
        
        # 2. 更新位置歷史
        current_pos = np.array([x_estimate, y_estimate])
        self.position_history.append(current_pos)
        
        # 3. 多重平滑處理
        if len(self.position_history) >= self.smoothing_window:
            # 滑動窗口平滑
            positions = np.array(self.position_history[-self.smoothing_window:])
            smoothed_pos = np.mean(positions, axis=0)  # 簡化版uniform_filter1d
            
            # 指數加權平滑
            if len(self.smoothed_positions) > 0:
                alpha = 0.7
                prev_smooth = self.smoothed_positions[-1]
                smoothed_pos = alpha * smoothed_pos + (1-alpha) * prev_smooth
            
            self.smoothed_positions.append(smoothed_pos)
            return int(smoothed_pos[0]), int(smoothed_pos[1])
        
        # 限制歷史長度
        if len(self.position_history) > 20:
            self.position_history.pop(0)
        if len(self.smoothed_positions) > 20:
            self.smoothed_positions.pop(0)
            
        return int(x_estimate), int(y_estimate)
    
    def neff(self):
        """計算有效粒子數"""
        return 1.0 / np.sum(np.square(self.particles[:, 4]))
    
    def resample(self):
        """重採樣階段 - 整合系統重採樣和自適應雜訊"""
        # 1. 檢查是否需要重採樣
        if self.neff() > self.num_particles * self.resample_threshold:
            return
        
        # 2. 系統重採樣演算法
        weights = self.particles[:, 4]
        cumsum = np.cumsum(weights)
        step = 1.0 / self.num_particles
        u = np.arange(0, 1, step)
        u += np.random.uniform(0, step)
        
        indices = np.searchsorted(cumsum, u)
        self.particles = self.particles[indices]
        
        # 3. 自適應雜訊注入 - 防止粒子退化
        avg_velocity = np.mean(np.linalg.norm(self.particles[:, 2:4], axis=1))
        noise_scale = max(0.5, min(5.0, avg_velocity * 0.1))
        
        # 位置雜訊
        self.particles[:, 0:2] += np.random.normal(0, noise_scale, (self.num_particles, 2))
        # 速度雜訊
        self.particles[:, 2:4] += np.random.normal(0, noise_scale * 0.3, (self.num_particles, 2))
        
        # 4. 重置權重
        self.particles[:, 4] = 1.0 / self.num_particles
        
    def get_particles_for_display(self):
        """獲取粒子位置用於視覺化"""
        return self.particles[:, 0:2].astype(int)
    
    def get_velocity_info(self):
        """獲取速度資訊用於顯示"""
        weights = self.particles[:, 4]
        avg_vx = np.sum(self.particles[:, 2] * weights)
        avg_vy = np.sum(self.particles[:, 3] * weights)
        speed = np.sqrt(avg_vx**2 + avg_vy**2)
        return avg_vx, avg_vy, speed

class RealsenseYOLOTracker(Node):
    def __init__(self):
        super().__init__('realsense_yolo_tracker')
        
        # 宣告參數
        self.declare_parameter('serial_a', '140122074462')  # 相機1的序列號
        self.declare_parameter('serial_b', '844212070157')  # 相機2的序列號
        self.declare_parameter('publish_rate', 30.0)  # 發布頻率 (Hz)
        self.declare_parameter('yolo_model_path', '/home/steven/Downloads/runs/detect/train2/weights/best.pt')  # YOLO模型路徑
        self.declare_parameter('arm_z_threshold', 0.205)  # 手臂 z 座標安全閾值
        
        # 獲取參數
        self.serial_a = self.get_parameter('serial_a').value
        self.serial_b = self.get_parameter('serial_b').value
        publish_rate = self.get_parameter('publish_rate').value
        self.yolo_model_path = self.get_parameter('yolo_model_path').value
        self.arm_z_threshold = self.get_parameter('arm_z_threshold').value

        # 初始化 TF2 相關組件
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 手臂安全狀態
        self.arm_safe_position = True
        self.last_arm_check_time = time.time()
        self.arm_check_interval = 0.1  # 每 100ms 檢查一次手臂位置

        # 校正矩陣
        self.calibration_matrices = {
            self.serial_a: np.array([
                [ 3.26089698e-04 ,-1.75862220e-05 ,-2.15716188e-01  ,5.08350697e-02],
                [-1.99080463e-05 ,-5.03942395e-04 ,-2.94761575e-02 ,-4.24283309e-01],
                [-1.11808349e-19 , 2.04981973e-19 , 2.08166817e-17 , 2.31999993e-01],
                [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]
                # [ 3.50759037e-04 ,-1.57342881e-05 ,-5.24834268e-03 ,-2.31204525e-02],
                # [ 7.42037347e-07 ,-5.42445669e-04 ,-9.33346010e-02, -4.11165647e-01],
                # [ 6.50521303e-19 , 0.00000000e+00  ,4.90038783e-02 , 2.15876118e-01],
                # [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]
                # [ 3.47377816e-04 ,-2.40837638e-05, -7.52764974e-03 ,-2.96364154e-02],
                # [-1.86728766e-05, -5.57890293e-04 ,-9.94210998e-02 ,-3.91421640e-01],
                # [ 1.08420217e-19 , 6.50521303e-19 , 5.84584947e-02 , 2.30151547e-01],
                # [ 0.00000000e+00  ,0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]
            ]),
            self.serial_b: np.array([
                [2.06689677e-04, 1.23002824e-04, 1.02221523e-07, 5.61656732e-07],
                [-1.09364930e-03, -6.50840206e-04, -5.40880891e-07, -2.97187309e-06],
                [4.81607004e-04, 2.86608516e-04, 2.38186068e-07, 1.30871469e-06],
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ])
        }
        
        # 創建 CV Bridge
        self.cv_bridge = CvBridge()
        
        # 加載 YOLO 模型
        self.get_logger().info(f'Loading enhanced YOLO model from {self.yolo_model_path}')
        self.yolo_model = YOLO(self.yolo_model_path)
        
        # 創建高級粒子濾波器字典
        self.particle_filters = {
            'cam_a': {},  # 鍵為目標 ID，值為 AdvancedParticleFilter
            'cam_b': {}
        }
        
        # 創建影像發布者
        self.image_publishers = {
            'cam_a': self.create_publisher(Image, '/camera/cam_a/color/image_raw', 10),
            'cam_b': self.create_publisher(Image, '/camera/cam_b/color/image_raw', 10)
        }
        
        # 創建位置發布者
        self.position_publishers = {
            'cam_a': {},
            'cam_b': {}
        }
        # 主要目標發布者
        self.position_publishers['cam_a']['main'] = self.create_publisher(Point, '/circle_position/camera1', 10)
        self.position_publishers['cam_b']['main'] = self.create_publisher(Point, '/circle_position/camera2', 10)
        
        # 目標 ID 計數器
        self.target_id_counter = 0
        
        # 初始化相機
        self.pipelines = {}
        self.init_camera(self.serial_a, 'cam_a')
        self.init_camera(self.serial_b, 'cam_b')
        
        # 圖像數據
        self.latest_images = {}
        self.depth_images = {}
        
        # 執行緒狀態
        self.running = True
        
        # 目標類別
        self.target_classes = []
        
        # 創建執行緒
        self.threads = [
            threading.Thread(target=self.camera_thread, args=(self.serial_a, 'cam_a')),
            threading.Thread(target=self.camera_thread, args=(self.serial_b, 'cam_b'))
        ]
        
        # 啟動執行緒
        for thread in self.threads:
            thread.start()
        
        # 創建定時器
        self.timer = self.create_timer(1.0/publish_rate, self.process_and_publish)
        
        self.get_logger().info(f'Enhanced RealSense YOLO追蹤節點已初始化（整合高級粒子濾波器和手臂安全檢查，閾值: {self.arm_z_threshold}）')
    
    def check_arm_safety(self):
        """檢查手臂是否在安全位置"""
        current_time = time.time()
        
        # 限制檢查頻率以避免過度計算
        if current_time - self.last_arm_check_time < self.arm_check_interval:
            return self.arm_safe_position
        
        self.last_arm_check_time = current_time
        
        try:
            # 獲取 base_link 到 tool0 的變換
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'tool0', 
                rclpy.time.Time()
            )
            
            # 提取 z 座標
            arm_z = transform.transform.translation.z
            
            # 檢查是否在安全範圍內
            previous_state = self.arm_safe_position
            self.arm_safe_position = arm_z >= self.arm_z_threshold
            
            # 狀態變化時記錄日誌
            if previous_state != self.arm_safe_position:
                if self.arm_safe_position:
                    self.get_logger().info(f'手臂進入安全位置 (z={arm_z:.3f} >= {self.arm_z_threshold}) - 恢復發布指令')
                else:
                    self.get_logger().warn(f'手臂位置過低 (z={arm_z:.3f} < {self.arm_z_threshold}) - 停止發布指令以確保安全')
            
            return self.arm_safe_position
            
        except TransformException as ex:
            # TF 變換不可用時，記錄警告但不阻止發布
            if hasattr(self, '_tf_error_logged') and not self._tf_error_logged:
                self.get_logger().warn(f'無法獲取手臂座標變換: {ex}，將繼續正常運行')
                self._tf_error_logged = True
            return True  # 預設為安全，避免因 TF 問題而完全停止
    
    def init_camera(self, serial, camera_name):
        """初始化指定序列號的相機"""
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            pipeline.start(config)
            self.pipelines[serial] = pipeline
            self.get_logger().info(f'相機 {camera_name} (序列號: {serial}) 初始化成功')
        except Exception as e:
            self.get_logger().error(f'相機 {camera_name} (序列號: {serial}) 初始化失敗: {str(e)}')
    
    def camera_thread(self, serial, camera_name):
        """持續獲取指定相機的圖像"""
        try:
            pipeline = self.pipelines.get(serial)
            if not pipeline:
                self.get_logger().error(f'相機 {camera_name} 的管道不存在')
                return
            
            while self.running and rclpy.ok():
                try:
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    depth_frame = frames.get_depth_frame()
                    
                    if not color_frame or not depth_frame:
                        time.sleep(0.01)
                        continue
                    
                    color_image = np.asanyarray(color_frame.get_data())
                    depth_image = np.asanyarray(depth_frame.get_data())
                    
                    self.latest_images[camera_name] = color_image
                    self.depth_images[camera_name] = depth_image
                    
                except Exception as e:
                    self.get_logger().warning(f'相機 {camera_name} 獲取圖像時發生錯誤: {str(e)}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'相機 {camera_name} 執行緒發生錯誤: {str(e)}')
    
    def transform_coordinates(self, x, y, camera_name):
        """將相機座標轉換為世界座標"""
        serial = self.serial_a if camera_name == 'cam_a' else self.serial_b
        point_cam = np.array([x, y, 0.264, 1])
        calibration_matrix = self.calibration_matrices[serial]
        transformed = np.dot(calibration_matrix, point_cam)
        return transformed[0], transformed[1], transformed[2]
    
    def get_depth(self, x, y, camera_name):
        """獲取特定點的深度"""
        if camera_name not in self.depth_images:
            return None
        
        depth_image = self.depth_images[camera_name]
        if depth_image is None:
            return None
            
        if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
            depth = depth_image[y, x]
            if depth == 0:
                return None
            depth_meters = depth / 1000.0
            return depth_meters
        return None

    def assign_target_id(self, camera_name, center_x, center_y):
        """為檢測到的目標分配 ID"""
        if not self.particle_filters[camera_name]:
            target_id = f"target_{self.target_id_counter}"
            self.target_id_counter += 1
            return target_id

        min_distance = float('inf')
        closest_id = None
        for target_id, particle_filter in self.particle_filters[camera_name].items():
            x_est, y_est = particle_filter.get_estimate()
            distance = np.sqrt((center_x - x_est) ** 2 + (center_y - y_est) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_id = target_id

        if min_distance < 80:  # 增加閾值以適應更平滑的追蹤
            return closest_id
        else:
            target_id = f"target_{self.target_id_counter}"
            self.target_id_counter += 1
            return target_id
    
    def assign_particle_filter(self, camera_name, target_id):
        """為新目標分配高級粒子濾波器"""
        if target_id not in self.particle_filters[camera_name]:
            self.particle_filters[camera_name][target_id] = AdvancedParticleFilter(
                num_particles=500, init_range=(0, 640, 0, 480)
            )
            self.get_logger().info(f'為 {camera_name} 的目標 {target_id} 分配高級粒子濾波器')
    
    def remove_old_targets(self, camera_name, active_target_ids):
        """移除不再檢測到的目標"""
        for target_id in list(self.particle_filters[camera_name].keys()):
            if target_id not in active_target_ids:
                del self.particle_filters[camera_name][target_id]
                if target_id in self.position_publishers[camera_name]:
                    del self.position_publishers[camera_name][target_id]
                    self.get_logger().info(f'移除 {camera_name} 的目標 {target_id}')
    
    def get_arm_z_coordinate(self):
        """獲取手臂當前的 z 座標"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'tool0', 
                rclpy.time.Time()
            )
            return transform.transform.translation.z
        except TransformException:
            # 如果無法獲取手臂座標，返回預設值
            return 0.265

    def publish_position(self, x, y, camera_name, target_id, is_main=False):
        """发布物体位置 - 修改为在不安全时发布 NaN"""
        
        # 检查手臂安全位置
        if not self.check_arm_safety():
            # 手臂位置不安全，发布 NaN 坐标作为停止信号
            msg = Point()
            msg.x = float('nan')
            msg.y = float('nan') 
            msg.z = float('nan')
            
            # 发布 NaN 坐标到主要目标（触发机器人切换到 ADJUST_XYZ 状态）
            if is_main:
                self.position_publishers[camera_name]['main'].publish(msg)
                camera_id = '1' if camera_name == 'cam_a' else '2'
                self.get_logger().warn(f'Camera {camera_id} Main: 手臂位置不安全，发布 NaN 停止信号')
            
            # 对于非主要目标，也可以选择发布 NaN 或者直接跳过
            # 这里选择跳过非主要目标的发布
            if not is_main:
                camera_id = '1' if camera_name == 'cam_a' else '2'
                self.get_logger().debug(f'Camera {camera_id} Target {target_id}: 因手臂不安全跳过发布')
            
            return
        
        # 手臂位置安全，正常发布坐标
        depth = self.get_depth(x, y, camera_name)
        if depth is None:
            depth = 0.0
            
        x_transformed, y_transformed, _ = self.transform_coordinates(x, y, camera_name)
        
        # 获取手臂当前的 z 坐标
        arm_z = self.get_arm_z_coordinate()
        
        msg = Point()
        msg.x = float(x_transformed)-0.008
        msg.y = float(y_transformed)
        msg.z = float(arm_z)+0.05  # 使用手臂的实际 z 坐标
        
        # 发布到独立主题
        if not is_main:
            if target_id not in self.position_publishers[camera_name]:
                topic_name = f'/circle_position/{camera_name}/{target_id}'
                self.position_publishers[camera_name][target_id] = self.create_publisher(Point, topic_name, 10)
                self.get_logger().info(f'为 {camera_name} 的目标 {target_id} 创建发布者: {topic_name}')
            
            self.position_publishers[camera_name][target_id].publish(msg)
        
        # 主要目标发布
        if is_main:
            self.position_publishers[camera_name]['main'].publish(msg)
        
        camera_id = '1' if camera_name == 'cam_a' else '2'
        topic_type = 'Main' if is_main else f'Target {target_id}'
        self.get_logger().info(f'Camera {camera_id} {topic_type} Published: ({msg.x}, {msg.y}, {msg.z}) [Arm Z: {arm_z:.3f}]')
    
    def process_and_publish(self):
        """處理圖像並發布結果 - 整合高級粒子濾波器"""
        for camera_name in ['cam_a', 'cam_b']:
            if camera_name not in self.latest_images:
                continue
            
            color_image = self.latest_images[camera_name]
            display_image = color_image.copy()
            
            camera_id = '1' if camera_name == 'cam_a' else '2'
            
            # 發布原始影像
            try:
                rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                img_msg = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = camera_name
                self.image_publishers[camera_name].publish(img_msg)
            except Exception as e:
                self.get_logger().warning(f'發布 {camera_name} 圖像時發生錯誤: {str(e)}')
            
            # YOLO 物體檢測
            try:
                results = self.yolo_model(color_image)
                active_target_ids = set()
                bottom_right_target = None
                candidates = []
                
                # 處理檢測結果
                if len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        if self.target_classes and box.cls not in self.target_classes:
                            continue
                        
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # 分配目標 ID 和粒子濾波器
                        target_id = self.assign_target_id(camera_name, center_x, center_y)
                        active_target_ids.add(target_id)
                        self.assign_particle_filter(camera_name, target_id)
                        
                        # 計算到右下角的距離
                        distance_to_corner = np.sqrt((center_x - 640) ** 2 + (center_y - 480) ** 2)
                        candidates.append((distance_to_corner, center_y, center_x, center_y, target_id))
                        
                        # 發布位置（包含安全檢查）
                        self.publish_position(center_x, center_y, camera_name, target_id, is_main=False)
                        
                        # 使用高級粒子濾波器進行追蹤
                        particle_filter = self.particle_filters[camera_name][target_id]
                        
                        # 預測 -> 更新 -> 重採樣 -> 估計
                        particle_filter.predict(dt=1.0, motion=(0, 0))
                        particle_filter.update((center_x, center_y))
                        particle_filter.resample()
                        x_est, y_est = particle_filter.get_estimate()
                        
                        # 獲取速度資訊
                        vx, vy, speed = particle_filter.get_velocity_info()
                        
                        # 視覺化高級粒子濾波器
                        # 繪製粒子雲（藍色）
                        particles_pos = particle_filter.get_particles_for_display()
                        for px, py in particles_pos[::5]:  # 每5個粒子顯示一個以減少視覺混亂
                            if 0 <= px < 640 and 0 <= py < 480:
                                cv2.circle(display_image, (px, py), 1, (255, 100, 0), -1)
                        
                        # 繪製估計位置（綠色大圓）
                        cv2.circle(display_image, (x_est, y_est), 12, (0, 255, 0), 3)
                        
                        # 繪製速度向量（青色箭頭）
                        if speed > 1.0:  # 只有當速度顯著時才顯示
                            end_x = int(x_est + vx * 3)
                            end_y = int(y_est + vy * 3)
                            cv2.arrowedLine(display_image, (x_est, y_est), (end_x, end_y), (255, 255, 0), 2)
                        
                        # 詳細資訊顯示
                        depth = self.get_depth(x_est, y_est, camera_name)
                        neff = particle_filter.neff()
                        
                        info_lines = [
                            f'ID {target_id}: ({x_est}, {y_est})',
                            f'Speed: {speed:.1f} px/s',
                            f'Depth: {depth:.3f}m' if depth else 'Depth: N/A',
                            f'Neff: {neff:.0f}/{particle_filter.num_particles}'
                        ]
                        
                        for i, line in enumerate(info_lines):
                            y_offset = y_est - 60 + i * 15
                            cv2.putText(display_image, line, (x_est + 15, y_offset),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                # 選擇右下角目標
                if candidates:
                    min_distance = min(cand[0] for cand in candidates)
                    close_candidates = [cand for cand in candidates if abs(cand[0] - min_distance) < 50]
                    if len(close_candidates) > 1:
                        bottom_right_target = max(close_candidates, key=lambda x: x[1])
                    else:
                        bottom_right_target = min(candidates, key=lambda x: x[0])
                
                # 繪製檢測框
                if len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        target_id = self.assign_target_id(camera_name, center_x, center_y)
                        
                        # 顏色選擇：黃色（主要目標）vs 紅色（其他目標）
                        color = (0, 255, 255) if bottom_right_target and target_id == bottom_right_target[4] else (0, 0, 255)
                        cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        cv2.circle(display_image, (center_x, center_y), 5, color, -1)
                        cv2.putText(display_image, f'ID: {target_id}', (int(x1), int(y1) - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # 發布右下角目標到主主題（包含安全檢查）
                if bottom_right_target is not None:
                    _, _, center_x, center_y, target_id = bottom_right_target
                    self.publish_position(center_x, center_y, camera_name, target_id, is_main=True)
                
                # 移除舊目標
                self.remove_old_targets(camera_name, active_target_ids)
                
                # 顯示影像
                cv2.imshow(f'Camera {camera_id} YOLO Tracking', display_image)
                
            except Exception as e:
                self.get_logger().error(f'處理 {camera_name} 圖像時發生錯誤: {str(e)}')
        
        # 處理按鍵
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('使用者按下 q 鍵，正在關閉程式...')
            self.cleanup()
    
    def cleanup(self):
        """釋放資源"""
        self.running = False
        
        # 等待所有執行緒結束
        for thread in self.threads:
            thread.join(timeout=1.0)
        
        # 停止所有相機
        for pipeline in self.pipelines.values():
            pipeline.stop()
        
        # 關閉所有視窗
        cv2.destroyAllWindows()
        
        self.get_logger().info('相機資源已釋放')


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseYOLOTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('節點被使用者中斷')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()