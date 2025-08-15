#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import random
from ultralytics import YOLO

class ParticleFilter:
    def __init__(self, num_particles=100, init_range=(0, 1280, 0, 720)):
        self.num_particles = num_particles
        x_min, x_max, y_min, y_max = init_range
        self.particles = np.array([
            [random.uniform(x_min, x_max), random.uniform(y_min, y_max), 1.0 / num_particles]
            for _ in range(num_particles)
        ])

    def predict(self, motion=(0, 0), noise_std=(10, 10)):
        dx, dy = motion
        x_noise, y_noise = noise_std
        for i in range(self.num_particles):
            self.particles[i, 0] += dx + np.random.normal(0, x_noise)
            self.particles[i, 1] += dy + np.random.normal(0, y_noise)

    def update(self, measurement, measurement_noise=20):
        if measurement is None:
            return
        x_meas, y_meas = measurement
        total_weight = 0.0
        for i in range(self.num_particles):
            distance = np.sqrt((self.particles[i, 0] - x_meas) ** 2 + (self.particles[i, 1] - y_meas) ** 2)
            self.particles[i, 2] = np.exp(- (distance ** 2) / (2 * measurement_noise ** 2))
            total_weight += self.particles[i, 2]
        if total_weight > 0:
            self.particles[:, 2] /= total_weight

    def resample(self):
        weights = self.particles[:, 2]
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=weights)
        self.particles = self.particles[indices]
        self.particles[:, 2] = 1.0 / self.num_particles

    def get_estimate(self):
        x_estimate = np.sum(self.particles[:, 0] * self.particles[:, 2])
        y_estimate = np.sum(self.particles[:, 1] * self.particles[:, 2])
        return int(x_estimate), int(y_estimate)

class CombinedDetectorNode(Node):
    def __init__(self):
        super().__init__('combined_detector_node')
        
        # ROS話題訂閱
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # 發布器
        self.blob_position_publisher = self.create_publisher(Point, '/blob_position', 10)
        self.yolo_position_publisher = self.create_publisher(Point, '/yolo_position', 10)
        
        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.depth_image = None

        # Blob 檢測器設定
        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 50
        self.params.maxThreshold = 255
        self.params.thresholdStep = 10
        self.params.filterByArea = True
        self.params.minArea = 250
        self.params.maxArea = 1000
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.1
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.2
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.2
        self.params.blobColor = 0
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        # 兩個粒子濾波器，分別用於 Blob 和 YOLO
        self.blob_particle_filter = ParticleFilter(num_particles=500)
        self.yolo_particle_filter = ParticleFilter(num_particles=500)
        
        # 校準矩陣
        self.calibration_matrix = np.array([
            [-4.49064911e-04,  1.45529593e-04 , 2.17262369e-01 , 2.51199743e-01],
            [-6.23688653e-05, -3.53414545e-05 , 2.18293567e+00 ,-8.55039463e-01],
            [-1.14586617e-17 ,-1.70160449e-17 , 1.60985808e-13 , 2.38000005e-01],
            [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]
        ])
        
        # 初始化 YOLO 模型
        self.yolo_model = YOLO('/home/steven/runs/detect/train6/weights/best.pt')
        
        # 用於選擇要追蹤的 YOLO 類別（空列表表示追蹤所有類別）
        self.target_classes = []
        
        self.get_logger().info('Combined Detector Node has been started')

    def transform_coordinates(self, x, y):
        point_cam = np.array([x, y, 0.264, 1])
        transformed = np.zeros(4)
        for i in range(4):
            transformed[i] = np.dot(self.calibration_matrix[i], point_cam)
        return transformed[0], transformed[1], transformed[2]

    def publish_position(self, publisher, x, y, detection_type):
        depth = self.get_depth(x, y)
        if depth is None:
            depth = 0.0
        x_transformed, y_transformed, _ = self.transform_coordinates(x, y)
        msg = Point()
        msg.x = float(x_transformed)
        msg.y = float(y_transformed)
        msg.z = float(depth)
        publisher.publish(msg)
        self.get_logger().info(f'{detection_type} position: ({msg.x}, {msg.y}, {msg.z})')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # 遮罩深度圖像的下半部
            self.depth_image = self.mask_lower_part(self.depth_image)
        except Exception as e:
            self.get_logger().error(f'深度影像轉換錯誤: {str(e)}')

    def get_depth(self, x, y):
        if self.depth_image is None:
            return None
        if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
            depth = self.depth_image[y, x]
            if depth == 0:
                return None
            depth_meters = depth / 1000.0
            return depth_meters
        return None

    def mask_lower_part(self, image):
        height, width = image.shape[:2]
        mask_start = height * 3 // 4
        masked_image = image.copy()
        if len(masked_image.shape) == 3:
            masked_image[mask_start:, :, :] = 0
        else:
            masked_image[mask_start:, :] = 0
        return masked_image

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        edges = cv2.Canny(thresh, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        connected_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        return connected_edges

    def process_blob_detection(self, cv_image):
        """處理 Blob 檢測的邏輯"""
        display_image = cv_image.copy()
        masked_image = self.mask_lower_part(cv_image)
        processed_image = self.preprocess_image(masked_image)
        keypoints = self.detector.detect(processed_image)

        measurement = None
        if keypoints:
            image_height, image_width = cv_image.shape[:2]
            selected_keypoint = None
            max_score = -float('inf')

            for keypoint in keypoints:
                x, y = keypoint.pt[0], keypoint.pt[1]
                mask_start = image_height * 3 // 4
                if y >= mask_start:
                    continue
                score = x + y
                if score > max_score:
                    max_score = score
                    selected_keypoint = keypoint

            if selected_keypoint:
                x, y = int(selected_keypoint.pt[0]), int(selected_keypoint.pt[1])
                radius = int(selected_keypoint.size / 2)
                cv2.circle(display_image, (x, y), radius, (0, 0, 255), 2)
                self.publish_position(self.blob_position_publisher, x, y, 'Blob')
                measurement = (x, y)

        # 更新 Blob 粒子濾波器
        if measurement is not None:
            self.blob_particle_filter.update(measurement)
            self.blob_particle_filter.resample()
        self.blob_particle_filter.predict(motion=(0, 0))

        # 繪製 Blob 粒子
        mask_start = display_image.shape[0] * 3 // 4
        for particle in self.blob_particle_filter.particles:
            x, y = int(particle[0]), int(particle[1])
            if y < mask_start:
                cv2.circle(display_image, (x, y), 1, (255, 0, 0), -1)
        
        # 繪製 Blob 估計位置
        x_est, y_est = self.blob_particle_filter.get_estimate()
        depth = self.get_depth(x_est, y_est)
        
        if y_est < mask_start:
            cv2.circle(display_image, (x_est, y_est), 10, (0, 255, 0), 2)
            coord_text = f'Blob Est: ({x_est}, {y_est})'
            depth_text = f'Depth: {depth:.3f}m' if depth is not None else 'Depth: N/A'
            cv2.putText(display_image, coord_text, (x_est + 10, y_est),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(display_image, depth_text, (x_est + 10, y_est + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return display_image, processed_image

    def process_yolo_detection(self, cv_image):
        """處理 YOLO 檢測的邏輯"""
        display_image = cv_image.copy()
        results = self.yolo_model(cv_image)
        
        measurement = None
        
        # 處理 YOLO 檢測結果
        if len(results[0].boxes) > 0:
            boxes = results[0].boxes
            selected_box = None
            max_score = -float('inf')
            
            for box in boxes:
                # 如果指定了特定類別，則只選擇這些類別
                if self.target_classes and box.cls not in self.target_classes:
                    continue
                
                # 選擇置信度最高的檢測結果
                score = box.conf[0]
                if score > max_score:
                    max_score = score
                    selected_box = box
            
            if selected_box is not None:
                # 獲取邊界框中心點
                x1, y1, x2, y2 = selected_box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # 繪製檢測結果
                cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 發布位置
                self.publish_position(self.yolo_position_publisher, center_x, center_y, 'YOLO')
                measurement = (center_x, center_y)

        # 更新 YOLO 粒子濾波器
        if measurement is not None:
            self.yolo_particle_filter.update(measurement)
            self.yolo_particle_filter.resample()
        self.yolo_particle_filter.predict(motion=(0, 0))

        # 繪製 YOLO 粒子
        for particle in self.yolo_particle_filter.particles:
            x, y = int(particle[0]), int(particle[1])
            cv2.circle(display_image, (x, y), 1, (255, 0, 0), -1)

        # 繪製 YOLO 估計位置
        x_est, y_est = self.yolo_particle_filter.get_estimate()
        depth = self.get_depth(x_est, y_est)
        cv2.circle(display_image, (x_est, y_est), 10, (0, 255, 0), 2)
        coord_text = f'YOLO Est: ({x_est}, {y_est})'
        depth_text = f'Depth: {depth:.3f}m' if depth is not None else 'Depth: N/A'
        cv2.putText(display_image, coord_text, (x_est + 10, y_est),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(display_image, depth_text, (x_est + 10, y_est + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return display_image

    def image_callback(self, msg):
        try:
            # 獲取原始圖像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 處理 Blob 檢測
            blob_display, processed_blob = self.process_blob_detection(cv_image.copy())
            
            # 處理 YOLO 檢測
            yolo_display = self.process_yolo_detection(cv_image.copy())
            
            # 顯示兩種檢測方法的結果
            cv2.imshow("Blob Detection", blob_display)
            cv2.imshow("YOLO Detection", yolo_display)
            cv2.imshow("Blob Processed", processed_blob)
            
            # 顯示深度圖
            if self.depth_image is not None:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_colormap)
            
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CombinedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("All resources released.")

if __name__ == '__main__':
    main()
































































# 單台相機yolo
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class RealSenseYOLO(Node):

    def __init__(self):
        super().__init__('realsense_yolo')
        
        # 订阅相机图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
            
        # 发布处理后的图像
        self.publisher_ = self.create_publisher(Image, '/yolov8/output', 10)
        
        # 初始化CvBridge
        self.br = CvBridge()
        
        # 加载YOLO模型
        self.yolo_model = YOLO('/home/steven/runs/detect/train6/weights/best.pt')
        
        self.get_logger().info('RealSense YOLO Node has been started')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        try:
            # 将ROS图像消息转换为OpenCV格式
            current_frame = self.br.imgmsg_to_cv2(data)
            
            # 使用YOLO模型进行目标检测
            results = self.yolo_model(current_frame)
            
            # 记录检测结果
            self.get_logger().info(f'Detection completed')
            
            # 在图像上标注检测结果
            annotated_frame = results[0].plot()
            
            # 将处理后的图像发布到ROS话题
            annotated_image_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.publisher_.publish(annotated_image_msg)
            
            self.get_logger().info('Published annotated frame')
            
            # 简单计算检测到的目标数量
            num_objects = len(results[0].boxes)
            self.get_logger().info(f'Number of objects detected: {num_objects}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    realsense_yolo = RealSenseYOLO()
    rclpy.spin(realsense_yolo)
    realsense_yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''








































































'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
from collections import deque
from geometry_msgs.msg import Point, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Int8, String, Float32
from std_srvs.srv import Trigger
import json
import math

class EnhancedZEDYOLO(Node):

    def __init__(self):
        super().__init__('enhanced_zed_yolo')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Image, '/yolov8/output', 10)
        self.coordinate_publisher = self.create_publisher(Point, '/lego_coordinates', 10)
        self.orientation_publisher = self.create_publisher(Quaternion, '/lego_orientation', 10)
        self.face_publisher = self.create_publisher(Int8, '/lego_orientation_type', 10)
        self.detection_publisher = self.create_publisher(String, '/lego_detections', 10)
        self.angle_publisher = self.create_publisher(Float32, '/lego_rotation_angle', 10)
        self.br = CvBridge()
        self.yolo_model = YOLO('/home/steven/runs/detect/train49/weights/best.pt')
        self.orientation_model = YOLO('/home/steven/runs/detect/train49/weights/best.pt')
        self.current_model = self.yolo_model
        self.get_logger().info('Enhanced ZED YOLO Node has been started')

        self.coordinates_history = deque(maxlen=30)
        self.orientation_history = deque(maxlen=30)
        self.face_history = deque(maxlen=30)
        self.calibration_matrix = np.array([
            [2.93408487e-01,  7.04756515e-03 , 0.00000000e+00 , 4.04165715e+00],
            [-5.77557483e-03 ,-2.91247281e-01 , 0.00000000e+00 , 6.84184036e+02],
            [0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 2.55000000e+02],
            [ 0.          ,0.   ,       0.          ,1.        ]
        ])


        self.need_orientation_detection = False
        self.orientation_detection_service = self.create_service(
            Trigger, 'trigger_orientation_detection', self.trigger_orientation_detection_callback)

    def get_lego_orientation_type(self, class_name):
        class_name_lower = class_name.lower()
        if 'front' in class_name_lower:
            return 0, 'Front'
        elif 'side_down' in class_name_lower:
            return 1, 'Side_Down'
        elif 'side_top' in class_name_lower:
            return 2, 'Side_Top'
        elif 'back' in class_name_lower:
            return 3, 'Back'
        else:
            return -1, 'Unknown'

    def get_bbox_coordinates_with_angle_and_face(self, results):
        bbox_data = []
        for result in results:
            obb = result.obb
            self.get_logger().info(f'obb content: {obb}')
            if obb is not None and len(obb.xyxyxyxyn) > 0:
                for i in range(len(obb.xyxyxyxyn)):
                    x_center = obb.xywhr[i, 0].item()
                    y_center = obb.xywhr[i, 1].item()
                    z = obb.xywhr[i, 2].item()
                    angle = obb.xywhr[i, 4].item()
                    cls = int(obb.cls[i].item())
                    conf = obb.conf[i].item()
                    class_name = result.names[cls]
                    face, orientation = self.get_lego_orientation_type(class_name)
                    self.get_logger().info(f'Detected class: {class_name}, Face orientation: {orientation}')
                    bbox_data.append((x_center, y_center, z, angle, face, conf, orientation, class_name))
        return bbox_data

    def angle_to_quaternion(self, angle):
        angle = np.clip(angle, 0, 180)
        direction = np.array([0, 0, -1])
        base_direction = np.array([0, 0, 1])
        rotation_axis = np.array([1, 0, 0])
        rotation_angle = np.radians(angle)
        rot_vector = rotation_axis * rotation_angle
        rotation = R.from_rotvec(rot_vector)
        base_to_fixed = R.from_rotvec(np.array([np.pi, 0, 0]))
        final_rotation = base_to_fixed * rotation
        return final_rotation.as_quat()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        try:
            current_frame = self.br.imgmsg_to_cv2(data)

            height, width, _ = current_frame.shape
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.rectangle(mask, (int(width * 0.0), int(height * 0.0)), (int(width * 1.0), int(height * 1.0)), 255, -1)
            masked_frame = cv2.bitwise_and(current_frame, current_frame, mask=mask)

            self.get_logger().info(f'Current model: {self.current_model}')
            results = self.current_model(masked_frame)
            self.get_logger().info(f'Detection results: {results}')
            
            try:
                annotated_frame = results[0].plot()
                annotated_image_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.publisher_.publish(annotated_image_msg)
                self.get_logger().info('Published annotated frame')

                if self.current_model == self.yolo_model:
                    bbox_data = self.get_bbox_coordinates_with_angle_and_face(results)
                    self.process_yolo_results(bbox_data)
                elif self.current_model == self.orientation_model:
                    orientation_type = self.get_orientation_from_results(results)
                    self.process_orientation_results(orientation_type)
            except AttributeError as e:
                self.get_logger().error(f'Error processing results: {e}')
                self.get_logger().info(f'Result attributes: {dir(results[0])}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_yolo_results(self, bbox_data):
        detections = []
        if bbox_data:
            # Sort bboxes by distance to (0,0)
            bbox_data.sort(key=lambda x: math.sqrt(x[0]**2 + x[1]**2))
            
            nearest_bbox = bbox_data[0]
            self.get_logger().info(f'Nearest Bbox data: {nearest_bbox}')
            self.coordinates_history.append(nearest_bbox[:3])
            self.orientation_history.append(nearest_bbox[3])
            self.face_history.append(nearest_bbox[4])

            if len(self.coordinates_history) == 10:
                self.publish_average_data()

            for bbox in bbox_data:
                detection = {
                    'class': bbox[7],
                    'orientation': bbox[6],
                    'confidence': round(bbox[5], 3),
                    'x': bbox[0],
                    'y': bbox[1],
                    'z': bbox[2],
                    'angle': bbox[3],
                    'distance_to_edge': math.sqrt(bbox[0]**2 + bbox[1]**2)
                }
                detections.append(detection)

        # Publish all detections, sorted by distance to edge
        detections.sort(key=lambda x: x['distance_to_edge'])
        detections_msg = String()
        detections_msg.data = json.dumps(detections)
        self.detection_publisher.publish(detections_msg)
        self.get_logger().info(f'Published detections: {detections_msg.data}')

    def publish_average_data(self):
        average_x = sum(coord[0] for coord in self.coordinates_history) / 10
        average_y = sum(coord[1] for coord in self.coordinates_history) / 10
        average_z = sum(coord[2] for coord in self.coordinates_history) / 10
        average_angle = sum(self.orientation_history) / 10

        point_cam = np.array([average_x, average_y, 0, 1])
        point_base = self.calibration_matrix @ point_cam
        x_base, y_base, z_base = point_base[:3]

        coordinate_msg = Point()
        coordinate_msg.x = -0.001*(float(x_base))
        coordinate_msg.y =-0.001*(float(y_base))
        coordinate_msg.z = float(z_base)

        self.coordinate_publisher.publish(coordinate_msg)
        self.get_logger().info(f'Published coordinates: x={coordinate_msg.x}, y={coordinate_msg.y}, z={coordinate_msg.z}')

        # Publish raw angle (radians)
        angle_msg = Float32()
        angle_msg.data = float(average_angle)
        self.angle_publisher.publish(angle_msg)
        self.get_logger().info(f'Published rotation angle: {angle_msg.data} radians')

        # Publish angle in degrees
        angle_degrees = math.degrees(average_angle)
        angle_msg_degrees = Float32()
        angle_msg_degrees.data = float(angle_degrees)
        self.angle_publisher.publish(angle_msg_degrees)
        self.get_logger().info(f'Published rotation angle: {angle_msg_degrees.data} degrees')

        # 調整角度並計算四元數
        adjusted_angle = average_angle + np.pi/2
        quat = quaternion_from_euler(0, 0, adjusted_angle)
        orientation_msg = Quaternion()
        orientation_msg.x = float(quat[2])
        orientation_msg.y = float(quat[3])
        orientation_msg.z = float(quat[1])
        orientation_msg.w = float(quat[0])

        self.orientation_publisher.publish(orientation_msg)
        self.get_logger().info(f'Published orientation: x={orientation_msg.x}, y={orientation_msg.y}, z={orientation_msg.z}, w={orientation_msg.w}')

        face_counts = {0: 0, 1: 0, 2: 0, 3: 0, -1: 0}
        for face in self.face_history:
            if face in face_counts:
                face_counts[face] += 1

        self.get_logger().info(f'Face counts: {face_counts}')
        most_common_face = max(face_counts, key=face_counts.get)
        face_msg = Int8()
        face_msg.data = most_common_face
        self.face_publisher.publish(face_msg)
        self.get_logger().info(f'Published face orientation: {most_common_face}')

        self.coordinates_history.clear()
        self.orientation_history.clear()
        self.face_history.clear()

    def get_orientation_from_results(self, results):
        self.get_logger().info(f'Orientation results: {results}')
        self.get_logger().info(f'Result attributes: {dir(results[0])}')
        if results and len(results) > 0 and hasattr(results[0], 'obb'):
            obb = results[0].obb
            if obb and hasattr(obb, 'cls') and len(obb.cls) > 0:
                orientation_type = int(obb.cls[0].item())
                return orientation_type
        return -1

    def process_orientation_results(self, orientation_type):
        face_msg = Int8()
        face_msg.data = orientation_type
        self.face_publisher.publish(face_msg)
        self.get_logger().info(f'Published orientation type: {orientation_type}')

    def trigger_orientation_detection_callback(self, request, response):
        self.need_orientation_detection = True
        self.current_model = self.orientation_model
        self.get_logger().info("Switched to orientation model")
        response.success = True
        response.message = "Orientation detection triggered"
        return response

def main(args=None):
    rclpy.init(args=args)
    enhanced_zed_yolo = EnhancedZEDYOLO()
    rclpy.spin(enhanced_zed_yolo)
    enhanced_zed_yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''








































































'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import random

class ParticleFilter:
    def __init__(self, num_particles=100, init_range=(0, 1280, 0, 720)):
        self.num_particles = num_particles
        x_min, x_max, y_min, y_max = init_range
        self.particles = np.array([
            [random.uniform(x_min, x_max), random.uniform(y_min, y_max), 1.0 / num_particles]
            for _ in range(num_particles)
        ])

    def predict(self, motion=(0, 0), noise_std=(10, 10)):
        dx, dy = motion
        x_noise, y_noise = noise_std
        for i in range(self.num_particles):
            self.particles[i, 0] += dx + np.random.normal(0, x_noise)
            self.particles[i, 1] += dy + np.random.normal(0, y_noise)

    def update(self, measurement, measurement_noise=20):
        if measurement is None:
            return
        x_meas, y_meas = measurement
        total_weight = 0.0
        for i in range(self.num_particles):
            distance = np.sqrt((self.particles[i, 0] - x_meas) ** 2 + (self.particles[i, 1] - y_meas) ** 2)
            self.particles[i, 2] = np.exp(- (distance ** 2) / (2 * measurement_noise ** 2))
            total_weight += self.particles[i, 2]
        if total_weight > 0:
            self.particles[:, 2] /= total_weight

    def resample(self):
        weights = self.particles[:, 2]
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=weights)
        self.particles = self.particles[indices]
        self.particles[:, 2] = 1.0 / self.num_particles

    def get_estimate(self):
        x_estimate = np.sum(self.particles[:, 0] * self.particles[:, 2])
        y_estimate = np.sum(self.particles[:, 1] * self.particles[:, 2])
        return int(x_estimate), int(y_estimate)

class BlobDetectorNode(Node):
    def __init__(self):
        super().__init__('blob_detector_node')
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.position_publisher = self.create_publisher(Point, '/circle_position', 10)
        self.bridge = CvBridge()
        self.depth_image = None

        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 50
        self.params.maxThreshold = 255
        self.params.thresholdStep = 10
        self.params.filterByArea = True
        self.params.minArea = 250
        self.params.maxArea = 1000
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.1
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.2
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.2
        self.params.blobColor = 0
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        self.particle_filter = ParticleFilter(num_particles=500)
        self.calibration_matrix = np.array([
        # 架高
        [-4.49064911e-04,  1.45529593e-04 , 2.17262369e-01 , 2.51199743e-01],
        [-6.23688653e-05, -3.53414545e-05 , 2.18293567e+00 ,-8.55039463e-01],
        [-1.14586617e-17 ,-1.70160449e-17 , 1.60985808e-13 , 2.38000005e-01],
        [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]

        # 底部
        # [-2.51585288e-04  ,4.48262939e-06 , 3.25196588e-02 , 1.73901913e-01],
        # [ 3.85851210e-06  ,2.78814743e-04, -1.20921590e-01 ,-6.46639496e-01],
        # [-2.16840434e-19, -2.43945489e-19,  3.95693018e-02 , 2.11600537e-01],
        # [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]

        # 上部
        # [ 4.36085524e-04 ,-2.63080284e-05 , 2.72170137e-02 , 1.03094749e-01],
        # [-3.61671077e-05 ,-4.37109365e-04, -6.83351542e-02 ,-2.58845283e-01],
        # [ 8.40256684e-19 , 1.95156391e-18 , 1.23399545e-01 , 4.67422520e-01],
        # [ 0.00000000e+00  ,0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]
        ])

    def transform_coordinates(self, x, y):
            point_cam = np.array([x, y, 0.264, 1])
            transformed = np.zeros(4)
            for i in range(4):
                transformed[i] = np.dot(self.calibration_matrix[i], point_cam)
            return transformed[0], transformed[1], transformed[2]

    def publish_position(self, x, y, diameter):
        depth = self.get_depth(x, y)
        if depth is None:
            depth = 0.0
        x_transformed, y_transformed, _ = self.transform_coordinates(x, y)
        msg = Point()
        msg.x = float(x_transformed)
        msg.y = float(y_transformed)
        msg.z = float(depth)
        self.position_publisher.publish(msg)
        self.get_logger().info(f'Published position and status: ({msg.x}, {msg.y}, {msg.z})')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # 遮罩深度圖像的下半部
            self.depth_image = self.mask_lower_part(self.depth_image)
        except Exception as e:
            self.get_logger().error(f'深度影像轉換錯誤: {str(e)}')

    def get_depth(self, x, y):
        if self.depth_image is None:
            return None
        if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
            depth = self.depth_image[y, x]
            if depth == 0:
                return None
            depth_meters = depth / 1000.0
            return depth_meters
        return None

    def mask_lower_part(self, image):
        # 獲取圖像尺寸
        height, width = image.shape[:2]
        # 計算遮罩起始點，遮住下方 1/4
        mask_start = height * 3 // 4  # 從 3/4 高度開始遮罩
        # 複製圖像以避免修改原始數據
        masked_image = image.copy()
        # 檢查圖像通道數
        if len(masked_image.shape) == 3:  # 3通道圖像 (BGR)
            masked_image[mask_start:, :, :] = 0  # 設為 0
        else:  # 單通道圖像 (灰度或深度圖)
            masked_image[mask_start:, :] = 0  # 設為 0
        return masked_image

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        edges = cv2.Canny(thresh, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        connected_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        return connected_edges

    def image_callback(self, msg):
        try:
            # 獲取原始圖像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 複製一份用於顯示
            display_image = cv_image.copy()
            # 遮罩下半部用於處理
            cv_image = self.mask_lower_part(cv_image)
            # 處理遮罩後的圖像
            processed_image = self.preprocess_image(cv_image)
            keypoints = self.detector.detect(processed_image)

            if not keypoints:
                msg = Point()
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                self.position_publisher.publish(msg)
                self.get_logger().info('No target detected: (0.0, 0.0, 0.0)')
                measurement = None
            else:
                image_height, image_width = cv_image.shape[:2]
                selected_keypoint = None
                max_score = -float('inf')

                for keypoint in keypoints:
                    x, y = keypoint.pt[0], keypoint.pt[1]
                    # 確保只考慮遮罩範圍內的關鍵點（y < mask_start）
                    mask_start = image_height * 3 // 4
                    if y >= mask_start:  # 如果關鍵點在遮罩區域內，忽略
                        continue
                    score = x + y
                    if score > max_score:
                        max_score = score
                        selected_keypoint = keypoint

                if selected_keypoint:
                    x, y = int(selected_keypoint.pt[0]), int(selected_keypoint.pt[1])
                    radius = int(selected_keypoint.size / 2)
                    # 在顯示圖像上繪製
                    cv2.circle(display_image, (x, y), radius, (0, 0, 255), 2)
                    self.publish_position(x, y, selected_keypoint.size)
                    measurement = (x, y)
                else:
                    measurement = None

            if measurement is not None:
                self.particle_filter.update(measurement)
                self.particle_filter.resample()
            self.particle_filter.predict(motion=(0, 0))

            # 只在可視區域內繪製粒子
            mask_start = display_image.shape[0] * 3 // 4
            for particle in self.particle_filter.particles:
                x, y = int(particle[0]), int(particle[1])
                if y < mask_start:  # 只繪製遮罩範圍外的粒子
                    cv2.circle(display_image, (x, y), 1, (255, 0, 0), -1)
            
            x_est, y_est = self.particle_filter.get_estimate()
            depth = self.get_depth(x_est, y_est)
            
            if y_est < mask_start:  # 只在遮罩範圍外繪製估計點
                cv2.circle(display_image, (x_est, y_est), 10, (0, 255, 0), 2)
                coord_text = f'Est: ({x_est}, {y_est})'
                depth_text = f'Depth: {depth:.3f}m' if depth is not None else 'Depth: N/A'
                cv2.putText(display_image, coord_text, (x_est + 10, y_est),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(display_image, depth_text, (x_est + 10, y_est + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 顯示圖像（使用未遮罩的 display_image）
            cv2.imshow("Original", display_image)
            cv2.imshow("Processed", processed_image)
            if self.depth_image is not None:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_colormap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BlobDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

'''