#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import cv2
import numpy as np
import random
import time

class ParticleFilter:
    def __init__(self, num_particles=100, init_range=(0, 640, 0, 480)):
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
    def __init__(self, node_name, pipeline, camera_id, calibration_matrix):
        super().__init__(node_name)
        self.pipeline = pipeline
        self.camera_id = camera_id
        self.depth_image = None
        self.position_publisher = self.create_publisher(Point, f'/circle_position/camera{self.camera_id}', 10)

        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 230
        self.params.maxThreshold = 255
        self.params.thresholdStep = 10
        self.params.filterByArea = True
        self.params.minArea = 1200
        self.params.maxArea = 2000
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.3
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.01
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.01
        self.params.blobColor = 0
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        self.particle_filter = ParticleFilter(num_particles=500, init_range=(0, 640, 0, 480))
        self.calibration_matrix = calibration_matrix

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
        self.get_logger().info(f'Camera {self.camera_id} Published: ({msg.x}, {msg.y}, {msg.z})')

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

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            self.depth_image = np.asanyarray(depth_frame.get_data())
            display_image = color_image.copy()
            masked_image = self.mask_lower_part(color_image)

            processed_image = self.preprocess_image(masked_image)
            keypoints = self.detector.detect(processed_image)

            measurement = None
            if keypoints:
                image_height, _ = color_image.shape[:2]
                selected_keypoint = None
                max_score = -float('inf')
                mask_start = image_height * 3 // 4

                for keypoint in keypoints:
                    x, y = keypoint.pt[0], keypoint.pt[1]
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
                    self.publish_position(x, y, selected_keypoint.size)
                    measurement = (x, y)

            if measurement is not None:
                self.particle_filter.update(measurement)
                self.particle_filter.resample()
            self.particle_filter.predict(motion=(0, 0))

            mask_start = display_image.shape[0] * 3 // 4
            for particle in self.particle_filter.particles:
                x, y = int(particle[0]), int(particle[1])
                if y < mask_start:
                    cv2.circle(display_image, (x, y), 1, (255, 0, 0), -1)

            x_est, y_est = self.particle_filter.get_estimate()
            depth = self.get_depth(x_est, y_est)
            if y_est < mask_start:
                cv2.circle(display_image, (x_est, y_est), 10, (0, 255, 0), 2)
                coord_text = f'Est: ({x_est}, {y_est})'
                depth_text = f'Depth: {depth:.3f}m' if depth is not None else 'Depth: N/A'
                cv2.putText(display_image, coord_text, (x_est + 10, y_est),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(display_image, depth_text, (x_est + 10, y_est + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow(f'Camera {self.camera_id} Original', display_image)
            cv2.moveWindow(f'Camera {self.camera_id} Original', (self.camera_id - 1) * 650, 0)

        except Exception as e:
            self.get_logger().error(f'Camera {self.camera_id} Error: {str(e)}')

def main():
    rclpy.init()
    serials = ['140122074462', '844212070157']
    pipelines = []
    nodes = []

    calibration_matrix = np.array([
        [-4.49064911e-04, 1.45529593e-04, 2.17262369e-01, 2.51199743e-01],
        [-6.23688653e-05, -3.53414545e-05, 2.18293567e+00, -8.55039463e-01],
        [-1.14586617e-17, -1.70160449e-17, 1.60985808e-13, 2.38000005e-01],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])

    for i, serial in enumerate(serials, 1):
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            pipeline.start(config)
            pipelines.append(pipeline)
            node = BlobDetectorNode(f'blob_detector_node_camera{i}', pipeline, i, calibration_matrix)
            nodes.append(node)
            print(f"Camera {serial} initialized.")
        except RuntimeError as e:
            print(f"Failed to initialize camera {serial}: {e}")
            continue

    if not pipelines:
        print("No cameras available. Exiting.")
        rclpy.shutdown()
        return

    try:
        start_time = time.time()
        frame_count = 0
        while True:
            for node in nodes:
                node.process_frame()
                rclpy.spin_once(node, timeout_sec=0.01)

            frame_count += 1
            if frame_count % 30 == 0:
                fps = frame_count / (time.time() - start_time)
                print(f"FPS: {fps:.2f}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        for pipeline in pipelines:
            pipeline.stop()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("All resources released.")

if __name__ == '__main__':
    main()