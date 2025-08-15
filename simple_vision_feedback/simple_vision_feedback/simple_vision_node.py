#!/usr/bin/env python3
"""
簡化版視覺反饋節點 - 使用基本圖像處理替代LLM
適用於螺絲組裝品質檢測的原型系統
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from typing import Dict, Any, Tuple, Optional

class SimpleVisionFeedbackNode(Node):
    def __init__(self):
        super().__init__('simple_vision_feedback_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        self.current_image = None
        
        # 訂閱影像
        self.image_sub = self.create_subscription(
            Image, '/zed_camera/image_left', self.image_callback, 10)
        
        # 發布反饋結果
        self.feedback_pub = self.create_publisher(String, '/assembly_feedback', 10)
        self.quality_pub = self.create_publisher(Bool, '/assembly_quality_ok', 10)
        self.score_pub = self.create_publisher(Point, '/assembly_quality_score', 10)
        
        # 服務
        self.analysis_service = self.create_service(
            Trigger, '/analyze_assembly', self.analyze_assembly_callback)
        self.quality_check_service = self.create_service(
            Trigger, '/quality_check', self.quality_check_callback)
        
        # 除錯參數
        self.save_debug_images = True
        self.show_debug_window = True
        self.debug_counter = 0
        
        # 檢測參數
        self.setup_detection_parameters()
        
        self.get_logger().info("Simple Vision Feedback Node started (No LLM)")
    
    def setup_detection_parameters(self):
        """設定檢測參數"""
        # 圓形檢測參數（用於螺絲頭和螺絲孔）
        self.circle_params = {
            'dp': 1,
            'min_dist': 50,
            'param1': 50,
            'param2': 30,
            'min_radius': 10,
            'max_radius': 100
        }
        
        # 品質評估閾值
        self.quality_thresholds = {
            'min_circularity': 0.7,
            'max_tilt_ratio': 0.15,
            'min_contrast': 30,
            'alignment_tolerance': 20
        }
    
    def image_callback(self, msg):
        """接收影像"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"影像轉換失敗: {e}")
    
    def analyze_assembly_callback(self, request, response):
        """組裝分析服務"""
        if self.current_image is None:
            response.success = False
            response.message = "沒有可用的影像"
            return response
        
        try:
            # 執行分析
            analysis_result = self.analyze_screw_assembly()
            
            # 發布結果
            self.publish_results(analysis_result)
            
            # 設定回應
            response.success = analysis_result['assembly_success']
            response.message = f"分析完成 - 品質分數: {analysis_result['quality_score']:.2f}"
            
            self.get_logger().info(f"組裝分析結果: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"分析失敗: {e}"
            self.get_logger().error(response.message)
        
        return response
    
    def quality_check_callback(self, request, response):
        """品質檢查服務"""
        if self.current_image is None:
            response.success = False
            response.message = "沒有可用的影像"
            return response
        
        try:
            analysis_result = self.quick_quality_check()
            self.publish_results(analysis_result)
            
            response.success = True
            response.message = f"品質檢查完成 - 分數: {analysis_result['quality_score']:.2f}"
            
        except Exception as e:
            response.success = False
            response.message = f"品質檢查失敗: {e}"
        
        return response
    
    def analyze_screw_assembly(self) -> Dict[str, Any]:
        """分析螺絲組裝狀態"""
        image = self.current_image.copy()
        
        # 1. 預處理
        processed_image = self.preprocess_image(image)
        
        # 2. 檢測圓形（螺絲頭）
        circles = self.detect_circles(processed_image)
        
        # 3. 分析檢測結果
        analysis = self.evaluate_assembly_quality(image, circles)
        
        # 4. 生成反饋
        feedback = self.generate_feedback(analysis)
        
        # 5. 保存除錯圖像
        if self.save_debug_images:
            self.save_debug_image(image, circles, analysis)
        
        return feedback
    
    def quick_quality_check(self) -> Dict[str, Any]:
        """快速品質檢查"""
        image = self.current_image.copy()
        
        # 簡化的品質指標
        brightness = np.mean(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        contrast = np.std(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        
        # 邊緣檢測強度
        edges = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 50, 150)
        edge_density = np.sum(edges > 0) / edges.size
        
        # 計算品質分數
        quality_score = min(1.0, (brightness / 128.0 + contrast / 64.0 + edge_density * 10) / 3)
        
        return {
            'screw_detected': edge_density > 0.05,
            'screw_inserted': quality_score > 0.6,
            'screw_tilted': False,  # 簡化處理
            'screw_flush': quality_score > 0.7,
            'assembly_success': quality_score > 0.6,
            'quality_score': quality_score,
            'confidence': 0.8,
            'brightness': brightness,
            'contrast': contrast,
            'edge_density': edge_density,
            'analysis_type': 'quick_check',
            'timestamp': time.time()
        }
    
    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """影像預處理"""
        # 轉灰階
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # 直方圖均衡化
        equalized = cv2.equalizeHist(blurred)
        
        return equalized
    
    def detect_circles(self, processed_image: np.ndarray) -> Optional[np.ndarray]:
        """檢測圓形（螺絲頭）"""
        circles = cv2.HoughCircles(
            processed_image,
            cv2.HOUGH_GRADIENT,
            dp=self.circle_params['dp'],
            minDist=self.circle_params['min_dist'],
            param1=self.circle_params['param1'],
            param2=self.circle_params['param2'],
            minRadius=self.circle_params['min_radius'],
            maxRadius=self.circle_params['max_radius']
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            self.get_logger().info(f"檢測到 {len(circles)} 個圓形")
        else:
            self.get_logger().info("未檢測到圓形")
        
        return circles
    
    def evaluate_assembly_quality(self, image: np.ndarray, circles: Optional[np.ndarray]) -> Dict[str, Any]:
        """評估組裝品質"""
        analysis = {
            'screw_count': 0,
            'average_circularity': 0.0,
            'tilt_detected': False,
            'alignment_score': 0.0,
            'contrast_score': 0.0
        }
        
        if circles is None or len(circles) == 0:
            return analysis
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        analysis['screw_count'] = len(circles)
        
        circularities = []
        for (x, y, r) in circles:
            # 計算圓形度
            circularity = self.calculate_circularity(gray, x, y, r)
            circularities.append(circularity)
            
            # 檢測傾斜
            if self.detect_tilt(gray, x, y, r):
                analysis['tilt_detected'] = True
        
        if circularities:
            analysis['average_circularity'] = np.mean(circularities)
        
        # 計算對比度分數
        analysis['contrast_score'] = np.std(gray) / 64.0  # 標準化到0-4範圍
        
        # 計算對準分數（基於圓心位置的一致性）
        if len(circles) > 1:
            centers = circles[:, :2]
            center_std = np.std(centers, axis=0)
            analysis['alignment_score'] = max(0, 1 - np.mean(center_std) / 50.0)
        else:
            analysis['alignment_score'] = 1.0
        
        return analysis
    
    def calculate_circularity(self, gray: np.ndarray, x: int, y: int, r: int) -> float:
        """計算圓形度"""
        try:
            # 創建圓形遮罩
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.circle(mask, (x, y), r, 255, -1)
            
            # 在遮罩區域進行邊緣檢測
            edges = cv2.Canny(gray, 50, 150)
            masked_edges = cv2.bitwise_and(edges, mask)
            
            # 計算邊緣像素數
            edge_pixels = np.sum(masked_edges > 0)
            expected_pixels = 2 * np.pi * r  # 理論圓周像素數
            
            if expected_pixels > 0:
                circularity = min(1.0, edge_pixels / expected_pixels)
            else:
                circularity = 0.0
            
            return circularity
        except:
            return 0.0
    
    def detect_tilt(self, gray: np.ndarray, x: int, y: int, r: int) -> bool:
        """檢測螺絲是否傾斜"""
        try:
            # 提取螺絲頭區域
            roi = gray[max(0, y-r):min(gray.shape[0], y+r), 
                      max(0, x-r):min(gray.shape[1], x+r)]
            
            if roi.size == 0:
                return False
            
            # 計算強度分布的不對稱性
            center_y, center_x = roi.shape[0] // 2, roi.shape[1] // 2
            
            # 分析上下半部的強度差異
            if center_y > 0:
                upper_half = roi[:center_y, :]
                lower_half = roi[center_y:, :]
                
                upper_mean = np.mean(upper_half)
                lower_mean = np.mean(lower_half)
                
                intensity_ratio = abs(upper_mean - lower_mean) / max(upper_mean, lower_mean, 1)
                
                return intensity_ratio > self.quality_thresholds['max_tilt_ratio']
            
            return False
        except:
            return False
    
    def generate_feedback(self, analysis: Dict[str, Any]) -> Dict[str, Any]:
        """生成反饋結果"""
        screw_detected = analysis['screw_count'] > 0
        screw_inserted = screw_detected and analysis['average_circularity'] > self.quality_thresholds['min_circularity']
        screw_tilted = analysis['tilt_detected']
        screw_flush = screw_inserted and not screw_tilted and analysis['contrast_score'] > 0.5
        
        # 計算綜合品質分數
        quality_components = [
            analysis['average_circularity'],
            analysis['alignment_score'],
            min(1.0, analysis['contrast_score']),
            1.0 if screw_detected else 0.0,
            0.0 if screw_tilted else 1.0
        ]
        quality_score = np.mean(quality_components)
        
        assembly_success = (screw_inserted and not screw_tilted and 
                          quality_score > 0.6 and analysis['screw_count'] == 1)
        
        # 生成問題和建議
        issues = []
        suggestions = []
        
        if not screw_detected:
            issues.append("未檢測到螺絲")
            suggestions.append("檢查螺絲位置和照明")
        elif analysis['screw_count'] > 1:
            issues.append(f"檢測到多個螺絲 ({analysis['screw_count']}個)")
            suggestions.append("確認只有一個螺絲在視野中")
        
        if screw_tilted:
            issues.append("螺絲可能傾斜")
            suggestions.append("重新調整螺絲角度")
        
        if analysis['average_circularity'] < self.quality_thresholds['min_circularity']:
            issues.append("螺絲頭形狀不規則")
            suggestions.append("檢查螺絲是否損壞")
        
        if analysis['contrast_score'] < 0.3:
            issues.append("影像對比度不足")
            suggestions.append("改善照明條件")
        
        if not issues:
            issues.append("未發現明顯問題")
            suggestions.append("組裝品質良好")
        
        return {
            'screw_detected': screw_detected,
            'screw_inserted': screw_inserted,
            'screw_tilted': screw_tilted,
            'screw_flush': screw_flush,
            'assembly_success': assembly_success,
            'quality_score': float(quality_score),
            'confidence': 0.8,  # 固定信心度
            'screw_count': analysis['screw_count'],
            'circularity': float(analysis['average_circularity']),
            'alignment_score': float(analysis['alignment_score']),
            'contrast_score': float(analysis['contrast_score']),
            'issues': issues,
            'suggestions': suggestions,
            'analysis_method': 'computer_vision',
            'timestamp': time.time()
        }
    
    def save_debug_image(self, image: np.ndarray, circles: Optional[np.ndarray], analysis: Dict[str, Any]):
        """保存除錯圖像"""
        debug_image = image.copy()
        
        # 繪製檢測到的圓形
        if circles is not None:
            for (x, y, r) in circles:
                cv2.circle(debug_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(debug_image, (x, y), 2, (0, 0, 255), 3)
        
        # 添加文字資訊
        info_text = [
            f"螺絲數量: {analysis['screw_count']}",
            f"圓形度: {analysis['average_circularity']:.2f}",
            f"傾斜: {'是' if analysis['tilt_detected'] else '否'}",
            f"對比度: {analysis['contrast_score']:.2f}"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(debug_image, text, (10, 30 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 保存圖像
        filename = f"debug_assembly_{self.debug_counter:04d}.jpg"
        cv2.imwrite(filename, debug_image)
        self.debug_counter += 1
        
        # 顯示視窗（如果啟用）
        if self.show_debug_window:
            cv2.imshow("Assembly Analysis", debug_image)
            cv2.waitKey(1)
        
        self.get_logger().info(f"除錯圖像已保存: {filename}")
    
    def publish_results(self, analysis_result: Dict[str, Any]):
        """發布分析結果"""
        try:
            # 發布完整反饋
            feedback_msg = String()
            feedback_msg.data = json.dumps(analysis_result, indent=2, ensure_ascii=False)
            self.feedback_pub.publish(feedback_msg)
            
            # 發布品質狀態
            quality_msg = Bool()
            quality_msg.data = analysis_result['assembly_success']
            self.quality_pub.publish(quality_msg)
            
            # 發布品質分數
            score_msg = Point()
            score_msg.x = analysis_result['quality_score']
            score_msg.y = analysis_result['confidence']
            score_msg.z = 1.0 if analysis_result['assembly_success'] else 0.0
            self.score_pub.publish(score_msg)
            
            self.get_logger().info(f"結果已發布 - 成功: {analysis_result['assembly_success']}, "
                                 f"分數: {analysis_result['quality_score']:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"發布結果失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVisionFeedbackNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()