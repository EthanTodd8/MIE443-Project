#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

from urllib import response

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.5

        # top 5 results
        self.result1 = None
        self.result2 = None
        self.result3 = None
        self.result4 = None
        self.result5 = None
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        save_detected_image = request.save_detected_image
        image1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = cv2.flip(image1, 0)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response
        
        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes
        
        if boxes is None or len(boxes) == 0:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No objects detected"
            return response
        
        
        ##filter by confidence threshold
        # save the top 5 most confident detections
        self.result1 = None
        self.result2 = None
        self.result3 = None
        self.result4 = None
        self.result5 = None

        mask = boxes.conf >= self.confidence_threshold
        if not mask.any():
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = f"No detections found above confidence threshold {self.confidence_threshold}"
            
            return response
        
        # Get top 5 highest confidence detections
        # Sort boxes by confidence in descending order
        sorted_indices = boxes.conf.argsort(descending=True)
        
        # Get up to top 5 detections
        top_k = min(5, len(sorted_indices))
        top_indices = sorted_indices[:top_k]
        
        # Store top 5 results
        results_list = []
        for i, idx in enumerate(top_indices):
            class_id = int(boxes.cls[idx])
            confidence = float(boxes.conf[idx])
            class_name = self.model.names[class_id]
            bbox = boxes.xyxy[idx].cpu().numpy()
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            detection_info = {
                'class_id': class_id,
                'class_name': class_name,
                'confidence': confidence,
                'bbox': (x1, y1, x2, y2),
                'center': (center_x, center_y)
            }
            results_list.append(detection_info)
            
            # Store in numbered result attributes
            if i == 0:
                self.result1 = detection_info
            elif i == 1:
                self.result2 = detection_info
            elif i == 2:
                self.result3 = detection_info
            elif i == 3:
                self.result4 = detection_info
            elif i == 4:
                self.result5 = detection_info
        
        # Target classes to prioritize
        target_classes = {'cup', 'bottle', 'clock', 'motorcycle', 'plant'}
        
        # Check if any of the top 5 detections are target classes
        best_detection = results_list[0]  # default to highest confidence
        for detection in results_list:
            if detection['class_name'].lower() in target_classes:
                best_detection = detection
                save_detected_image = True  # Force save if target class found
                self.get_logger().info(f"Target class found: {detection['class_name']}")
                break
        
        response.success = True
        response.class_id = best_detection['class_id']
        response.class_name = best_detection['class_name']
        response.confidence = best_detection['confidence']
        response.message = f"Top 5 detections - Best: {best_detection['class_name']} ({best_detection['confidence']:.4f})"
        
        if save_detected_image:
            filename = f"/home/turtlebot/ros2_ws/yoloDetectedImages/yolo_detections_{best_detection['class_name']}_.jpg"
            annotated_image = results[0].plot()  # Get annotated image with bounding boxes
            cv2.imwrite(filename, annotated_image)
            self.get_logger().info(f'Saved annotated image: {filename}')
        
        # Log all top 5 detections
        self.get_logger().info("=== Top 5 Detections ===")
        for i, det in enumerate(results_list, 1):
            self.get_logger().info(f"  {i}. {det['class_name']} - Confidence: {det['confidence']:.4f}")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()