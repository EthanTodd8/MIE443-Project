#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO
from typing import List, Tuple, Optional


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.5
        
        # Confidence range for filtering (min, max)
        self.confidence_range = (0.0, 1.0)
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def filter_by_confidence_range(
        self, 
        boxes, 
        min_confidence: float = 0.0, 
        max_confidence: float = 1.0
    ) -> np.ndarray:
        """
        Filter detections by confidence range.
        
        Args:
            boxes: YOLO boxes object containing detections
            min_confidence: Minimum confidence threshold
            max_confidence: Maximum confidence threshold
            
        Returns:
            Filtered boxes within confidence range
        """
        confidences = boxes.conf.cpu().numpy()
        mask = (confidences >= min_confidence) & (confidences <= max_confidence)
        
        # Filter boxes by confidence mask
        filtered_boxes = boxes[mask]
        
        return filtered_boxes

    def get_highest_confidence_detection(
        self, 
        boxes
    ) -> Optional[Tuple[int, str, float, Tuple[int, int, int, int]]]:
        """
        Get the detection with the highest confidence.
        
        Args:
            boxes: YOLO boxes object containing detections
            
        Returns:
            Tuple of (class_id, class_name, confidence, bbox) or None if no detections
        """
        if len(boxes) == 0:
            return None
        
        # Get indices of boxes sorted by confidence (descending)
        confidences = boxes.conf.cpu().numpy()
        highest_idx = np.argmax(confidences)
        
        highest_box = boxes[highest_idx]
        
        # Extract detection information
        class_id = int(highest_box.cls.cpu().numpy())
        class_name = self.model.names[class_id]
        confidence = float(highest_box.conf.cpu().numpy())
        
        # Extract bounding box coordinates (x1, y1, x2, y2)
        bbox = highest_box.xyxy.cpu().numpy()[0].astype(int)
        
        return class_id, class_name, confidence, bbox

    def display_bounding_boxes(
        self, 
        image: np.ndarray, 
        boxes, 
        thickness: int = 2,
        font_scale: float = 0.6
    ) -> np.ndarray:
        """
        Draw bounding boxes on image with confidence levels.
        
        Args:
            image: Input image (numpy array)
            boxes: YOLO boxes object containing detections
            thickness: Thickness of bounding box lines
            font_scale: Font scale for text
            
        Returns:
            Image with drawn bounding boxes
        """
        annotated_image = image.copy()
        
        # Define color map for confidence levels
        # Red (0-0.5), Yellow (0.5-0.7), Green (0.7-1.0)
        def get_color_by_confidence(confidence: float) -> Tuple[int, int, int]:
            if confidence < 0.5:
                # Red
                return (0, 0, 255)
            elif confidence < 0.7:
                # Yellow
                return (0, 255, 255)
            else:
                # Green
                return (0, 255, 0)
        
        for box in boxes:
            # Get box coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            
            # Get class and confidence
            class_id = int(box.cls.cpu().numpy())
            class_name = self.model.names[class_id]
            confidence = float(box.conf.cpu().numpy())
            
            # Get color based on confidence
            color = get_color_by_confidence(confidence)
            
            # Draw bounding box
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, thickness)
            
            # Prepare label text
            label = f"{class_name}: {confidence:.2f}"
            
            # Get text size for background
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)[0]
            
            # Draw background rectangle for text
            cv2.rectangle(
                annotated_image,
                (x1, y1 - text_size[1] - 8),
                (x1 + text_size[0] + 8, y1),
                color,
                -1
            )
            
            # Put text on image
            cv2.putText(
                annotated_image,
                label,
                (x1 + 4, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),
                1
            )
        
        return annotated_image

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        save_detected_image = request.save_detected_image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
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

        # Filter detections by confidence range
        filtered_boxes = self.filter_by_confidence_range(
            boxes,
            min_confidence=self.confidence_threshold,
            max_confidence=1.0
        )
        
        # Get highest confidence detection
        detection = self.get_highest_confidence_detection(filtered_boxes)
        
        if detection is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = f"No detections found above confidence threshold {self.confidence_threshold}"
            return response
        
        class_id, class_name, confidence, bbox = detection
        
        # Display bounding boxes on image if requested
        if save_detected_image:
            annotated_image = self.display_bounding_boxes(image, filtered_boxes)
            cv2.imwrite('detected_objects.jpg', annotated_image)
            self.get_logger().info(f'Saved annotated image: detected_objects.jpg')
        
        # Populate response
        response.success = True
        response.class_id = class_id
        response.class_name = class_name  # Return class as string
        response.confidence = confidence
        response.message = f"Detected {class_name} with confidence {confidence:.4f}"
        
        self.get_logger().info(
            f'Detection: class_id={class_id}, class_name={class_name}, '
            f'confidence={confidence:.4f}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()