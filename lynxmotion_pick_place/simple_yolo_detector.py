#!/usr/bin/env python3
"""
Simple YOLOv8 Detector - Working Version Without Complex ROS Messages
Uses the exact same approach as the working VLM detector
"""

import rclpy
import cv2
import numpy as np
import os
import ast
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class SimpleYOLODetector(Node):
    def __init__(self):
        super().__init__('simple_yolo_detector')
        
        # Initialize parameters
        self.bridge = CvBridge()
        self.confidence_threshold = 0.25
        self.topic_rgb = '/zedm/zed_node/left/image_rect_color'
        
        # Create output directory for saved images
        self.output_dir = "detection_results"
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"📁 Saving detection images to: {os.path.abspath(self.output_dir)}")
        
        # Stats tracking
        self.detection_count = 0
        self.image_count = 0
        
        # Load YOLOv8 model
        self.get_logger().info("🤖 Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')
        
        # Publishers and subscribers
        self.create_subscription(Image, self.topic_rgb, self.on_image, 10)
        self.create_subscription(String, '/task_cmd', self.on_cmd, 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        
        self.get_logger().info(f"🤖 Simple YOLOv8 Detector Ready!")
        self.get_logger().info(f"   Model: YOLOv8n (ultra-fast, 6.2MB)")
        self.get_logger().info(f"   Confidence Threshold: {self.confidence_threshold}")

    def on_cmd(self, msg: String):
        """Handle task commands"""
        self.get_logger().info(f"🎯 Received command: {msg.data}")

    def on_image(self, msg: Image):
        """Process incoming images with YOLOv8"""
        self.image_count += 1
        
        try:
            # Convert ROS image to OpenCV
            img_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLOv8 detection
            results = self.model(img_bgr, conf=self.confidence_threshold, verbose=False)
            
            # Process detections
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            detected_objects = []
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        # Extract detection data
                        bbox = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf[0].cpu().numpy())
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id]
                        
                        x1, y1, x2, y2 = bbox
                        center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                        
                        # Store detection info
                        detected_objects.append({
                            'name': class_name,
                            'confidence': confidence,
                            'center': (center_x, center_y),
                            'bbox': bbox
                        })
                        
                        # Create ROS detection message for 3D integration
                        det = Detection2D()
                        det.bbox.center.x = center_x
                        det.bbox.center.y = center_y
                        det.bbox.size_x = float(x2 - x1)
                        det.bbox.size_y = float(y2 - y1)
                        
                        # Create ObjectHypothesisWithPose - simplified approach
                        hyp = ObjectHypothesisWithPose()
                        # Direct assignment approach - bypass hypothesis subfield for now
                        hyp.id = class_name  # Use id field instead
                        hyp.score = confidence
                        det.results.append(hyp)
                        detection_array.detections.append(det)
            
            # Log detections if any found
            if detected_objects:
                self.detection_count += 1
                
                self.get_logger().info(f"🔍 FRAME {self.image_count} - DETECTED {len(detected_objects)} OBJECTS:")
                
                for i, obj in enumerate(detected_objects):
                    self.get_logger().info(f"   [{i+1}] {obj['name']}")
                    self.get_logger().info(f"       📍 Center: ({obj['center'][0]:.1f}, {obj['center'][1]:.1f})")
                    self.get_logger().info(f"       🎯 Confidence: {obj['confidence']:.3f}")
                
                # Draw and save annotated image
                annotated = img_bgr.copy()
                for obj in detected_objects:
                    x1, y1, x2, y2 = [int(coord) for coord in obj['bbox']]
                    
                    # Draw bounding box
                    color = (0, 255, 0) if obj['confidence'] > 0.7 else (0, 255, 255)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw center point
                    center_x, center_y = [int(coord) for coord in obj['center']]
                    cv2.circle(annotated, (center_x, center_y), 5, (255, 0, 0), -1)
                    
                    # Add label
                    label = f"{obj['name']} ({obj['confidence']:.2f})"
                    cv2.putText(annotated, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Save image
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"simple_yolo_{timestamp}_frame{self.image_count:06d}.jpg"
                filepath = os.path.join(self.output_dir, filename)
                cv2.imwrite(filepath, annotated)
                self.get_logger().info(f"💾 Saved: {filename}")
                
                # Show real-time detection window
                cv2.imshow('YOLOv8 Real-Time Detections', annotated)
                cv2.waitKey(1)  # Required for OpenCV window updates
                
                # Publish detections
                if detection_array.detections:
                    self.pub_det.publish(detection_array)
                    
            else:
                # Show camera feed even when no detections
                cv2.imshow('YOLOv8 Real-Time Detections', img_bgr)
                cv2.waitKey(1)
                
                # Log no detections occasionally  
                if self.image_count % 60 == 0:
                    self.get_logger().info(f"🔍 FRAME {self.image_count} - No objects detected")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")
            import traceback
            traceback.print_exc()

def main():
    rclpy.init()
    
    print("\n" + "="*60)
    print("🚀 SIMPLE YOLOv8 DETECTOR")  
    print("="*60)
    print("✅ Simplified version - should work without ROS message issues")
    print("✅ Enhanced logging with coordinates and confidence scores")
    print("✅ Automatic image saving with bounding boxes")
    print("="*60)
    
    try:
        detector = SimpleYOLODetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("🛑 Simple YOLOv8 Detector shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
