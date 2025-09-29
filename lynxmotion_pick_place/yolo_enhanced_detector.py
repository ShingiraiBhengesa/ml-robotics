#!/usr/bin/env python3
"""
YOLOv8 Enhanced Detector - Reliable Object Detection with Enhanced Logging
- Uses YOLOv8 for rock-solid object detection (no tensor issues)
- Comprehensive logging with coordinates and confidence
- Automatic image saving with bounding boxes
- Color detection for natural language commands  
- Perfect for robotics applications
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
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, ObjectHypothesis
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLOEnhancedDetector(Node):
    def __init__(self):
        super().__init__('yolo_enhanced_detector')
        
        # Initialize parameters
        self.bridge = CvBridge()
        self.confidence_threshold = 0.25
        self.topic_rgb = '/zedm/zed_node/left/image_rect_color'
        self.save_images = True
        
        # Create output directory for saved images
        self.output_dir = "detection_results"
        if self.save_images:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f"📁 Saving detection images to: {os.path.abspath(self.output_dir)}")
        
        # Object class mapping for robotics (COCO classes relevant to pick-and-place)
        self.target_classes = {
            0: 'person',
            32: 'sports ball',    # This covers balls
            39: 'bottle',         # Bottles  
            41: 'cup',            # Cups
            46: 'bowl',           # Bowls
            47: 'banana',         # Food items
            49: 'orange',
            67: 'cell phone',     # Small objects
            64: 'laptop',
            73: 'book',
            84: 'scissors'        # Tools
        }
        
        # Color detection HSV ranges
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)],  # Red wraps around
            'blue': [(100, 50, 50), (130, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'yellow': [(20, 50, 50), (30, 255, 255)],
            'orange': [(10, 50, 50), (20, 255, 255)],
            'purple': [(130, 50, 50), (160, 255, 255)]
        }
        
        # Current target (from natural language)
        self.current_target = None
        
        # Stats tracking
        self.detection_count = 0
        self.image_count = 0
        
        # Load YOLOv8 model
        self.get_logger().info("🤖 Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')  # nano version for speed
        
        # Publishers and subscribers
        self.create_subscription(Image, self.topic_rgb, self.on_image, 10)
        self.create_subscription(String, '/task_cmd', self.on_cmd, 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/detections/annotated_image', 10)
        
        self.get_logger().info(f"🤖 YOLOv8 Enhanced Detector Ready!")
        self.get_logger().info(f"   Model: YOLOv8n (ultra-fast, 6.2MB)")
        self.get_logger().info(f"   Device: {'GPU' if self.model.device.type == 'cuda' else 'CPU'}")
        self.get_logger().info(f"   Listening: {self.topic_rgb}")
        self.get_logger().info(f"   Confidence Threshold: {self.confidence_threshold}")
        self.get_logger().info(f"   Target Classes: {list(self.target_classes.values())}")

    def on_cmd(self, msg: String):
        """Handle natural language task commands"""
        try:
            d = ast.literal_eval(msg.data)
            color = d.get('color')
            obj = d.get('object')
            
            if color and obj:
                self.current_target = {'color': color.lower(), 'object': obj.lower()}
                self.get_logger().info(f"🎯 NEW TARGET: {color} {obj}")
            elif obj:
                self.current_target = {'color': None, 'object': obj.lower()}
                self.get_logger().info(f"🎯 NEW TARGET: {obj}")
            else:
                self.current_target = None
                self.get_logger().info("🔍 GENERAL DETECTION MODE")
                
        except Exception as e:
            self.get_logger().warn(f'❌ Bad /task_cmd: {e}')

    def detect_dominant_color(self, image, bbox):
        """Detect the dominant color in the bounding box region"""
        x1, y1, x2, y2 = [int(coord) for coord in bbox]
        
        # Extract region of interest
        roi = image[y1:y2, x1:x2]
        if roi.size == 0:
            return 'unknown'
        
        # Convert to HSV for better color detection
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Check each color range
        max_pixels = 0
        detected_color = 'unknown'
        
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
            
            if color_name == 'red':  # Red wraps around in HSV
                mask1 = cv2.inRange(hsv_roi, np.array(ranges[0]), np.array(ranges[1]))
                mask2 = cv2.inRange(hsv_roi, np.array(ranges[2]), np.array(ranges[3]))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_roi, np.array(ranges[0]), np.array(ranges[1]))
            
            pixel_count = cv2.countNonZero(mask)
            if pixel_count > max_pixels:
                max_pixels = pixel_count
                detected_color = color_name
        
        # Only return color if it's significant (>20% of region)
        total_pixels = roi.shape[0] * roi.shape[1]
        if max_pixels > total_pixels * 0.2:
            return detected_color
        else:
            return 'unknown'

    def should_detect_object(self, class_name, detected_color):
        """Check if object matches current target criteria"""
        if not self.current_target:
            return True  # Detect everything in general mode
        
        target_object = self.current_target.get('object')
        target_color = self.current_target.get('color')
        
        # Map object names (handle variations)
        object_matches = False
        if target_object:
            if target_object in ['ball', 'sphere'] and 'ball' in class_name:
                object_matches = True
            elif target_object == 'bottle' and 'bottle' in class_name:
                object_matches = True
            elif target_object == 'cup' and 'cup' in class_name:
                object_matches = True
            elif target_object == 'bowl' and 'bowl' in class_name:
                object_matches = True
            elif target_object in class_name:
                object_matches = True
        
        # Check color match
        color_matches = True  # Default to true if no color specified
        if target_color and detected_color != 'unknown':
            color_matches = (target_color == detected_color)
        
        return object_matches and color_matches

    def draw_detections(self, image, detections_info):
        """Draw bounding boxes and annotations on image"""
        annotated = image.copy()
        
        for detection in detections_info:
            bbox = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            color_detected = detection['color']
            
            x1, y1, x2, y2 = [int(coord) for coord in bbox]
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            
            # Choose color based on confidence
            if confidence > 0.8:
                box_color = (0, 255, 0)  # Green for high confidence
            elif confidence > 0.6:
                box_color = (0, 255, 255)  # Yellow for medium confidence
            else:
                box_color = (0, 165, 255)  # Orange for lower confidence
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)
            
            # Draw center point
            cv2.circle(annotated, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # Create detailed label
            if color_detected != 'unknown':
                label_text = f"{color_detected} {class_name} ({confidence:.2f})"
            else:
                label_text = f"{class_name} ({confidence:.2f})"
            coord_text = f"({center_x},{center_y})"
            
            # Calculate label background size
            (label_w, label_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            (coord_w, coord_h), _ = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            # Draw label background
            bg_width = max(label_w, coord_w) + 10
            cv2.rectangle(annotated, (x1, y1-40), (x1 + bg_width, y1), box_color, -1)
            
            # Draw text
            cv2.putText(annotated, label_text, (x1 + 5, y1 - 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated, coord_text, (x1 + 5, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add status overlay
        target_text = f"Target: {self.current_target or 'All Objects'}"
        stats_text = f"Images: {self.image_count} | Detections: {self.detection_count}"
        
        cv2.putText(annotated, target_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(annotated, stats_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return annotated

    def on_image(self, msg: Image):
        """Process incoming images with YOLOv8"""
        self.image_count += 1
        
        # Convert ROS image to OpenCV
        img_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run YOLOv8 detection
        results = self.model(img_bgr, conf=self.confidence_threshold, verbose=False)
        
        # Process detections
        detections_info = []
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    # Extract detection data
                    bbox = box.xyxy[0].cpu().numpy()  # x1, y1, x2, y2
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    # Get class name
                    class_name = self.model.names[class_id]
                    
                    # Process ALL objects that YOLOv8 detects
                    if True:  # Detect everything for debugging
                        # Detect color in bounding box region
                        detected_color = self.detect_dominant_color(img_bgr, bbox)
                        
                        # Check if this detection matches our target
                        if self.should_detect_object(class_name, detected_color):
                            
                            # Store detection info
                            detection_info = {
                                'bbox': bbox,
                                'class': class_name,
                                'confidence': confidence,
                                'color': detected_color
                            }
                            detections_info.append(detection_info)
                            
                            # Create ROS detection message
                            det = Detection2D()
                            center_x, center_y = (bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0
                            det.bbox.center.x = center_x
                            det.bbox.center.y = center_y
                            det.bbox.size_x = float(bbox[2] - bbox[0])
                            det.bbox.size_y = float(bbox[3] - bbox[1])
                            
                            hyp = ObjectHypothesisWithPose()
                            if detected_color != 'unknown':
                                hyp.hypothesis.class_id = f"{detected_color} {class_name}"
                            else:
                                hyp.hypothesis.class_id = class_name
                            hyp.hypothesis.score = confidence
                            det.results.append(hyp)
                            detection_array.detections.append(det)
        
        # Log and process detections
        if detections_info:
            self.detection_count += 1
            
            self.get_logger().info(f"🔍 FRAME {self.image_count} - DETECTED {len(detections_info)} OBJECTS:")
            
            for i, detection in enumerate(detections_info):
                bbox = detection['bbox']
                class_name = detection['class']
                confidence = detection['confidence']
                detected_color = detection['color']
                
                center_x, center_y = (bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0
                
                # Enhanced logging
                color_info = f" ({detected_color})" if detected_color != 'unknown' else ""
                self.get_logger().info(f"   [{i+1}] {class_name}{color_info}")
                self.get_logger().info(f"       📍 Center: ({center_x:.1f}, {center_y:.1f})")
                self.get_logger().info(f"       📦 BBox: ({bbox[0]:.1f}, {bbox[1]:.1f}) → ({bbox[2]:.1f}, {bbox[3]:.1f})")
                self.get_logger().info(f"       🎯 Confidence: {confidence:.3f}")
                if detected_color != 'unknown':
                    self.get_logger().info(f"       🎨 Dominant Color: {detected_color}")
            
            # Draw annotations
            annotated_img = self.draw_detections(img_bgr, detections_info)
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
                annotated_msg.header = msg.header
                self.pub_annotated.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")
            
            # Save image if enabled
            if self.save_images:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"yolo_detection_{timestamp}_frame{self.image_count:06d}.jpg"
                filepath = os.path.join(self.output_dir, filename)
                
                try:
                    cv2.imwrite(filepath, annotated_img)
                    self.get_logger().info(f"💾 Saved: {filename}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save image: {e}")
            
            # Publish detections
            if detection_array.detections:
                self.pub_det.publish(detection_array)
                
        else:
            # No detections - log occasionally
            if self.image_count % 60 == 0:  # Every 2 seconds at 30fps
                target_info = f"Target: {self.current_target}" if self.current_target else "All objects"
                self.get_logger().info(f"🔍 FRAME {self.image_count} - No objects detected ({target_info})")

def main():
    rclpy.init()
    
    print("\n" + "="*70)
    print("🚀 YOLOv8 ENHANCED DETECTOR")
    print("="*70)
    print("✅ Rock-solid object detection with zero tensor issues")
    print("✅ Automatic color detection for natural language commands")
    print("✅ Enhanced logging with coordinates and confidence scores")
    print("✅ Automatic image saving with bounding boxes")
    print("✅ Perfect integration with robotics pipeline")
    print("="*70)
    
    try:
        detector = YOLOEnhancedDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("🛑 YOLOv8 Enhanced Detector shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
