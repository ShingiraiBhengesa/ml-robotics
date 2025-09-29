#!/usr/bin/env python3
"""
Enhanced VLM Detector Node with Robust Visualization
- Logs all detections with confidence scores
- Saves annotated images with bounding boxes
- Shows coordinates and labels clearly
- Provides comprehensive debugging output
"""

import rclpy
import torch
import ast
import cv2
import numpy as np
import os
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

class EnhancedVLMDetector(Node):
    def __init__(self):
        super().__init__('enhanced_vlm_detector')
        
        # Initialize parameters
        self.bridge = CvBridge()
        self.model_id = self.declare_parameter('model_id', 'IDEA-Research/grounding-dino-tiny').get_parameter_value().string_value
        self.box_threshold = float(self.declare_parameter('box_threshold', 0.35).value)
        self.text_threshold = float(self.declare_parameter('text_threshold', 0.25).value)
        self.topic_rgb = self.declare_parameter('topic_rgb', '/zedm/zed_node/left/image_rect_color').get_parameter_value().string_value
        self.save_images = self.declare_parameter('save_images', True).get_parameter_value().bool_value
        
        # Create output directory for saved images
        self.output_dir = "detection_results"
        if self.save_images:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f"📁 Saving detection images to: {os.path.abspath(self.output_dir)}")
        
        # Current detection prompts - Single prompt to avoid tensor issues
        self.prompt_options = ["a ball", "a bottle", "a cup", "a block", "a toy", "an object", "a red object", "a blue object"]
        self.current_prompt_index = 0
        self.prompts = [self.prompt_options[self.current_prompt_index]]
        self.current_target = None
        self.general_mode = True
        
        # Stats tracking
        self.detection_count = 0
        self.image_count = 0
        
        # Load model
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(self.device).eval()
        
        # Publishers and subscribers
        self.create_subscription(Image, self.topic_rgb, self.on_image, 10)
        self.create_subscription(String, '/task_cmd', self.on_cmd, 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/detections/annotated_image', 10)
        
        self.get_logger().info(f"🤖 Enhanced VLM Detector Ready!")
        self.get_logger().info(f"   Model: {self.model_id}")
        self.get_logger().info(f"   Device: {self.device}")
        self.get_logger().info(f"   Listening: {self.topic_rgb}")
        self.get_logger().info(f"   Box Threshold: {self.box_threshold}")
        self.get_logger().info(f"   Text Threshold: {self.text_threshold}")
        self.get_logger().info(f"   Current Prompts: {self.prompts}")

    def on_cmd(self, msg: String):
        """Handle task commands and update detection targets"""
        try:
            d = ast.literal_eval(msg.data)
            color = d.get('color')
            obj = d.get('object')
            
            if color and obj:
                # Focus on specific target
                self.current_target = f"a {color} {obj}"
                self.prompts = [self.current_target]
                self.get_logger().info(f"🎯 NEW TARGET: '{self.current_target}'")
            elif obj:
                # Just object, no color
                self.current_target = f"a {obj}"
                self.prompts = [self.current_target]
                self.get_logger().info(f"🎯 NEW TARGET: '{self.current_target}'")
            else:
                # Reset to general detection
                self.prompts = ["a red ball", "a blue bottle", "a yellow block", "a green cup", "a bottle", "a ball", "a block", "a cup"]
                self.current_target = None
                self.get_logger().info("🔍 GENERAL DETECTION MODE")
                
        except Exception as e:
            self.get_logger().warn(f'❌ Bad /task_cmd: {e}')

    def draw_detections(self, image, detections_data):
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        for i, (box, score, label) in enumerate(zip(
            detections_data["boxes"], 
            detections_data["scores"], 
            detections_data["labels"]
        )):
            x1, y1, x2, y2 = [int(coord) for coord in box.tolist()]
            confidence = float(score.item() if hasattr(score, "item") else score)
            
            # Choose color based on confidence
            if confidence > 0.7:
                color = (0, 255, 0)  # Green for high confidence
            elif confidence > 0.5:
                color = (0, 255, 255)  # Yellow for medium confidence
            else:
                color = (0, 165, 255)  # Orange for lower confidence
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(annotated, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # Create label with coordinates and confidence
            label_text = f"{label} ({confidence:.2f})"
            coord_text = f"({center_x},{center_y})"
            
            # Draw label background
            (label_w, label_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            (coord_w, coord_h), _ = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            cv2.rectangle(annotated, (x1, y1-35), (x1 + max(label_w, coord_w) + 5, y1), color, -1)
            
            # Draw text
            cv2.putText(annotated, label_text, (x1 + 2, y1 - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated, coord_text, (x1 + 2, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add overlay info
        info_text = f"Target: {self.current_target or 'General Detection'}"
        stats_text = f"Images: {self.image_count} | Detections: {self.detection_count}"
        
        cv2.putText(annotated, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(annotated, stats_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return annotated

    @torch.inference_mode()
    def on_image(self, msg: Image):
        """Process incoming images and detect objects"""
        self.image_count += 1
        
        # Convert ROS image to OpenCV
        img_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # Process with VLM
        inputs = self.processor(images=img_rgb, text=self.prompts, return_tensors="pt").to(self.device)
        outputs = self.model(**inputs)
        
        # Post-process results
        results = self.processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            threshold=self.box_threshold, 
            text_threshold=self.text_threshold,
            target_sizes=[img_rgb.shape[:2]]
        )[0]
        
        # Create detection message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        if len(results["boxes"]) > 0:
            self.detection_count += 1
            
            self.get_logger().info(f"🔍 FRAME {self.image_count} - DETECTED {len(results['boxes'])} OBJECTS:")
            
            for i, (box, score, label) in enumerate(zip(results["boxes"], results["scores"], results["labels"])):
                x1, y1, x2, y2 = box.tolist()
                confidence = float(score.item() if hasattr(score, "item") else score)
                center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                
                # Log detection details
                self.get_logger().info(f"   [{i+1}] {label}")
                self.get_logger().info(f"       📍 Center: ({center_x:.1f}, {center_y:.1f})")
                self.get_logger().info(f"       📦 BBox: ({x1:.1f}, {y1:.1f}) → ({x2:.1f}, {y2:.1f})")
                self.get_logger().info(f"       🎯 Confidence: {confidence:.3f}")
                
                # Create ROS detection message
                det = Detection2D()
                det.bbox.center.x = center_x
                det.bbox.center.y = center_y
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = confidence
                det.results.append(hyp)
                detection_array.detections.append(det)
            
            # Draw annotations
            annotated_img = self.draw_detections(img_bgr, results)
            
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
                filename = f"detection_{timestamp}_frame{self.image_count:06d}.jpg"
                filepath = os.path.join(self.output_dir, filename)
                
                try:
                    cv2.imwrite(filepath, annotated_img)
                    self.get_logger().info(f"💾 Saved: {filename}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save image: {e}")
            
            # Publish detections
            self.pub_det.publish(detection_array)
            
        else:
            # No detections
            if self.image_count % 30 == 0:  # Log every 30 frames when no detection
                self.get_logger().info(f"🔍 FRAME {self.image_count} - No objects detected (Target: {self.current_target or 'General'})")

def main():
    rclpy.init()
    
    try:
        detector = EnhancedVLMDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("🛑 Enhanced VLM Detector shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
