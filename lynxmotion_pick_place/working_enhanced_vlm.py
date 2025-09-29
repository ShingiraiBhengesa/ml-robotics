#!/usr/bin/env python3
"""
Working Enhanced VLM Detector - Uses the original approach with enhanced logging
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

class WorkingEnhancedVLM(Node):
    def __init__(self):
        super().__init__('working_enhanced_vlm')
        
        self.bridge = CvBridge()
        self.model_id = 'IDEA-Research/grounding-dino-tiny'
        self.box_threshold = 0.35
        self.text_threshold = 0.25
        self.topic_rgb = '/zedm/zed_node/left/image_rect_color'
        
        # Create output directory for saved images
        self.output_dir = "detection_results"
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"📁 Saving detection images to: {os.path.abspath(self.output_dir)}")
        
        # Start with a single, simple prompt that works
        self.prompts = ["a ball"]  # Start simple
        self.detection_count = 0
        self.image_count = 0
        
        # Load model
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(dev).eval()
        
        # Publishers and subscribers
        self.create_subscription(Image, self.topic_rgb, self.on_image, 10)
        self.create_subscription(String, '/task_cmd', self.on_cmd, 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        
        self.get_logger().info(f"🤖 Working Enhanced VLM Ready!")
        self.get_logger().info(f"   Model: {self.model_id}")
        self.get_logger().info(f"   Device: {dev}")
        self.get_logger().info(f"   Current Prompts: {self.prompts}")

    def on_cmd(self, msg: String):
        try:
            d = ast.literal_eval(msg.data)
            color, obj = d.get('color'), d.get('object')
            if color and obj: 
                self.prompts = [f"a {color} {obj}"]
                self.get_logger().info(f"🎯 NEW TARGET: {self.prompts[0]}")
            elif obj:
                self.prompts = [f"a {obj}"]
                self.get_logger().info(f"🎯 NEW TARGET: {self.prompts[0]}")
        except Exception as e:
            self.get_logger().warn(f'❌ Bad /task_cmd: {e}')

    def draw_detections(self, image, results):
        """Draw bounding boxes and save annotated image"""
        annotated = image.copy()
        
        for i, (box, score, label) in enumerate(zip(results["boxes"], results["scores"], results["labels"])):
            x1, y1, x2, y2 = [int(coord) for coord in box.tolist()]
            confidence = float(score.item() if hasattr(score, "item") else score)
            
            # Choose color based on confidence
            if confidence > 0.7:
                color = (0, 255, 0)  # Green
            elif confidence > 0.5:
                color = (0, 255, 255)  # Yellow  
            else:
                color = (0, 165, 255)  # Orange
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(annotated, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # Add label with coordinates
            label_text = f"{label} ({confidence:.2f})"
            coord_text = f"({center_x},{center_y})"
            
            cv2.rectangle(annotated, (x1, y1-35), (x1 + 200, y1), color, -1)
            cv2.putText(annotated, label_text, (x1 + 2, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated, coord_text, (x1 + 2, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add stats
        stats_text = f"Images: {self.image_count} | Detections: {self.detection_count} | Target: {self.prompts[0]}"
        cv2.putText(annotated, stats_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return annotated

    @torch.inference_mode()
    def on_image(self, msg: Image):
        """Process images and detect objects"""
        self.image_count += 1
        
        # Convert image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')[:, :, ::-1]  # BGR to RGB
        
        # Process with VLM (single prompt only to avoid tensor issues)
        inputs = self.processor(images=img, text=self.prompts, return_tensors="pt").to(self.model.device)
        outputs = self.model(**inputs)
        
        results = self.processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            threshold=self.box_threshold, text_threshold=self.text_threshold,
            target_sizes=[img.shape[:2]]
        )[0]
        
        # Create detection message
        out = Detection2DArray()
        out.header = msg.header
        
        if len(results["boxes"]) > 0:
            self.detection_count += 1
            
            self.get_logger().info(f"🔍 FRAME {self.image_count} - DETECTED {len(results['boxes'])} OBJECTS:")
            
            for i, (box, score, label) in enumerate(zip(results["boxes"], results["scores"], results["labels"])):
                x1, y1, x2, y2 = box.tolist()
                confidence = float(score.item() if hasattr(score, "item") else score)
                center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                
                # Enhanced logging
                self.get_logger().info(f"   [{i+1}] {label}")
                self.get_logger().info(f"       📍 Center: ({center_x:.1f}, {center_y:.1f})")
                self.get_logger().info(f"       📦 BBox: ({x1:.1f}, {y1:.1f}) → ({x2:.1f}, {y2:.1f})")
                self.get_logger().info(f"       🎯 Confidence: {confidence:.3f}")
                
                # Create detection message
                det = Detection2D()
                det.bbox.center.x, det.bbox.center.y = center_x, center_y
                det.bbox.size_x, det.bbox.size_y = (x2-x1), (y2-y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = confidence
                det.results.append(hyp)
                out.detections.append(det)
            
            # Draw annotations and save image
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            annotated_img = self.draw_detections(img_bgr, results)
            
            # Save annotated image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"detection_{timestamp}_frame{self.image_count:06d}.jpg"
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, annotated_img)
            self.get_logger().info(f"💾 Saved: {filename}")
            
            # Publish detections
            if out.detections: 
                self.pub_det.publish(out)
                
        else:
            if self.image_count % 30 == 0:
                self.get_logger().info(f"🔍 FRAME {self.image_count} - No objects detected (Target: {self.prompts[0]})")

def main():
    rclpy.init()
    
    try:
        detector = WorkingEnhancedVLM()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("🛑 Working Enhanced VLM shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
