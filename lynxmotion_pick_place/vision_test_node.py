#!/usr/bin/env python3
"""
Vision Pipeline Test Node
Tests VLM detection + depth sampling pipeline in isolation.

This node helps verify:
1. VLM is detecting objects correctly
2. Depth sampling is working with detections  
3. 3D coordinates are being extracted properly
4. Visual feedback of detection results
"""

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading

class VisionTestNode(Node):
    def __init__(self):
        super().__init__('vision_test_node')
        
        self.bridge = CvBridge()
        
        # Test results tracking
        self.latest_rgb_image = None
        self.latest_detections = None
        self.latest_poses = None
        self.detection_count = 0
        self.pose_count = 0
        
        # Subscribers for monitoring the vision pipeline
        self.create_subscription(Image, '/zed/left/image_rect_color', self.rgb_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(PoseArray, '/target_candidates', self.pose_callback, 10)
        
        # Publisher for test commands
        self.task_pub = self.create_publisher(String, '/task_cmd', 10)
        
        self.get_logger().info("Vision Test Node started!")
        self.get_logger().info("Monitoring vision pipeline...")
        
        # Start test interface
        self.start_test_interface()
    
    def rgb_callback(self, msg):
        """Store latest RGB image."""
        try:
            self.latest_rgb_image = (msg, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
    
    def detection_callback(self, msg):
        """Monitor VLM detections."""
        self.latest_detections = msg
        self.detection_count += 1
        
        self.get_logger().info(f"🔍 Detections received: {len(msg.detections)} objects")
        
        for i, detection in enumerate(msg.detections):
            x = detection.bbox.center.x
            y = detection.bbox.center.y  
            w = detection.bbox.size_x
            h = detection.bbox.size_y
            
            if detection.results:
                score = detection.results[0].hypothesis.score
                self.get_logger().info(f"  Detection {i+1}: center=({x:.1f},{y:.1f}) size=({w:.1f}x{h:.1f}) score={score:.3f}")
        
        # Visualize detections if we have RGB image
        if self.latest_rgb_image is not None:
            self.visualize_detections()
    
    def pose_callback(self, msg):
        """Monitor 3D poses from depth sampling."""
        self.latest_poses = msg
        self.pose_count += 1
        
        self.get_logger().info(f"📍 3D Poses received: {len(msg.poses)} positions")
        
        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            self.get_logger().info(f"  Pose {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    def visualize_detections(self):
        """Visualize detections on the RGB image."""
        if self.latest_rgb_image is None or self.latest_detections is None:
            return
        
        try:
            img_msg, img = self.latest_rgb_image
            vis_img = img.copy()
            
            # Draw detection boxes
            for i, detection in enumerate(self.latest_detections.detections):
                x = int(detection.bbox.center.x)
                y = int(detection.bbox.center.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                
                # Calculate box corners
                x1 = x - w // 2
                y1 = y - h // 2
                x2 = x + w // 2
                y2 = y + h // 2
                
                # Draw bounding box
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(vis_img, (x, y), 5, (255, 0, 0), -1)
                
                # Add label
                if detection.results:
                    score = detection.results[0].hypothesis.score
                    label = f"Det {i+1}: {score:.2f}"
                    cv2.putText(vis_img, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add stats overlay
            stats_text = f"Detections: {len(self.latest_detections.detections)}"
            if self.latest_poses:
                stats_text += f" | 3D Poses: {len(self.latest_poses.poses)}"
            
            cv2.putText(vis_img, stats_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Show image
            cv2.imshow('Vision Pipeline Test', vis_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")
    
    def send_test_command(self, command):
        """Send a test command to the pipeline."""
        try:
            task_msg = String()
            task_msg.data = str(command)
            self.task_pub.publish(task_msg)
            self.get_logger().info(f"📤 Sent test command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
    
    def print_status(self):
        """Print current pipeline status."""
        print("\n" + "="*60)
        print("📊 VISION PIPELINE STATUS")
        print("="*60)
        print(f"📷 RGB Images: {'✅ Receiving' if self.latest_rgb_image else '❌ No data'}")
        print(f"🔍 VLM Detections: {self.detection_count} messages received")
        print(f"📍 3D Poses: {self.pose_count} messages received")
        
        if self.latest_detections:
            print(f"   Latest: {len(self.latest_detections.detections)} objects detected")
        
        if self.latest_poses:
            print(f"   Latest: {len(self.latest_poses.poses)} 3D positions")
        
        print("="*60)
    
    def start_test_interface(self):
        """Start interactive test interface."""
        def interface_loop():
            print("\n" + "="*60)
            print("🔬 VISION PIPELINE TEST INTERFACE") 
            print("="*60)
            print("Commands:")
            print("  1 - Test 'red ball' detection")
            print("  2 - Test 'blue bottle' detection") 
            print("  3 - Test 'yellow block' detection")
            print("  s - Show status")
            print("  q - Quit")
            print("="*60)
            
            while rclpy.ok():
                try:
                    cmd = input("\n🧪 Test command: ").strip().lower()
                    
                    if cmd == 'q' or cmd == 'quit':
                        print("👋 Exiting vision test...")
                        break
                    elif cmd == 's' or cmd == 'status':
                        self.print_status()
                    elif cmd == '1':
                        self.send_test_command({'color': 'red', 'object': 'ball'})
                        print("🔴 Testing red ball detection...")
                    elif cmd == '2':
                        self.send_test_command({'color': 'blue', 'object': 'bottle'})
                        print("🔵 Testing blue bottle detection...")
                    elif cmd == '3':
                        self.send_test_command({'color': 'yellow', 'object': 'block'})
                        print("🟡 Testing yellow block detection...")
                    else:
                        print("❓ Unknown command. Try 1, 2, 3, s, or q")
                        
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break
                except Exception as e:
                    print(f"❌ Error: {e}")
        
        # Start interface in separate thread
        interface_thread = threading.Thread(target=interface_loop, daemon=True)
        interface_thread.start()

def main():
    rclpy.init()
    
    try:
        node = VisionTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Vision Test Node shutting down...")
    finally:
        # Close any OpenCV windows
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
