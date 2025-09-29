#!/usr/bin/env python3
"""
Enhanced Vision Test Node - Shows 3D Positions with Visual Feedback
- Displays object detections with 3D coordinates
- Shows depth data integration
- Saves results with comprehensive logging
- Real-time visualization of detection → 3D position pipeline
"""

import rclpy
import cv2
import numpy as np
import os
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading

class Enhanced3DVisionTest(Node):
    def __init__(self):
        super().__init__('enhanced_3d_vision_test')
        
        self.bridge = CvBridge()
        
        # Create output directory
        self.output_dir = "3d_detection_results"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Data storage
        self.latest_rgb_image = None
        self.latest_detections = None
        self.latest_3d_poses = None
        self.detection_count = 0
        self.pose_count = 0
        
        # Subscribers
        self.create_subscription(Image, '/zedm/zed_node/left/image_rect_color', self.rgb_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(PoseArray, '/target_candidates', self.pose_callback, 10)
        
        # Publishers
        self.create_subscription(String, '/task_cmd', self.task_callback, 10)
        self.task_pub = self.create_publisher(String, '/task_cmd', 10)
        
        self.get_logger().info("🔬 Enhanced 3D Vision Test Node Ready!")
        self.get_logger().info(f"📁 Saving results to: {os.path.abspath(self.output_dir)}")
        
        # Start interactive test interface
        self.start_test_interface()
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_rgb_image = (msg.header, cv_image)
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
    
    def detection_callback(self, msg):
        """Process VLM detections"""
        self.latest_detections = msg
        self.detection_count += 1
        
        if len(msg.detections) > 0:
            self.get_logger().info(f"🔍 DETECTIONS UPDATE #{self.detection_count}")
            self.get_logger().info(f"   Found {len(msg.detections)} objects:")
            
            for i, detection in enumerate(msg.detections):
                x = detection.bbox.center.x
                y = detection.bbox.center.y  
                w = detection.bbox.size_x
                h = detection.bbox.size_y
                
                if detection.results:
                    label = detection.results[0].hypothesis.class_id
                    score = detection.results[0].hypothesis.score
                    self.get_logger().info(f"     [{i+1}] {label} (conf: {score:.3f})")
                    self.get_logger().info(f"         📍 2D Center: ({x:.1f}, {y:.1f})")
                    self.get_logger().info(f"         📦 BBox Size: {w:.1f} x {h:.1f}")
            
            # Update visualization
            self.update_visualization()
    
    def pose_callback(self, msg):
        """Process 3D poses from depth sampling"""
        self.latest_3d_poses = msg
        self.pose_count += 1
        
        if len(msg.poses) > 0:
            self.get_logger().info(f"📍 3D POSES UPDATE #{self.pose_count}")
            self.get_logger().info(f"   Found {len(msg.poses)} 3D positions:")
            
            for i, pose in enumerate(msg.poses):
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                
                self.get_logger().info(f"     [{i+1}] 3D Position:")
                self.get_logger().info(f"         X: {x:.3f}m")
                self.get_logger().info(f"         Y: {y:.3f}m") 
                self.get_logger().info(f"         Z: {z:.3f}m (distance)")
                self.get_logger().info(f"         🎯 Total Distance: {np.sqrt(x*x + y*y + z*z):.3f}m")
            
            # Update visualization
            self.update_visualization()
    
    def task_callback(self, msg):
        """Monitor task commands"""
        self.get_logger().info(f"📤 Task Command Received: {msg.data}")
    
    def update_visualization(self):
        """Create visualization combining detections and 3D data"""
        if self.latest_rgb_image is None:
            return
            
        header, img = self.latest_rgb_image
        vis_img = img.copy()
        
        # Draw 2D detections
        if self.latest_detections:
            for i, detection in enumerate(self.latest_detections.detections):
                x = int(detection.bbox.center.x)
                y = int(detection.bbox.center.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                
                # Bounding box
                x1, y1 = x - w//2, y - h//2
                x2, y2 = x + w//2, y + h//2
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Center point
                cv2.circle(vis_img, (x, y), 8, (255, 0, 0), -1)
                
                # Label
                if detection.results:
                    label = detection.results[0].hypothesis.class_id
                    score = detection.results[0].hypothesis.score
                    label_text = f"{label} ({score:.2f})"
                    
                    # Background for text
                    (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(vis_img, (x1, y1-30), (x1 + tw + 5, y1), (0, 255, 0), -1)
                    cv2.putText(vis_img, label_text, (x1 + 2, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add 3D position data
        if self.latest_3d_poses:
            y_offset = 30
            for i, pose in enumerate(self.latest_3d_poses.poses):
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                distance = np.sqrt(x*x + y*y + z*z)
                
                pos_text = f"Obj {i+1}: ({x:.3f}, {y:.3f}, {z:.3f}) - {distance:.3f}m"
                cv2.putText(vis_img, pos_text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y_offset += 25
        
        # Add status information
        status_text = f"Detections: {self.detection_count} | 3D Poses: {self.pose_count}"
        cv2.putText(vis_img, status_text, (10, img.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display image
        cv2.imshow('3D Vision Pipeline', vis_img)
        cv2.waitKey(1)
        
        # Save comprehensive result
        if self.latest_detections and self.latest_3d_poses:
            self.save_comprehensive_result(vis_img)
    
    def save_comprehensive_result(self, vis_img):
        """Save detection results with all data"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        
        # Save image
        img_filename = f"3d_result_{timestamp}.jpg"
        img_path = os.path.join(self.output_dir, img_filename)
        cv2.imwrite(img_path, vis_img)
        
        # Save data file
        data_filename = f"3d_data_{timestamp}.txt"
        data_path = os.path.join(self.output_dir, data_filename)
        
        with open(data_path, 'w') as f:
            f.write(f"3D Detection Results - {timestamp}\n")
            f.write("="*50 + "\n\n")
            
            f.write("2D DETECTIONS:\n")
            if self.latest_detections:
                for i, det in enumerate(self.latest_detections.detections):
                    f.write(f"  [{i+1}] Center: ({det.bbox.center.x:.1f}, {det.bbox.center.y:.1f})\n")
                    f.write(f"      Size: {det.bbox.size_x:.1f} x {det.bbox.size_y:.1f}\n")
                    if det.results:
                        f.write(f"      Label: {det.results[0].hypothesis.class_id}\n")
                        f.write(f"      Confidence: {det.results[0].hypothesis.score:.3f}\n")
                    f.write("\n")
            
            f.write("\n3D POSITIONS:\n")
            if self.latest_3d_poses:
                for i, pose in enumerate(self.latest_3d_poses.poses):
                    x, y, z = pose.position.x, pose.position.y, pose.position.z
                    dist = np.sqrt(x*x + y*y + z*z)
                    f.write(f"  [{i+1}] Position: ({x:.3f}, {y:.3f}, {z:.3f}) m\n")
                    f.write(f"      Distance: {dist:.3f} m\n\n")
        
        self.get_logger().info(f"💾 Saved comprehensive result: {img_filename} + {data_filename}")
    
    def send_test_command(self, command):
        """Send test command to pipeline"""
        try:
            task_msg = String()
            task_msg.data = str(command)
            self.task_pub.publish(task_msg)
            self.get_logger().info(f"📤 Sent: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
    
    def print_status(self):
        """Print current status"""
        print("\n" + "="*70)
        print("📊 3D VISION PIPELINE STATUS")
        print("="*70)
        print(f"📷 RGB Images: {'✅ Receiving' if self.latest_rgb_image else '❌ No data'}")
        print(f"🔍 2D Detections: {self.detection_count} received")
        print(f"📍 3D Poses: {self.pose_count} received")
        
        if self.latest_detections:
            print(f"   Current 2D detections: {len(self.latest_detections.detections)} objects")
        
        if self.latest_3d_poses:
            print(f"   Current 3D positions: {len(self.latest_3d_poses.poses)} objects")
            
        print("="*70)
    
    def start_test_interface(self):
        """Interactive test interface"""
        def interface_loop():
            print("\n" + "="*70)
            print("🔬 ENHANCED 3D VISION TEST INTERFACE")
            print("="*70)
            print("Commands:")
            print("  1 - Test 'red ball' detection")
            print("  2 - Test 'blue bottle' detection") 
            print("  3 - Test 'yellow block' detection")
            print("  4 - Test 'green cup' detection")
            print("  s - Show status")
            print("  q - Quit")
            print("="*70)
            
            while rclpy.ok():
                try:
                    cmd = input("\n🧪 Test command: ").strip().lower()
                    
                    if cmd in ['q', 'quit']:
                        print("👋 Exiting 3D vision test...")
                        break
                    elif cmd in ['s', 'status']:
                        self.print_status()
                    elif cmd == '1':
                        self.send_test_command({'color': 'red', 'object': 'ball'})
                    elif cmd == '2':
                        self.send_test_command({'color': 'blue', 'object': 'bottle'})
                    elif cmd == '3':
                        self.send_test_command({'color': 'yellow', 'object': 'block'})
                    elif cmd == '4':
                        self.send_test_command({'color': 'green', 'object': 'cup'})
                    else:
                        print("❓ Unknown command. Try 1-4, s, or q")
                        
                except (KeyboardInterrupt, EOFError):
                    break
                except Exception as e:
                    print(f"❌ Error: {e}")
        
        interface_thread = threading.Thread(target=interface_loop, daemon=True)
        interface_thread.start()

def main():
    rclpy.init()
    
    try:
        node = Enhanced3DVisionTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Enhanced 3D Vision Test shutting down...")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
