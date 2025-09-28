#!/usr/bin/env python3
"""
Integration test for the complete pick and place pipeline.
Tests the system without requiring actual hardware.
"""

import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from cv_bridge import CvBridge

class PipelineIntegrationTest(Node):
    def __init__(self):
        super().__init__('pipeline_integration_test')
        self.bridge = CvBridge()
        
        # Test results tracking
        self.detections_received = False
        self.joint_commands_received = False
        self.target_selected = False
        
        # Publishers for test data
        self.task_pub = self.create_publisher(String, '/task_cmd', 10)
        self.image_pub = self.create_publisher(Image, '/zed/left/image_rect_color', 10)
        
        # Subscribers to monitor pipeline output
        self.create_subscription(Detection2DArray, '/detections', self.detection_cb, 10)
        self.create_subscription(JointState, '/arm/joint_cmd', self.joint_cb, 10)
        self.create_subscription(PoseStamped, '/selected_target', self.target_cb, 10)
        
        self.get_logger().info("Pipeline Integration Test initialized")
    
    def detection_cb(self, msg):
        self.detections_received = True
        self.get_logger().info(f"✓ Detections received: {len(msg.detections)} objects")
    
    def joint_cb(self, msg):
        self.joint_commands_received = True
        self.get_logger().info("✓ Joint commands received from IK node")
        # Log joint angles
        if len(msg.position) >= 5:
            angles = [f"{np.degrees(pos):.1f}°" for pos in msg.position[:5]]
            self.get_logger().info(f"  Joint angles: {angles}")
    
    def target_cb(self, msg):
        self.target_selected = True
        pos = msg.pose.position
        self.get_logger().info(f"✓ Target selected at: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    
    def create_test_image(self):
        """Create a synthetic test image with a red circle"""
        # Create a 640x480 test image
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img.fill(50)  # Dark gray background
        
        # Add a red circle (simulating a red ball)
        center = (320, 240)
        radius = 30
        cv2.circle(img, center, radius, (0, 0, 255), -1)  # Red circle in BGR
        
        # Convert to ROS Image message
        return self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
    
    def run_test(self):
        """Run the integration test sequence"""
        self.get_logger().info("🚀 Starting pipeline integration test...")
        time.sleep(2)  # Wait for nodes to initialize
        
        # Step 1: Send task command
        task_msg = String()
        task_msg.data = "{'color': 'red', 'object': 'ball'}"
        self.task_pub.publish(task_msg)
        self.get_logger().info("📝 Sent task command: red ball")
        
        time.sleep(1)
        
        # Step 2: Publish test image repeatedly
        test_image = self.create_test_image()
        test_image.header.frame_id = 'zed_left_camera_frame'
        
        for i in range(10):
            test_image.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(test_image)
            time.sleep(0.5)
        
        self.get_logger().info("📸 Published test images with red ball")
        
        # Wait for pipeline to process
        time.sleep(5)
        
        # Check results
        self.report_results()
    
    def report_results(self):
        """Report test results"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("🔍 PIPELINE INTEGRATION TEST RESULTS")
        self.get_logger().info("="*50)
        
        total_checks = 3
        passed_checks = 0
        
        if self.detections_received:
            self.get_logger().info("✅ VLM Detection: PASSED")
            passed_checks += 1
        else:
            self.get_logger().info("❌ VLM Detection: FAILED")
        
        if self.target_selected:
            self.get_logger().info("✅ Target Selection: PASSED") 
            passed_checks += 1
        else:
            self.get_logger().info("❌ Target Selection: FAILED")
        
        if self.joint_commands_received:
            self.get_logger().info("✅ IK & Joint Commands: PASSED")
            passed_checks += 1
        else:
            self.get_logger().info("❌ IK & Joint Commands: FAILED")
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"🎯 SUMMARY: {passed_checks}/{total_checks} checks passed")
        
        if passed_checks == total_checks:
            self.get_logger().info("🎉 ALL TESTS PASSED! Pipeline is working correctly.")
        else:
            self.get_logger().info("⚠️  Some tests failed. Check individual components.")
        
        self.get_logger().info("="*50)

def main():
    rclpy.init()
    
    test_node = PipelineIntegrationTest()
    
    # Run test in a separate thread to allow ROS spinning
    test_thread = threading.Thread(target=test_node.run_test)
    test_thread.start()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
