#!/usr/bin/env python3
"""
VLM Detection Test - Verify VLM can detect objects without being told what's there
Tests the actual vision capabilities of the model
"""

import rclpy
import time
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from sensor_msgs.msg import Image

class VLMDetectionTest(Node):
    def __init__(self):
        super().__init__('vlm_detection_test')
        
        # Detection tracking
        self.detection_history = []
        self.image_count = 0
        self.detection_count = 0
        
        # Subscribe to detection results
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(Image, '/zedm/zed_node/left/image_rect_color', self.image_callback, 10)
        
        # Publisher for test commands
        self.test_pub = self.create_publisher(String, '/vlm_test_mode', 10)
        
        self.get_logger().info("🧪 VLM Detection Test Started")
        self.get_logger().info("   Monitoring what VLM detects without being told...")
        self.get_logger().info("   Place objects in view and observe detection results")
        
        # Start test timer
        self.test_timer = self.create_timer(10.0, self.print_test_summary)
        self.start_time = time.time()

    def image_callback(self, msg):
        """Count incoming images"""
        self.image_count += 1

    def detection_callback(self, msg):
        """Process and log detections"""
        if len(msg.detections) > 0:
            self.detection_count += 1
            current_time = time.time() - self.start_time
            
            detected_objects = []
            for detection in msg.detections:
                if detection.results:
                    label = detection.results[0].hypothesis.class_id
                    confidence = detection.results[0].hypothesis.score
                    x = detection.bbox.center.x
                    y = detection.bbox.center.y
                    
                    detected_objects.append({
                        'label': label,
                        'confidence': confidence,
                        'position': (x, y)
                    })
            
            self.detection_history.append({
                'time': current_time,
                'objects': detected_objects
            })
            
            # Real-time logging
            self.get_logger().info(f"🎯 DETECTION #{self.detection_count} at {current_time:.1f}s:")
            for i, obj in enumerate(detected_objects):
                self.get_logger().info(f"   [{i+1}] '{obj['label']}' - Confidence: {obj['confidence']:.3f} - Position: ({obj['position'][0]:.0f}, {obj['position'][1]:.0f})")

    def print_test_summary(self):
        """Print test summary every 10 seconds"""
        elapsed = time.time() - self.start_time
        
        # Get unique objects detected
        unique_objects = set()
        total_detections = 0
        high_confidence_detections = 0
        
        for detection_event in self.detection_history:
            for obj in detection_event['objects']:
                unique_objects.add(obj['label'])
                total_detections += 1
                if obj['confidence'] > 0.7:
                    high_confidence_detections += 1
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📊 VLM DETECTION TEST SUMMARY")
        self.get_logger().info("="*60)
        self.get_logger().info(f"⏱️  Test Duration: {elapsed:.1f} seconds")
        self.get_logger().info(f"📷 Images Processed: {self.image_count}")
        self.get_logger().info(f"🎯 Detection Events: {self.detection_count}")
        self.get_logger().info(f"🔍 Total Object Detections: {total_detections}")
        self.get_logger().info(f"✅ High Confidence (>0.7): {high_confidence_detections}")
        self.get_logger().info(f"🏷️  Unique Objects Detected: {len(unique_objects)}")
        
        if unique_objects:
            self.get_logger().info("   Objects Found:")
            for obj in sorted(unique_objects):
                # Count how many times this object was detected
                count = sum(1 for event in self.detection_history 
                           for det_obj in event['objects'] 
                           if det_obj['label'] == obj)
                avg_conf = sum(det_obj['confidence'] for event in self.detection_history 
                              for det_obj in event['objects'] 
                              if det_obj['label'] == obj) / count
                self.get_logger().info(f"     • {obj}: {count} detections (avg conf: {avg_conf:.3f})")
        else:
            self.get_logger().info("❌ No objects detected yet")
            self.get_logger().info("   • Check if camera is working")
            self.get_logger().info("   • Place objects in camera view")
            self.get_logger().info("   • Verify lighting conditions")
        
        self.get_logger().info("="*60)

    def get_detection_quality(self):
        """Evaluate detection quality"""
        if not self.detection_history:
            return "No detections yet"
        
        total_confidence = sum(obj['confidence'] 
                             for event in self.detection_history 
                             for obj in event['objects'])
        avg_confidence = total_confidence / sum(len(event['objects']) for event in self.detection_history)
        
        if avg_confidence > 0.8:
            return "Excellent (>0.8)"
        elif avg_confidence > 0.6:
            return "Good (>0.6)"
        elif avg_confidence > 0.4:
            return "Fair (>0.4)"
        else:
            return "Poor (<0.4)"

def main():
    rclpy.init()
    
    print("\n" + "="*70)
    print("🧪 VLM DETECTION CAPABILITY TEST")
    print("="*70)
    print("This test monitors what the VLM detects without being told what to look for.")
    print("Place various objects in the camera view and observe the results.")
    print("Press Ctrl+C to stop the test.")
    print("="*70)
    
    try:
        test_node = VLMDetectionTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n🛑 VLM Detection Test stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
