import rclpy, ast, cv2, numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.bridge = CvBridge()
        self.create_subscription(String, '/task_cmd', self.cmd_cb, 10)
        self.create_subscription(Detection2DArray, '/detections', self.det_cb, 10)
        self.create_subscription(Image, '/zed/left/image_rect_color', self.rgb_cb, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/selected_target', 10)
        self.pending_task = None
        self.last_rgb = None

    def rgb_cb(self, img: Image):
        self.last_rgb = (img, self.bridge.imgmsg_to_cv2(img, 'bgr8'))

    def cmd_cb(self, msg: String):
        try:
            self.pending_task = ast.literal_eval(msg.data)
            self.get_logger().info(f"New task: {self.pending_task}")
        except Exception as e:
            self.get_logger().warn(f"Bad /task_cmd: {e}")

    def det_cb(self, dets: Detection2DArray):
        if not self.pending_task or not self.last_rgb: return
        if self.pending_task.get('color') != 'red': return
        img_msg, img = self.last_rgb
        h, w, _ = img.shape
        best = None; best_score = 0.0
        for d in dets.detections:
            x = int(d.bbox.center.x); y = int(d.bbox.center.y)
            sx = int(d.bbox.size_x);  sy = int(d.bbox.size_y)
            x1 = max(0, x - sx//4); y1 = max(0, y - sy//4)
            x2 = min(w, x + sx//4); y2 = min(h, y + sy//4)
            if x2<=x1 or y2<=y1: continue
            crop = img[y1:y2, x1:x2]
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, (0,120,70), (10,255,255))
            m2 = cv2.inRange(hsv, (170,120,70), (180,255,255))
            score = float(np.count_nonzero(cv2.bitwise_or(m1,m2))) / float(crop.size/3)
            if score > best_score: best_score = score; best = (x,y)
        if best and best_score > 0.15:
            ps = PoseStamped(); ps.header = img_msg.header; ps.header.frame_id = 'zed_left_camera_frame'
            ps.pose.position.x = 0.0; ps.pose.position.y = 0.0; ps.pose.position.z = 0.30
            self.target_pub.publish(ps)
            self.get_logger().info(f"Red confirmed (score={best_score:.2f}). Triggered selection.")
            self.pending_task = None

def main():
    rclpy.init(); rclpy.spin(TaskManager()); rclpy.shutdown()
