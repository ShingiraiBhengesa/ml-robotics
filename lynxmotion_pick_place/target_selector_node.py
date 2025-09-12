import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class TargetSelector(Node):
    def __init__(self):
        super().__init__('target_selector')
        self.base = self.declare_parameter('base_frame','base_link').get_parameter_value().string_value
        self.camera_frame = self.declare_parameter('camera_frame','zed_left_camera_frame').get_parameter_value().string_value
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)
        self.create_subscription(PoseArray, '/target_candidates', self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/selected_target', 10)

    def cb(self, pa: PoseArray):
        if not pa.poses: return
        best = min(pa.poses, key=lambda p: p.position.x**2 + p.position.y**2 + p.position.z**2)
        ps = PoseStamped(); ps.header = pa.header; ps.header.frame_id = self.camera_frame; ps.pose = best
        try:
            out = self.buf.transform(ps, self.base, timeout=Duration(seconds=0.2))
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")

def main():
    rclpy.init(); rclpy.spin(TargetSelector()); rclpy.shutdown()
