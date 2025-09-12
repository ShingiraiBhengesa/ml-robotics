import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from .utils.ik_core import calculate_ik_mm_deg

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.grip_angle = float(self.declare_parameter('grip_angle_deg', 90.0).value)
        self.create_subscription(PoseStamped, '/selected_target', self.cb, 10)
        self.pub = self.create_publisher(JointState, '/arm/joint_cmd', 10)

    def cb(self, ps: PoseStamped):
        X = ps.pose.position.x * 1000.0
        Y = ps.pose.position.y * 1000.0
        Z = ps.pose.position.z * 1000.0
        ik_deg = calculate_ik_mm_deg(X, Y, Z, self.grip_angle)
        if ik_deg is None:
            self.get_logger().warn('IK failed/outside limits'); return
        js = JointState(); js.header = Header()
        js.name = ['base','shoulder','elbow','wrist','gripper']
        js.position = [
            math.radians(ik_deg['base']),
            math.radians(ik_deg['shoulder']),
            math.radians(ik_deg['elbow']),
            math.radians(ik_deg['wrist']),
            math.radians(10.0)
        ]
        self.pub.publish(js)

def main():
    rclpy.init(); rclpy.spin(IKNode()); rclpy.shutdown()
