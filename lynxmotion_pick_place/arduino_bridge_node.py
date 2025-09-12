import rclpy, math, serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.port = self.declare_parameter('port','COM3').get_parameter_value().string_value
        self.baud = int(self.declare_parameter('baud',115200).value)
        self.move_time = int(self.declare_parameter('move_time_ms',400).value)
        self.dry_run = bool(self.declare_parameter('dry_run', True).value)
        self.ser = None
        if not self.dry_run:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.05)
        self.create_subscription(JointState, '/arm/joint_cmd', self.cb, 10)
        self.get_logger().info(f'ArduinoBridge dry_run={self.dry_run} port={self.port}')

    def cb(self, js: JointState):
        idx = {n:i for i,n in enumerate(js.name)}
        def deg(name):
            i = idx.get(name, None);  return None if i is None else math.degrees(js.position[i])
        vals = [deg(n) for n in ['base','shoulder','elbow','wrist','gripper']]
        if any(v is None for v in vals): return
        line = f"J {vals[0]:.1f} {vals[1]:.1f} {vals[2]:.1f} {vals[3]:.1f} {vals[4]:.1f} T{self.move_time}\n"
        if self.dry_run:  self.get_logger().info(f"[DRY] {line.strip()}")
        else:
            try: self.ser.write(line.encode('ascii'))
            except Exception as e: self.get_logger().warn(f"Serial write failed: {e}")

def main():
    rclpy.init(); rclpy.spin(ArduinoBridge()); rclpy.shutdown()
