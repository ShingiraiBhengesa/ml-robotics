#!/usr/bin/env python3
import rclpy, sys
from rclpy.node import Node
from std_msgs.msg import String

def parse(cmd):
    cmd = cmd.lower()
    color = next((c for c in ["red","blue","green","yellow"] if c in cmd), None)
    obj   = "ball" if "ball" in cmd else None
    act   = "pick" if "pick" in cmd else ("place" if "place" in cmd else "pick")
    return {"action": act, "object": obj, "color": color}

class CLI(Node):
    def __init__(self, msg):
        super().__init__('arm_cli')
        pub = self.create_publisher(String, '/task_cmd', 10)
        pub.publish(String(data=str(msg)))
        self.get_logger().info(f"Sent task: {msg}")

if __name__ == "__main__":
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: arm_cli.py "pick up the red ball"'); sys.exit(1)
    CLI(parse(" ".join(sys.argv[1:])))
    rclpy.shutdown()
