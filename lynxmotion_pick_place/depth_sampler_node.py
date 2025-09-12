import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

class DepthSampler(Node):
    def __init__(self):
        super().__init__('depth_sampler')
        self.bridge = CvBridge()
        self.depth = None
        self.fx = self.fy = self.cx = self.cy = None
        self.window = int(self.declare_parameter('window',7).value)
        self.topic_depth = self.declare_parameter('topic_depth','/zed/depth/depth_registered').get_parameter_value().string_value
        self.topic_info  = self.declare_parameter('topic_info','/zed/left/camera_info').get_parameter_value().string_value
        self.create_subscription(CameraInfo, self.topic_info,  self.info_cb, 10)
        self.create_subscription(Image, self.topic_depth, self.depth_cb, 10)
        self.create_subscription(Detection2DArray, '/detections', self.det_cb, 10)
        self.pub = self.create_publisher(PoseArray, '/target_candidates', 10)

    def info_cb(self, info: CameraInfo):
        K = info.k
        self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]

    def depth_cb(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def det_cb(self, dets: Detection2DArray):
        if self.depth is None or self.fx is None: return
        pa = PoseArray(); pa.header = dets.header
        h, w = self.depth.shape
        for d in dets.detections:
            u = int(round(d.bbox.center.x)); v = int(round(d.bbox.center.y))
            if u<0 or v<0 or u>=w or v>=h: continue
            w2 = self.window//2
            patch = self.depth[max(0,v-w2):v+w2+1, max(0,u-w2):u+w2+1]
            finite = patch[np.isfinite(patch)]
            if finite.size == 0: continue
            Z = float(np.median(finite))
            if not np.isfinite(Z) or Z <= 0 or Z > 1.5:  # 1.5 m clamp
                continue
            X = (u - self.cx) / self.fx * Z
            Y = (v - self.cy) / self.fy * Z
            pose = Pose(); pose.position.x=X; pose.position.y=Y; pose.position.z=Z
            pa.poses.append(pose)
        if pa.poses: self.pub.publish(pa)

def main():
    rclpy.init(); rclpy.spin(DepthSampler()); rclpy.shutdown()
