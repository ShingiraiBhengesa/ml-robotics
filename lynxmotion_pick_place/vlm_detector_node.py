import rclpy, torch, ast
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

class VLMDINO(Node):
    def __init__(self):
        super().__init__('vlm_detector')
        self.bridge = CvBridge()
        self.model_id = self.declare_parameter('model_id','IDEA-Research/grounding-dino-tiny').get_parameter_value().string_value
        self.box_threshold = float(self.declare_parameter('box_threshold',0.35).value)
        self.text_threshold = float(self.declare_parameter('text_threshold',0.25).value)
        self.topic_rgb = self.declare_parameter('topic_rgb','/zed/left/image_rect_color').get_parameter_value().string_value
        self.prompts = ["a red ball"]
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(dev).eval()
        self.create_subscription(Image, self.topic_rgb, self.on_image, 10)
        self.create_subscription(String, '/task_cmd', self.on_cmd, 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.get_logger().info(f'VLM={self.model_id} on {dev}')

    def on_cmd(self, msg: String):
        try:
            d = ast.literal_eval(msg.data)
            color, obj = d.get('color'), d.get('object')
            if color and obj: self.prompts = [f"a {color} {obj}"]
        except Exception as e:
            self.get_logger().warn(f'Bad /task_cmd: {e}')

    @torch.inference_mode()
    def on_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')[:, :, ::-1]
        inputs = self.processor(images=img, text=[self.prompts], return_tensors="pt").to(self.model.device)
        outputs = self.model(**inputs)
        results = self.processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            threshold=self.box_threshold, text_threshold=self.text_threshold,
            target_sizes=[img.shape[:2]]
        )[0]
        out = Detection2DArray(); out.header = msg.header
        for box, score, labels in zip(results["boxes"], results["scores"], results["labels"]):
            x1,y1,x2,y2 = box.tolist()
            det = Detection2D()
            det.bbox.center.x, det.bbox.center.y = (x1+x2)/2.0, (y1+y2)/2.0
            det.bbox.size_x, det.bbox.size_y = (x2-x1), (y2-y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = labels
            hyp.hypothesis.score = float(score.item() if hasattr(score, "item") else score)
            det.results.append(hyp); out.detections.append(det)
        if out.detections: self.pub_det.publish(out)

def main():
    rclpy.init(); rclpy.spin(VLMDINO()); rclpy.shutdown()
