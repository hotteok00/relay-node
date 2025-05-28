# ros2
import rclpy
from rclpy.node import Node
from inference_interfaces.msg import InferencePayload

from cv_bridge import CvBridge

# colab
import requests as req
import numpy as np

from dotenv import load_dotenv
import os

# constant
INFERENCE = "/inference"

class ColabInference():
    _SAMPLE_PAYLOAD = {
        "observation/exterior_image_1_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/joint_position": np.random.rand(7),
        "observation/gripper_position": np.random.rand(1),
        "prompt": "do something",
    }

    def get_keys(self):
        return [k for k in self._SAMPLE_PAYLOAD.keys()]

    def __init__(self):
        load_dotenv('/home/lhj/ros2_ws/src/robot_control_bridge/.env')
        self.url = os.getenv("url")
        print(f'url: {self.url}')
        
        # init infer server
        self.send_payload(self._SAMPLE_PAYLOAD)

    def to_serializable(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self.to_serializable(v) for k, v in obj.items()}
        return obj
        
    def show_shape(self, obj):
        if isinstance(obj, dict):
            return {k: [type(v), self.show_shape(v)] for k, v in obj.items()}
        elif isinstance(obj, list):
            if not obj:
                return [0]
            rest = self.show_shape(obj[0])
            return [len(obj)] + (rest if isinstance(rest, list) else [rest])
        elif isinstance(obj, np.ndarray):
            return list(obj.shape)
        return []
    
    def send_payload(self, payload):
        print('payload =', self.show_shape(payload))
        json_payload = self.to_serializable(payload)
        print('json_payload =', self.show_shape(json_payload))

        # res = req.get(url=self.url)
        res = req.post(url=self.url+'/infer', json=json_payload)
        action_chunk = res.json()['result']['actions']

        print(f'status: {res.status_code}, type & shape: {self.show_shape(res.json())}')
        for action in action_chunk: print(action)

        return action_chunk

class Inference(Node):
    def __init__(self):
        super().__init__('inference')

        self.colab_inference = ColabInference()
        self.keys = self.colab_inference.get_keys()
        self.get_logger().info(f'keys: {self.keys}')

        self.bridge = CvBridge()

        # sub for payload
        self.create_subscription(InferencePayload, INFERENCE, self.payload_callback, 10)

        # pub for action_chunk
        # self.publisher = self.create_publisher()

    def payload_callback(self, sub):
        # payload 재구성
        payload = {
            "observation/exterior_image_1_left": self.bridge.imgmsg_to_cv2(sub.exterior_image_1_left, desired_encoding='bgr8'),
            "observation/wrist_image_left": self.bridge.imgmsg_to_cv2(sub.wrist_image_left, desired_encoding='bgr8'),
            "observation/joint_position": np.array(sub.joint_position, dtype=np.float64),
            "observation/gripper_position": np.array([sub.gripper_position], dtype=np.float64),
            "prompt": sub.prompt,
        }

        action_chunk = self.colab_inference.send_payload(payload)
        self.get_logger().info(f'action_chunk: {action_chunk}')

def main(args=None):
    rclpy.init(args=args)
    node = Inference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()