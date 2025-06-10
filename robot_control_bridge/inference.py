# ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from inference_interfaces.msg import InferencePayload
from inference_interfaces.msg import ActionChunk
from cv_bridge import CvBridge

# colab
import requests as req
import numpy as np

from dotenv import load_dotenv
import os

# constant
from robot_control_bridge import config
# INFERENCE = "/inference"
# ACTION_CHUNK = "/action_chunk"
INFERENCE = config.INFERENCE
ACTION_CHUNK = config.ACTION_CHUNK
JOINT_COMMMAND = config.JOINT_COMMMAND

sample_action_chunk = [
    [0.05953603326504964, 0.2615764716171902, 0.07282771495818596, 0.2555080993741107, 0.034175989975872345, 0.36930163828633567, -0.1129511131982861, 0.032671105213359114], 
    [0.05563774290754342, 0.2615764716171902, 0.06363303666627418, 0.23771938910603618, 0.03693713395358378, 0.3506048192162253, -0.11939695250332316, -0.00514044283571408], 
    [0.04822275401370946, 0.2615764716171902, 0.048206690857023005, 0.20787443016524454, 0.042189141898447136, 0.33068425394197787, -0.13165766745141705, -0.0032509732019762303], 
    [0.03801689735974212, 0.2615764716171902, 0.031396673906974926, 0.17535251785550432, 0.04941791067992796, 0.32987923562009336, -0.14853309383802404, 0.007389813642882883], 
    [0.026019193303315924, 0.2615764716171902, 0.018186425564537367, 0.14979498632776733, 0.057915838043901435, 0.35424362038601565, -0.1683713473504138, -0.005293214572259627], 
    [0.013404060710393628, 0.2615764716171902, 0.011932028578949982, 0.13769476353084775, 0.0668510876514401, 0.39002722188246763, -0.1892305215135538, 0.006708914259352905], 
    [0.0014063566539675443, 0.2615764716171902, 0.013245706904108256, 0.14023630357931738, 0.07534901501541336, 0.4143916066483899, -0.20906877502594357, 0.01157944264966375], 
    [-0.008799499999999905, 0.2615764716171902, 0.019935896952341103, 0.15317964413219565, 0.08257778379689418, 0.4135865883265053, -0.22594420141255056, -0.0032509732019762303], 
    [-0.016214488893833756, 0.2615764716171902, 0.02800975814137069, 0.16879993805402205, 0.08782979174175765, 0.39366602305225795, -0.23820491636064456, 0.09674949373248935],
    [-0.02011277925134003, 0.2615764716171902, 0.03333900651705468, 0.17911029936215417, 0.09059093571946919, 0.3749692039821475, -0.2446507556656815, 0.2764517022888696]
]

class ColabInference():
    _SAMPLE_PAYLOAD = {
        "observation/exterior_image_1_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/joint_position": np.random.rand(7),
        "observation/gripper_position": np.random.rand(1),
        "prompt": "do something",
    }

    def __init__(self):
        load_dotenv(os.path.expanduser('~/ros2_ws/src/robot_control_bridge/.env'))
        self.url = os.getenv("url")
        print(f'url: {self.url}')
        
        res = req.get(url=self.url)
        self.code = res.status_code

    def _to_serializable(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self._to_serializable(v) for k, v in obj.items()}
        return obj
        
    def _show_shape(self, obj):
        if isinstance(obj, dict):
            return {k: [type(v), self._show_shape(v)] for k, v in obj.items()}
        elif isinstance(obj, list):
            if not obj:
                return [0]
            rest = self._show_shape(obj[0])
            return [len(obj)] + (rest if isinstance(rest, list) else [rest])
        elif isinstance(obj, np.ndarray):
            return list(obj.shape)
        return []
    
    def _send_payload(self, payload):
        print('payload =', self._show_shape(payload))
        json_payload = self._to_serializable(payload)
        print('json_payload =', self._show_shape(json_payload))

        res = req.post(url=self.url+'/infer', json=json_payload)
        action_chunk = res.json()['result']['actions']

        print(f'status: {res.status_code}, type & shape: {self._show_shape(res.json())}')
        for action in action_chunk: print(action)

        return action_chunk

    def get_keys(self):
        return [k for k in self._SAMPLE_PAYLOAD.keys()]
    
    def inferenc_by_code(self, payload):
        if self.code == 200:
            return self._send_payload(payload)
        else:
            return sample_action_chunk

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
        self.publisher_ac = self.create_publisher(ActionChunk, ACTION_CHUNK, 10)

        self.publisher_jc = self.create_publisher(Float64MultiArray, JOINT_COMMMAND, 10)

    def payload_callback(self, sub):
        # payload 재구성
        # payload = {     # droid
        #     "observation/exterior_image_1_left": self.bridge.imgmsg_to_cv2(sub.exterior_image_1_left, desired_encoding='bgr8'),
        #     "observation/wrist_image_left": self.bridge.imgmsg_to_cv2(sub.wrist_image_left, desired_encoding='bgr8'),
        #     "observation/joint_position": np.array(sub.joint_position, dtype=np.float64),
        #     "observation/gripper_position": np.array([sub.gripper_position], dtype=np.float64),
        #     "prompt": sub.prompt,
        # }

        payload = {     # libero
            "observation/state": np.concatenate((sub.joint_position, [0, sub.gripper_position]), dtype=np.float64),
            "observation/image": self.bridge.imgmsg_to_cv2(sub.exterior_image_1_left, desired_encoding='bgr8'),
            "observation/wrist_image": self.bridge.imgmsg_to_cv2(sub.wrist_image_left, desired_encoding='bgr8'),
            "prompt": sub.prompt,
        }

        action_chunk = self.colab_inference.inferenc_by_code(payload)
        self.get_logger().info(f'action_chunk: {action_chunk}')

        # action chunk
        pub = ActionChunk()
        pub.rows, pub.cols = len(action_chunk), len(action_chunk[0])
        pub.data = [action for action_token in action_chunk for action in action_token]

        self.publisher_ac.publish(pub)

        for action_token in action_chunk:
            pub = Float64MultiArray()
            pub.data = action_token[:6]
            self.publisher_jc.publish(pub)
            input('if done')

def main(args=None):
    rclpy.init(args=args)
    node = Inference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()