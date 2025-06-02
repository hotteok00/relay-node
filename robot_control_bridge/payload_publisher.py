import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from inference_interfaces.msg import InferencePayload
from cv_bridge import CvBridge

import numpy as np
import cv2
import json

from dotenv import load_dotenv
import os

# constant
INFERENCE = "/inference"

class PayloadPublisher(Node):
    def __init__(self):
        super().__init__('payload_publisher')
        self.publisher = self.create_publisher(InferencePayload, INFERENCE, 10)
        self.bridge = CvBridge()

        load_dotenv('/home/lhj/ros2_ws/src/robot_control_bridge/.env')
        self.img_path=os.getenv('img_path')
        with open(os.path.join(self.img_path, 'data.json'), 'r') as f:
            self.joint_status = json.load(f)
        self.img_idx = 0

    def publish_once_rand(self):
        msg = InferencePayload()

        # 1. 이미지 생성
        dummy_img = np.random.randint(0, 256, (224, 224, 3), dtype=np.uint8)
        msg.exterior_image_1_left = self.bridge.cv2_to_imgmsg(dummy_img, encoding='bgr8')
        msg.wrist_image_left = self.bridge.cv2_to_imgmsg(dummy_img, encoding='bgr8')

        # 2. 조인트/그리퍼/프롬프트
        msg.joint_position = [0.1 * i for i in range(7)]
        msg.gripper_position = 0.5
        msg.prompt = "do something"

        self.publisher.publish(msg)
        self.get_logger().info("✅ Sent InferencePayload : Rand")

    def publish_once_real(self):
        msg = InferencePayload()

        path1 = self.img_path + f'/observation_exterior_image_1_left_{self.img_idx}.png'
        path2 = self.img_path + f'/observation_wrist_image_left_{self.img_idx}.png'

        img1 = cv2.imread(path1)
        img2 = cv2.imread(path2)

        # 1. 이미지 읽기
        msg.exterior_image_1_left = self.bridge.cv2_to_imgmsg(img1, encoding='bgr8')
        msg.wrist_image_left = self.bridge.cv2_to_imgmsg(img2, encoding='bgr8')

        # 2. 조인트/그리퍼/프롬프트
        data = self.joint_status[self.img_idx]
        msg.joint_position = data["joint_position"]
        msg.gripper_position = data["gripper_position"]
        msg.prompt = "fold the blanket"
        # msg.prompt = input('input prompt: ')

        self.publisher.publish(msg)
        self.get_logger().info(f"✅ Sent InferencePayload : Real_{self.img_idx}")

        self.img_idx = (self.img_idx + 1) % 10

def main(args=None):
    rclpy.init(args=args)
    node = PayloadPublisher()

    try:
        while rclpy.ok():
            option = input("🔁 real | rand : ")
            if option == 'real':
                node.publish_once_real()
            elif option == 'rand':
                node.publish_once_rand()
            else:
                print("wrong input: select [real | rand]")
    except KeyboardInterrupt:
        print("\n🛑 종료됨.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()