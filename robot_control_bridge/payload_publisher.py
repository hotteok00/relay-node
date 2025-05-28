import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from inference_interfaces.msg import InferencePayload
from cv_bridge import CvBridge
import numpy as np

# constant
INFERENCE = "/inference"

class PayloadPublisher(Node):
    def __init__(self):
        super().__init__('payload_publisher')
        self.publisher = self.create_publisher(InferencePayload, INFERENCE, 10)
        self.bridge = CvBridge()

    def publish_once(self):
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
        self.get_logger().info("✅ Sent InferencePayload")

def main(args=None):
    rclpy.init(args=args)
    node = PayloadPublisher()

    try:
        while rclpy.ok():
            input("🔁 엔터를 누르면 메시지를 보냅니다... ")
            node.publish_once()
    except KeyboardInterrupt:
        print("\n🛑 종료됨.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()