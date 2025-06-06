import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from inference_interfaces.msg import ImageAndJoint as IJ

from collections import deque

from robot_control_bridge import config
# IMAGE_RAW = '/image_raw'
# JOINT_STATES = '/joint_states'
# IMAGE_AND_JOINT = '/image_and_joint'
IMAGE_RAW = config.IMAGE_RAW
JOINT_STATES = config.JOINT_STATES
IMAGE_AND_JOINT = config.IMAGE_AND_JOINT
max_queue_len = config.max_queue_len

def get_ros_time(msg):
    # sensor_msgs/Image, JointState 등에서 header.stamp
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

class ImageAndJoint(Node):
    def __init__(self):
        super().__init__('image_and_joint')

        # self.max_queue_len = 10
        self.image_queue = deque(maxlen=max_queue_len)
        self.joint_queue = deque(maxlen=max_queue_len)

        self.joint_dict = {f'joint_{i}':i for i in range(1, 7)}

        # self.create_subscription(Image, IMAGE_RAW, self.image_callback, 10)
        self.create_subscription(CompressedImage, IMAGE_RAW, self.image_callback, 10)
        self.create_subscription(JointState, JOINT_STATES, self.joint_callback, 10)

        self.publisher_ = self.create_publisher(IJ, IMAGE_AND_JOINT, 10)

        self.create_timer(1, self.synchronize_queue)

    def image_callback(self, msg):
        self.image_queue.append(msg)
        # self.synchronize_queue()

    def joint_callback(self, msg):
        for name, position in dict(zip(msg.name, msg.position)).items():
            self.joint_dict[name] = position

        msg.name = list(self.joint_dict.keys())
        msg.position = list(self.joint_dict.values())

        self.joint_queue.append(msg)
        # self.synchronize_queue()

    def synchronize_queue(self):
        if len(self.image_queue) == 0 or len(self.joint_queue) == 0:
            return

        img_msg = self.image_queue[0]
        img_time = get_ros_time(img_msg)

        # joint 큐에서 가장 가까운 시간 메시지 찾기
        best_idx = None
        min_time_diff = float('inf')
        for idx, joint_msg in enumerate(self.joint_queue):
            joint_time = get_ros_time(joint_msg)
            time_diff = abs(img_time - joint_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                best_idx = idx

        if min_time_diff < 0.05:
            # 매칭된 메시지 큐에서 삭제
            image_msg = self.image_queue.popleft()
            joint_msg = self.joint_queue[best_idx]
            del self.joint_queue[best_idx]
            
            pub = IJ()
            pub.image = image_msg
            pub.joint = joint_msg.position
            self.publisher_.publish(pub)
            
            # 로그 출력
            self.get_logger().info(
                f"[SYNC] img stamp: {img_time:.3f}, joint stamp: {get_ros_time(joint_msg):.3f}, "
                f"joint name: {joint_msg.name}"
                f"joint pos: {joint_msg.position}"
            )

def main():
    rclpy.init()
    node = ImageAndJoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
