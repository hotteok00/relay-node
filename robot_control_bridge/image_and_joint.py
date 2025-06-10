import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from inference_interfaces.msg import ImageAndJoint as IJ

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

from collections import deque

from robot_control_bridge import config
IMAGE_RAW = config.IMAGE_RAW
JOINT_STATES = config.JOINT_STATES
IMAGE_AND_JOINT = config.IMAGE_AND_JOINT
namespace = config.namespace
max_queue_len = config.max_queue_len

REALSENSE_IMAGE = config.REALSENSE_IMAGE

# now = lambda: rclpy.time.Time(rclpy.clock.Clock().now()).to_msg()

def get_ros_time(msg):
    # sensor_msgs/Image, JointState 등에서 header.stamp
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

class ImageAndJoint(Node):
    def __init__(self):
        super().__init__('image_and_joint')

        # self.max_queue_len = 10
        self.image_raw_queue = deque(maxlen=max_queue_len)
        self.image_real_queue = deque(maxlen=max_queue_len)
        self.joint_queue = deque(maxlen=max_queue_len)

        self.joint_dict = {f'joint_{i}':i for i in range(1, 7)}

        self.prev_joint = [0,0,1.57,0,1.57,0]

        # self.create_subscription(Image, IMAGE_RAW, self.image_callback, 10)
        self.create_subscription(CompressedImage, IMAGE_RAW, self.image_raw_callback, QOS)
        self.create_subscription(CompressedImage, REALSENSE_IMAGE, self.image_real_callback, QOS)
        self.create_subscription(JointState, namespace+JOINT_STATES, self.joint_callback, QOS)

        self.publisher_ = self.create_publisher(IJ, IMAGE_AND_JOINT, 10)

        self.timer = self.create_timer(1, self.lazy_start)

    def lazy_start(self):
        self.create_timer(0.1, self.synchronize_queue)
        self.timer.cancel()  # Cancel the lazy start timer after the first call

    def image_raw_callback(self, msg):
        # self.get_logger().info(f"Image raw msg header.stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, ROS now: {self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec:09d}")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_raw_queue.append(msg)
        # self.synchronize_queue()

    def image_real_callback(self, msg):
        # self.get_logger().info(f"Image real msg header.stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, ROS now: {self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec:09d}")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_real_queue.append(msg)

    def joint_callback(self, msg):
        # self.get_logger().info(f"Joint msg header.stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, ROS now: {self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec:09d}")
        for name, position in dict(zip(msg.name, msg.position)).items():
            self.joint_dict[name] = position

        msg.name = list(self.joint_dict.keys())
        msg.position = list(self.joint_dict.values())
        msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_queue.append(msg)
        # self.synchronize_queue()

    def synchronize_queue(self):
        # 큐가 비었으면 리턴
        if len(self.joint_queue) == 0 or len(self.image_raw_queue) == 0 or len(self.image_real_queue) == 0:
            return

        # joint 기준으로 동기화
        joint_msg = self.joint_queue[0]
        joint_time = get_ros_time(joint_msg)

        # image_raw에서 joint_time에 가장 가까운 것 찾기
        best_raw_idx = None
        min_raw_diff = float('inf')
        for idx, img_msg in enumerate(self.image_raw_queue):
            img_time = get_ros_time(img_msg)
            diff = abs(joint_time - img_time)
            if diff < min_raw_diff:
                min_raw_diff = diff
                best_raw_idx = idx

        # 오래된 image는 버리기 (joint보다 너무 과거인 데이터는 pop)
        # while len(self.image_raw_queue) > 0 and get_ros_time(self.image_raw_queue[0]) < joint_time - threshold:
        #     self.image_raw_queue.popleft()

        # image_real도 동일하게
        best_real_idx = None
        min_real_diff = float('inf')
        for idx, img_msg in enumerate(self.image_real_queue):
            img_time = get_ros_time(img_msg)
            diff = abs(joint_time - img_time)
            if diff < min_real_diff:
                min_real_diff = diff
                best_real_idx = idx

        # while len(self.image_real_queue) > 0 and get_ros_time(self.image_real_queue[0]) < joint_time - threshold:
        #     self.image_real_queue.popleft()

        # 로그로 차이 출력
        self.get_logger().info(f"{min_raw_diff}, {min_real_diff}")

        joint_msg = self.joint_queue.popleft()
        image_raw_msg = self.image_raw_queue[best_raw_idx]
        del self.image_raw_queue[best_raw_idx]
        image_real_msg = self.image_real_queue[best_real_idx]
        del self.image_real_queue[best_real_idx]

        pub = IJ()
        pub.image_raw = image_raw_msg
        pub.image_real = image_real_msg
        pub.crnt_joint = joint_msg.position
        pub.next_joint = [a - b for a, b in zip(pub.crnt_joint, self.prev_joint)]
        self.publisher_.publish(pub)

        self.get_logger().info(
            f"[SYNC] joint stamp: {joint_time:.3f}, "
            f"raw_img stamp: {get_ros_time(image_raw_msg):.3f}, "
            f"real_img stamp: {get_ros_time(image_real_msg):.3f}, "
            f"joint name: {joint_msg.name}, "
            f"joint pos: {joint_msg.position}"
        )

        self.prev_joint = joint_msg.position


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
