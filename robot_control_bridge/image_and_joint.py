import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from collections import deque

IMAGE_RAW = '/image_raw'
JOINT_STATES = '/joint_states'

def get_ros_time(msg):
    # sensor_msgs/Image, JointState 등에서 header.stamp
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

class ImageAndJoint(Node):
    def __init__(self):
        super().__init__('image_and_joint')

        self.create_subscription(Image, IMAGE_RAW, self.image_callback, 10)
        self.create_subscription(JointState, JOINT_STATES, self.joint_callback, 10)

        self.max_queue_len = 10
        self.image_queue = deque(maxlen=self.max_queue_len)
        self.joint_queue = deque(maxlen=self.max_queue_len)

    def image_callback(self, msg):
        self.image_queue.append(msg)
        self.synchronize_queue()

    def joint_callback(self, msg):
        self.joint_queue.append(msg)
        self.synchronize_queue()

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
            joint_msg = self.joint_queue[best_idx]
            # 매칭된 메시지 큐에서 삭제
            self.image_queue.popleft()
            del self.joint_queue[best_idx]
            # 로그 출력
            self.get_logger().info(
                f"[SYNC] img stamp: {img_time:.3f}, joint stamp: {get_ros_time(joint_msg):.3f}, "
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
