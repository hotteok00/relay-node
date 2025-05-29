import rclpy
from rclpy.node import Node
from inference_interfaces.msg import ActionChunk
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import time

JOINT_STATES = "/joint_states"
ACTION_CHUNK = "/action_chunk"
JOINT_TRAJECTORY = "/dsr_moveit_controller/joint_trajectory"
SECONDS = 1

class ActionChunkToTrajectory(Node):
    def __init__(self):
        super().__init__('action_chunk_to_trajectory')

        self.joint_position = []
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]

        self.create_subscription(JointState, JOINT_STATES, self.state_callback, 10)

        self.create_subscription(ActionChunk, ACTION_CHUNK, self.action_chunk_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, JOINT_TRAJECTORY, 10)

        self.get_logger().info("✅ ActionChunk → JointTrajectory node started")

    def state_callback(self, msg):
        self.joint_position = list(msg.position)

    def action_chunk_callback(self, sub):
        if sub.rows * sub.cols != len(sub.data):
            self.get_logger().error("❌ Mismatch between rows, cols, and data length")
            return

        self.get_logger().info("Start Publish Joint Traejctory")

        pub = JointTrajectory()
        pub.header = Header()
        pub.joint_names = self.joint_names

        for i in range(sub.rows):
            offset = i * sub.cols
            point = JointTrajectoryPoint()
            # point.positions = sub.data[offset:offset+sub.cols]

            # 3번째(index 2)와 마지막(index -1)은 제외
            raw_joint_values = sub.data[offset : offset+sub.cols]
            filtered_joint_values = [v for i, v in enumerate(raw_joint_values) if i != 2 and i != len(raw_joint_values) - 1]
            point.positions = [x + y for x, y in zip(self.joint_position, filtered_joint_values)]

            point.time_from_start = Duration(sec=i * SECONDS, nanosec=0)
            
            pub.points.append(point)
            self.publisher.publish(pub)
            pub.points.pop()

            # time.sleep(SECONDS * 2)
            input("동작 완료 시, enter")

        # pub.points = pub.points[:1]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        pub.points.append(point)
        self.publisher.publish(pub)

        self.get_logger().info(f"📤 Published trajectory with {sub.rows} points")            

def main(args=None):
    rclpy.init(args=args)
    node = ActionChunkToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()