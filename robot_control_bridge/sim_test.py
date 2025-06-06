import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from inference_interfaces.msg import ActionChunk
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time
from collections import deque
import threading

from robot_control_bridge import config
# JOINT_STATES = "/dsr_moveit_controller/controller_state"
# ACTION_CHUNK = "/action_chunk"
# JOINT_TRAJECTORY = "/dsr_moveit_controller/joint_trajectory"
ACTION_CHUNK = config.ACTION_CHUNK
# JOINT_STATES = config.MOVEIT_JOINT_STATES
JOINT_STATES = config.JOINT_STATES
JOINT_TRAJECTORY = config.JOINT_TRAJECTORY
max_queue_len = config.max_queue_len

SECONDS = 1

class ActionChunkToTrajectory(Node):
    def __init__(self):
        super().__init__('action_chunk_to_trajectory')

        self.joint_position = deque(maxlen=max_queue_len)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]

        self.joint_dict = {name:i for i, name in enumerate(self.joint_names)}

        self.lock = threading.Lock()
        self.data_ready = threading.Condition(self.lock)

        # self.create_subscription(JointTrajectoryControllerState, JOINT_STATES, self.state_callback, 10)
        self.create_subscription(JointState, JOINT_STATES, self.state_callback, 10)
        self.create_subscription(ActionChunk, ACTION_CHUNK, self.action_chunk_callback, 10)
        
        self.publisher = self.create_publisher(JointTrajectory, JOINT_TRAJECTORY, 10)

        # set home
        pub = JointTrajectory()
        pub.header = Header()
        pub.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        # point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.points.append(point)
        
        self.publisher.publish(pub)

        self.get_logger().info("✅ ActionChunk → JointTrajectory node started")

    def state_callback(self, msg):
        self.get_logger().info(f'{__name__}')

        for name, position in dict(zip(msg.name, msg.position)).items():
            self.joint_dict[name] = position

        msg.name = list(self.joint_dict.keys())
        msg.position = list(self.joint_dict.values())
        # self.joint_position = list(msg.output.positions)
        self.get_logger().info(f'current_position {msg.position}')

        with self.data_ready:
            self.joint_position.append(msg.position)
            self.data_ready.notify()

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

            # ======================================================== #

            # 3번째(index 2)와 마지막(index -1)은 제외
            raw_joint_values = sub.data[offset : offset+sub.cols]
            filtered_joint_values = [v for i, v in enumerate(raw_joint_values) if i != 2 and i != len(raw_joint_values) - 1]
            
            # insert convert logic
            # 1. get current posx (6DoF) to get IK 7DOF joints
            # 2. compute FK 7DOF joints with action chunk (7j + 1g)
            # 3. convert IK 7DoF to 6DoF joint
            
            # ======================================================== #
            
            with self.data_ready:
                while not self.joint_position:
                    self.data_ready.wait()
                current_position = self.joint_position.popleft()
            self.get_logger().info(f"current posistion: {current_position}")
            self.get_logger().info(f"filtered joint values: {filtered_joint_values}")
            point.positions = [x + y for x, y in zip(current_position, filtered_joint_values)]

            point.time_from_start = Duration(sec=i * SECONDS, nanosec=0)
            
            pub.points.append(point)
            self.publisher.publish(pub)
            self.get_logger().info(f"published topic: {pub}")

            pub.points.pop()

            with self.data_ready:
                time.sleep(SECONDS * 2)
            # input("동작 완료 시, enter")

        self.get_logger().info(f"📤 Published trajectory with {sub.rows} points")            

def main(args=None):
    rclpy.init(args=args)
    node = ActionChunkToTrajectory()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()