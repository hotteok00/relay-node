import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        # MoveIt에서 오는 액션 goal을 받는 서버
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/dsr_moveit_controller/follow_joint_trajectory',
            self.goal_callback
        )
        # Gazebo에 명령 publish
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/dsr01/gz/dsr_position_controller/commands',
            10
        )

    def goal_callback(self, goal_handle):
        self.get_logger().info('Trajectory goal received from MoveIt!')
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            self.get_logger().error('Received empty trajectory!')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # 마지막 목표 point만 사용
        last_point = trajectory.points[-1]
        positions = last_point.positions

        # Gazebo로 publish
        msg = Float64MultiArray()
        msg.data = list(positions)
        self.get_logger().info(f'Publishing to Gazebo: {msg.data}')
        self.publisher.publish(msg)

        # (필요하다면) 약간의 sleep으로 명령 전달 대기 후 succeed
        # import time
        # time.sleep(0.2)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()