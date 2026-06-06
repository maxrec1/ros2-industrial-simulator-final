#!/usr/bin/env python3
"""Send a single joint-position command to the SCARA robot via FollowJointTrajectory."""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4']
DEFAULT_TIMING_SEC = 5
ACTION_NAME = '/arm_trajectory_controller/follow_joint_trajectory'


class MoveJointsClient(Node):
    def __init__(self, positions: list[float]) -> None:
        super().__init__('move_joints_client')
        self._positions = positions
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def send(self) -> bool:
        self.get_logger().info(f'Waiting for action server {ACTION_NAME} ...')
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = self._positions
        point.time_from_start = Duration(sec=DEFAULT_TIMING_SEC, nanosec=0)
        goal.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {self._positions}')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error('Failed to send goal')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=float(DEFAULT_TIMING_SEC) + 10.0
        )

        if not result_future.done() or result_future.result() is None:
            self.get_logger().error('No result received')
            return False

        error_code = result_future.result().result.error_code
        if error_code != 0:
            self.get_logger().error(f'Motion failed with error code {error_code}')
            return False

        self.get_logger().info('Motion completed successfully')
        return True


def main():
    # Strip leading '--' separator that ros2 run passes before user args
    args = sys.argv[1:]
    if args and args[0] == '--':
        args = args[1:]

    if len(args) != len(JOINT_NAMES):
        print(
            f'Usage: ros2 run scara_robot_pkg move_joints -- '
            + ' '.join(f'<{j}>' for j in JOINT_NAMES)
        )
        sys.exit(1)

    try:
        positions = [float(v) for v in args]
    except ValueError as exc:
        print(f'Error: all positions must be numbers ({exc})')
        sys.exit(1)

    rclpy.init()
    node = MoveJointsClient(positions)
    try:
        success = node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
