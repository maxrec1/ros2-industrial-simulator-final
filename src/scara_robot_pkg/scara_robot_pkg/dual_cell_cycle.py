#!/usr/bin/env python3

import math
import os
import random
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from conveyorbelt_msgs.srv import ConveyorBeltControl
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from linkattacher_msgs.srv import AttachLink, DetachLink
except ImportError:
    AttachLink = None
    DetachLink = None


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DualCellCycle(Node):
    def __init__(self) -> None:
        super().__init__('dual_cell_cycle')

        default_config = os.path.join(
            get_package_share_directory('scara_robot_pkg'),
            'config',
            'dual_cell_sequence.yaml',
        )
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('chip_name', 'chip1')
        self.declare_parameter('belt_run_power', 20.0)
        self.declare_parameter('spawn_randomized', True)

        self._chip_name = self.get_parameter('chip_name').value
        self._belt_run_power = float(self.get_parameter('belt_run_power').value)
        self._spawn_randomized = bool(self.get_parameter('spawn_randomized').value)

        with open(self.get_parameter('config_path').value, 'r', encoding='utf-8') as f:
            self._config = yaml.safe_load(f)

        self._scara_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory',
        )
        self._bobby_client = ActionClient(
            self, FollowJointTrajectory,
            '/bobby_arm_controller/follow_joint_trajectory',
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/bobby_gripper_controller/follow_joint_trajectory',
        )

        self._belt1_client = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        self._belt2_client = self.create_client(ConveyorBeltControl, '/belt2/CONVEYORPOWER')
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._use_link_attacher = AttachLink is not None and DetachLink is not None
        if self._use_link_attacher:
            self._attach_client = self.create_client(AttachLink, '/ATTACHLINK')
            self._detach_client = self.create_client(DetachLink, '/DETACHLINK')
        else:
            self._attach_client = None
            self._detach_client = None
            self.get_logger().warn(
                'linkattacher_msgs is not available. Running in teleport fallback mode.'
            )

        self._belt1_ready = False
        self._belt2_ready = False
        self.create_subscription(Bool, 'belt1/object_ready', self._on_belt1_ready, 10)
        self.create_subscription(Bool, 'belt2/object_ready', self._on_belt2_ready, 10)

    def _on_belt1_ready(self, msg: Bool) -> None:
        if msg.data:
            self._belt1_ready = True

    def _on_belt2_ready(self, msg: Bool) -> None:
        if msg.data:
            self._belt2_ready = True

    def _wait_ready(self) -> None:
        services = [
            (self._belt1_client, '/CONVEYORPOWER'),
            (self._belt2_client, '/belt2/CONVEYORPOWER'),
            (self._spawn_client, '/spawn_entity'),
            (self._delete_client, '/delete_entity'),
        ]
        if self._use_link_attacher:
            services.extend([
                (self._attach_client, '/ATTACHLINK'),
                (self._detach_client, '/DETACHLINK'),
            ])
        for client, name in services:
            if not client.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f'Service not available: {name}')
        for client, name in [
            (self._scara_client, 'SCARA controller'),
            (self._bobby_client, 'bobby controller'),
            (self._gripper_client, 'bobby gripper controller'),
        ]:
            if not client.wait_for_server(timeout_sec=20.0):
                raise RuntimeError(f'Action server not available: {name}')

    def _set_belt(self, client, power: float, label: str) -> None:
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f'Could not set {label} belt power to {power}')
        self.get_logger().info(f'{label} belt power = {power}')

    def _send_stage(self, client, joint_names, positions, seconds: float, label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start = Duration(
            sec=int(seconds),
            nanosec=int((seconds % 1.0) * 1e9),
        )
        goal.trajectory.points = [point]

        self.get_logger().info(f'Executing {label}: {point.positions}')
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError(f'Failed to send {label}')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f'Goal rejected: {label}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(10.0, seconds + 5.0))
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError(f'No result for {label}')
        result = result_future.result().result
        if result.error_code != 0:
            raise RuntimeError(f'{label} failed with error code {result.error_code}')

    def _stage(self, robot: str, name: str) -> None:
        cfg = self._config[robot]
        timing = float(self._config['timing']['default'])
        client = self._scara_client if robot == 'scara' else self._bobby_client
        self._send_stage(client, cfg['joint_names'], cfg['stages'][name], timing, f'{robot}.{name}')

    def _gripper(self, name: str) -> None:
        cfg = self._config['gripper']
        timing = float(self._config['timing'].get('gripper', 1.0))
        self._send_stage(self._gripper_client, cfg['joint_names'], cfg[name], timing, f'gripper.{name}')

    def _delete_chip(self, required: bool = False) -> None:
        req = DeleteEntity.Request()
        req.name = self._chip_name
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if required and (not future.done() or future.result() is None or not future.result().success):
            raise RuntimeError(f'Could not delete {self._chip_name}')

    def _spawn_chip_at(self, x: float, y: float, z: float, yaw: float, label: str) -> None:
        share = get_package_share_directory('conveyorbelt_gazebo')
        urdf_path = os.path.join(share, 'urdf', 'chip.urdf')
        with open(urdf_path, 'r', encoding='utf-8') as f:
            xml = f.read().replace('package://conveyorbelt_gazebo/', f'file://{share}/')

        req = SpawnEntity.Request()
        req.name = self._chip_name
        req.xml = xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 1.2
        req.initial_pose.orientation = yaw_to_quat(yaw)
        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            message = future.result().status_message if future.done() and future.result() else 'no response'
            raise RuntimeError(f'Could not spawn {self._chip_name}: {message}')
        self.get_logger().info(
            f'Spawned {self._chip_name} for {label} at x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.2f}'
        )

    def _spawn_chip(self) -> None:
        self._delete_chip(required=False)
        x = random.uniform(-0.13, 0.13) if self._spawn_randomized else 0.0
        y = random.uniform(-0.58, -0.42) if self._spawn_randomized else -0.5
        yaw = random.uniform(-math.pi, math.pi) if self._spawn_randomized else 0.0
        self._spawn_chip_at(x, y, 1.2, yaw, 'belt1 input')

    def _attach(self, robot_link: str, label: str) -> None:
        if not self._use_link_attacher:
            self._delete_chip(required=True)
            self.get_logger().info(f'Fallback picked {self._chip_name} ({label})')
            return

        req = AttachLink.Request()
        req.model1_name = 'combined_cell'
        req.link1_name = robot_link
        req.model2_name = self._chip_name
        req.link2_name = 'base_link_chip'
        future = self._attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Attach failed: {label}')
        self.get_logger().info(f'Attached {self._chip_name} to {robot_link} ({label})')

    def _detach(self, robot_link: str, label: str) -> None:
        if not self._use_link_attacher:
            if label == 'SCARA place on belt2':
                self._spawn_chip_at(-1.3, -1.0, 1.2, 0.0, 'belt2 handoff')
            else:
                self._spawn_chip_at(0.0, 1.45, 1.05, 0.0, 'drop box')
            self.get_logger().info(f'Fallback placed {self._chip_name} ({label})')
            return

        req = DetachLink.Request()
        req.model1_name = 'combined_cell'
        req.link1_name = robot_link
        req.model2_name = self._chip_name
        req.link2_name = 'base_link_chip'
        future = self._detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Detach failed: {label}')
        self.get_logger().info(f'Detached {self._chip_name} from {robot_link} ({label})')

    def _wait_for_belt_event(self, attr: str, label: str) -> None:
        setattr(self, attr, False)
        self.get_logger().info(f'Waiting for {label} sonar event...')
        while not getattr(self, attr):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'{label} object ready')

    def run_once(self) -> None:
        self._wait_ready()

        self._set_belt(self._belt1_client, 0.0, 'belt1')
        self._set_belt(self._belt2_client, 0.0, 'belt2')
        self._stage('scara', 'home')
        self._stage('bobby', 'home')
        self._gripper('open')

        self._spawn_chip()
        time.sleep(0.5)

        self._set_belt(self._belt1_client, self._belt_run_power, 'belt1')
        self._wait_for_belt_event('_belt1_ready', 'belt1')

        self._stage('scara', 'pre_pick_belt1')
        self._stage('scara', 'pick_belt1')
        self._attach('Link_4', 'SCARA pick')
        self._stage('scara', 'lift_belt1')
        self._stage('scara', 'pre_place_belt2')
        self._stage('scara', 'place_belt2')
        self._detach('Link_4', 'SCARA place on belt2')
        self._stage('scara', 'retreat')

        self._set_belt(self._belt2_client, self._belt_run_power, 'belt2')
        self._wait_for_belt_event('_belt2_ready', 'belt2')

        self._gripper('open')
        self._stage('bobby', 'pre_pick_belt2')
        self._stage('bobby', 'pick_belt2')
        self._gripper('close')
        self._attach('bobby_gripper_base', 'bobby pick')
        self._stage('bobby', 'lift_belt2')
        self._stage('bobby', 'pre_drop_box')
        self._stage('bobby', 'drop_box')
        self._detach('bobby_gripper_base', 'bobby drop in box')
        self._gripper('open')
        self._stage('bobby', 'home')
        self._stage('scara', 'home')
        self.get_logger().info('Dual-cell cycle complete')


def main() -> None:
    rclpy.init()
    node = DualCellCycle()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
