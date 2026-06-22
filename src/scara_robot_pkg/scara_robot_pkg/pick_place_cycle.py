#!/usr/bin/env python3

import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from conveyorbelt_msgs.srv import ConveyorBeltControl
from std_msgs.msg import Bool
from linkattacher_msgs.srv import AttachLink, DetachLink


class PickPlaceCycle(Node):
    def __init__(self) -> None:
        super().__init__('pick_place_cycle')

        self.declare_parameter('config_path', '')
        self.declare_parameter('controller_action', '/arm_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('belt1_service', '/CONVEYORPOWER')
        self.declare_parameter('belt2_service', '/belt2/CONVEYORPOWER')
        self.declare_parameter('belt_stop_power', 0.0)
        self.declare_parameter('belt_run_power', 20.0)
        self.declare_parameter('robot_model_name', 'combined_cell')
        self.declare_parameter('tool_link_name', 'Link_4')
        self.declare_parameter('chip_model_name', 'chip1')
        self.declare_parameter('chip_link_name', 'base_link_chip')

        self._action_name = self.get_parameter('controller_action').get_parameter_value().string_value
        self._belt1_name = self.get_parameter('belt1_service').get_parameter_value().string_value
        self._belt2_name = self.get_parameter('belt2_service').get_parameter_value().string_value
        self._belt_stop_power = float(self.get_parameter('belt_stop_power').get_parameter_value().double_value)
        self._belt_run_power = float(self.get_parameter('belt_run_power').get_parameter_value().double_value)
        self._robot_model_name = self.get_parameter('robot_model_name').get_parameter_value().string_value
        self._tool_link_name = self.get_parameter('tool_link_name').get_parameter_value().string_value
        self._chip_model_name = self.get_parameter('chip_model_name').get_parameter_value().string_value
        self._chip_link_name = self.get_parameter('chip_link_name').get_parameter_value().string_value

        self._traj_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._belt1_client = self.create_client(ConveyorBeltControl, self._belt1_name)
        self._belt2_client = self.create_client(ConveyorBeltControl, self._belt2_name)
        self._attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self._detach_client = self.create_client(DetachLink, '/DETACHLINK')

        # Flag set by sonar stopper when belt2 object is in position
        self._belt2_object_ready: bool = False
        self.create_subscription(Bool, 'belt2/object_ready', self._on_belt2_ready, 10)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError('Parameter config_path is required')

        self._config = self._load_config(config_path)

    def _on_belt2_ready(self, msg: Bool) -> None:
        if msg.data:
            self._belt2_object_ready = True

    def _load_config(self, config_path: str):
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        required = ['joint_names', 'default_timing_sec', 'stages']
        for key in required:
            if key not in data:
                raise RuntimeError(f'Missing key in config: {key}')
        return data

    def _wait_for_services(self) -> None:
        for client, name in [
            (self._belt1_client, self._belt1_name),
            (self._belt2_client, self._belt2_name),
            (self._attach_client, '/ATTACHLINK'),
            (self._detach_client, '/DETACHLINK'),
        ]:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'Service not available: {name}')

    def _set_belt_power(self, client, power: float, label: str) -> None:
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Failed to call {label} belt power service')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'{label} belt rejected power command {power}')
        self.get_logger().info(f'{label} belt power set to {power}')

    def _wait_for_action(self) -> None:
        if not self._traj_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError(f'Action server not available: {self._action_name}')

    def _send_joint_stage(self, stage_name: str) -> None:
        stage = self._config['stages'].get(stage_name)
        if stage is None:
            raise RuntimeError(f'Stage not found in config: {stage_name}')

        positions = stage.get('positions', [])
        joint_names = self._config['joint_names']
        if len(positions) != len(joint_names):
            raise RuntimeError(f'Stage {stage_name} has wrong joint count')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = float(self._config['default_timing_sec'])
        point.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1.0) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info(f'Executing stage: {stage_name} -> {positions}')
        send_future = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError(f'Failed to send goal for stage {stage_name}')

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f'Goal rejected for stage {stage_name}')

        result_future = goal_handle.get_result_async()
        timeout = max(10.0, float(self._config['default_timing_sec']) + 5.0)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError(f'No result for stage {stage_name}')

        result = result_future.result().result
        if result.error_code != 0:
            raise RuntimeError(f'Stage {stage_name} failed with error code {result.error_code}')

    def _attach_chip(self) -> None:
        req = AttachLink.Request()
        req.model1_name = self._robot_model_name
        req.link1_name = self._tool_link_name
        req.model2_name = self._chip_model_name
        req.link2_name = self._chip_link_name
        future = self._attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Attach service did not return a response')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'Attach failed: {result.message}')
        self.get_logger().info(
            f'Chip attached: {self._chip_model_name}/{self._chip_link_name} -> '
            f'{self._robot_model_name}/{self._tool_link_name}'
        )

    def _detach_chip(self) -> None:
        req = DetachLink.Request()
        req.model1_name = self._robot_model_name
        req.link1_name = self._tool_link_name
        req.model2_name = self._chip_model_name
        req.link2_name = self._chip_link_name
        future = self._detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Detach service did not return a response')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'Detach failed: {result.message}')
        self.get_logger().info(
            f'Chip detached: {self._chip_model_name}/{self._chip_link_name} from '
            f'{self._robot_model_name}/{self._tool_link_name}'
        )

    def _wait_for_belt2_object(self) -> None:
        """Spin until sonar_belt_stopper publishes object_ready on belt2."""
        self.get_logger().info('Waiting for object on belt2 sonar...')
        self._belt2_object_ready = False
        while not self._belt2_object_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Object ready on belt2 — starting pick-place.')

    def run_cycle(self) -> None:
        self._wait_for_services()
        self._wait_for_action()

        cycle = 0
        while True:
            cycle += 1
            self.get_logger().info(f'=== Cycle {cycle}: starting belt2 (belt1 off) ===')
            # belt1 intentionally left off
            self._set_belt_power(self._belt2_client, self._belt_run_power, 'belt2')

            # Block until sonar detects object at belt2 exit
            self._wait_for_belt2_object()

            # Execute pick-place arm sequence
            self._send_joint_stage('pre_pick')
            time.sleep(0.2)
            self._send_joint_stage('pick')
            time.sleep(0.2)
            self._attach_chip()                   # grip chip after pick
            self._send_joint_stage('lift')
            time.sleep(0.2)
            self._send_joint_stage('pre_place')
            time.sleep(0.2)
            self._send_joint_stage('place')
            time.sleep(0.2)
            self._detach_chip()                   # release chip after place
            self._send_joint_stage('retreat')
            time.sleep(0.2)

            self.get_logger().info(f'=== Cycle {cycle} complete ===')


def main() -> None:
    rclpy.init()
    node = PickPlaceCycle()
    try:
        node.run_cycle()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

