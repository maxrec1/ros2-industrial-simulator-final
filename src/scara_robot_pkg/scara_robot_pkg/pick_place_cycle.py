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
from ament_index_python.packages import get_package_share_directory

# Bobby (second robot) joint layout — used for the bobby stage that runs
# sequentially after the SCARA has merged the chip onto the PCB.
BOBBY_ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
BOBBY_GRIPPER_JOINTS = ['finger_left_joint', 'finger_right_joint']
BOBBY_GRIPPER_OPEN = [0.04, -0.04]


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
        self.declare_parameter('pcb_model_name', 'pcb1')
        self.declare_parameter('pcb_link_name', 'base_link_pcb')

        self._action_name = self.get_parameter('controller_action').get_parameter_value().string_value
        self._belt1_name = self.get_parameter('belt1_service').get_parameter_value().string_value
        self._belt2_name = self.get_parameter('belt2_service').get_parameter_value().string_value
        self._belt_stop_power = float(self.get_parameter('belt_stop_power').get_parameter_value().double_value)
        self._belt_run_power = float(self.get_parameter('belt_run_power').get_parameter_value().double_value)
        self._robot_model_name = self.get_parameter('robot_model_name').get_parameter_value().string_value
        self._tool_link_name = self.get_parameter('tool_link_name').get_parameter_value().string_value
        self._chip_model_name = self.get_parameter('chip_model_name').get_parameter_value().string_value
        self._chip_link_name = self.get_parameter('chip_link_name').get_parameter_value().string_value
        self._pcb_model_name = self.get_parameter('pcb_model_name').get_parameter_value().string_value
        self._pcb_link_name = self.get_parameter('pcb_link_name').get_parameter_value().string_value

        self._traj_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._belt1_client = self.create_client(ConveyorBeltControl, self._belt1_name)
        self._belt2_client = self.create_client(ConveyorBeltControl, self._belt2_name)
        self._attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self._detach_client = self.create_client(DetachLink, '/DETACHLINK')

        # Flag set by sonar stopper when belt2 object is in position
        self._belt2_object_ready: bool = False
        self.create_subscription(Bool, 'belt2/object_ready', self._on_belt2_ready, 10)

        # ----- Bobby stage (runs sequentially in this same node) -----
        self.declare_parameter('bobby_arm_action', '/bobby_arm_controller/follow_joint_trajectory')
        self.declare_parameter('bobby_gripper_action', '/bobby_gripper_controller/follow_joint_trajectory')
        _default_wp = os.path.join(
            get_package_share_directory('scara_robot_pkg'), 'config', 'bobby_waypoints.yaml')
        self.declare_parameter('bobby_waypoints_path', _default_wp)
        self.declare_parameter('bobby_robot_link', 'bobby_link_6')
        self.declare_parameter('bobby_move_time', 4.0)
        self.declare_parameter('bobby_gripper_time', 1.0)

        self._bobby_arm = ActionClient(
            self, FollowJointTrajectory, self.get_parameter('bobby_arm_action').value)
        self._bobby_grip = ActionClient(
            self, FollowJointTrajectory, self.get_parameter('bobby_gripper_action').value)
        self._bobby_robot_link = self.get_parameter('bobby_robot_link').value
        self._bobby_move_time = float(self.get_parameter('bobby_move_time').value)
        self._bobby_gripper_time = float(self.get_parameter('bobby_gripper_time').value)
        with open(self.get_parameter('bobby_waypoints_path').value) as f:
            self._bobby_wp = yaml.safe_load(f)['positions']

        # Flag set by sonar stopper when the PCB reaches the belt1 sonar
        self._belt1_object_ready: bool = False
        self.create_subscription(Bool, 'belt1/object_ready', self._on_belt1_ready, 10)

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
        # The conveyor plugin advertises /CONVEYORPOWER before its service
        # callback is fully ready, so the very first call right after bringup can
        # time out even though wait_for_service() already returned. Retry a few
        # times (re-confirming the service each round) instead of crashing.
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        last_err = 'unknown error'
        for attempt in range(1, 6):
            if not client.wait_for_service(timeout_sec=5.0):
                last_err = 'service not available'
            else:
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    if future.result().success:
                        self.get_logger().info(f'{label} belt power set to {power}')
                        return
                    last_err = f'rejected power command {power}'
                else:
                    last_err = 'call did not return in time'
            self.get_logger().warn(
                f'{label} belt power attempt {attempt} failed ({last_err}); retrying...')
        raise RuntimeError(f'Failed to set {label} belt power after retries: {last_err}')

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

    def _attach_chip_to_pcb(self) -> None:
        req = AttachLink.Request()
        req.model1_name = self._chip_model_name
        req.link1_name = self._chip_link_name
        req.model2_name = self._pcb_model_name
        req.link2_name = self._pcb_link_name
        future = self._attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Attach chip-to-pcb service did not return a response')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'Attach chip-to-pcb failed: {result.message}')
        self.get_logger().info(
            f'Chip attached to PCB: {self._chip_model_name}/{self._chip_link_name} -> '
            f'{self._pcb_model_name}/{self._pcb_link_name}'
        )

    # ---------------------- Bobby stage (sequential) ----------------------- #
    def _on_belt1_ready(self, msg: Bool) -> None:
        if msg.data:
            self._belt1_object_ready = True

    def _wait_for_belt1_object(self) -> None:
        """Spin until sonar_belt_stopper publishes object_ready on belt1."""
        self.get_logger().info('Waiting for PCB on belt1 sonar...')
        self._belt1_object_ready = False
        while not self._belt1_object_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('PCB ready on belt1 — starting bobby.')

    def _send_bobby_traj(self, client, joints, positions, secs, label) -> None:
        if not client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError(f'Bobby action server unavailable: {label}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1.0) * 1e9))
        goal.trajectory.points = [pt]
        gh_fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut)
        gh = gh_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f'Bobby goal rejected: {label}')
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)

    def _bobby_arm_to(self, name: str) -> None:
        wp = self._bobby_wp[name]
        self.get_logger().info(f'bobby arm -> {name}')
        self._send_bobby_traj(self._bobby_arm, BOBBY_ARM_JOINTS,
                              [wp[j] for j in BOBBY_ARM_JOINTS], self._bobby_move_time, name)

    def _bobby_arm_home(self) -> None:
        self.get_logger().info('bobby arm -> home')
        self._send_bobby_traj(self._bobby_arm, BOBBY_ARM_JOINTS, [0.0] * 6,
                              self._bobby_move_time, 'home')

    def _bobby_gripper(self, positions, label: str) -> None:
        self.get_logger().info(f'bobby gripper -> {label}')
        self._send_bobby_traj(self._bobby_grip, BOBBY_GRIPPER_JOINTS, positions,
                              self._bobby_gripper_time, label)

    def _attach_pcb_to_bobby(self) -> None:
        req = AttachLink.Request()
        req.model1_name = self._robot_model_name
        req.link1_name = self._bobby_robot_link
        req.model2_name = self._pcb_model_name
        req.link2_name = self._pcb_link_name
        future = self._attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError('Attach pcb-to-bobby failed')
        self.get_logger().info('PCB (with chip) welded to bobby gripper')

    def _detach_pcb_from_bobby(self) -> None:
        req = DetachLink.Request()
        req.model1_name = self._robot_model_name
        req.link1_name = self._bobby_robot_link
        req.model2_name = self._pcb_model_name
        req.link2_name = self._pcb_link_name
        future = self._detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError('Detach pcb-from-bobby failed')
        self.get_logger().info('PCB released from bobby gripper')

    def _run_bobby_sequence(self) -> None:
        self.get_logger().info('=== Bobby sequence start ===')
        # start clean: home + gripper open
        self._bobby_arm_home()
        self._bobby_gripper(BOBBY_GRIPPER_OPEN, 'open')
        # home -> approach -> pre_grab -> grabbed
        for name in ['approach', 'pre_grab', 'grabbed']:
            self._bobby_arm_to(name)
        # close on the PCB (finger values stored in the 'grabbed' waypoint)
        grab = self._bobby_wp['grabbed']
        self._bobby_gripper([grab['finger_left_joint'], grab['finger_right_joint']], 'close')
        # weld PCB (with chip) to bobby gripper
        self._attach_pcb_to_bobby()
        # transport: grabbed -> ready_for_drop_off -> ablage
        # (ablage is the final drop pose before releasing)
        for name in ['ready_for_drop_off', 'ablage']:
            self._bobby_arm_to(name)
        # release + open + back home
        self._detach_pcb_from_bobby()
        self._bobby_gripper(BOBBY_GRIPPER_OPEN, 'open')
        self._bobby_arm_home()
        self.get_logger().info('=== Bobby sequence complete ===')

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
            self._attach_chip_to_pcb()            # weld chip onto pcb (rides along + no spin)
            self._send_joint_stage('retreat')
            time.sleep(0.2)

            self.get_logger().info(f'=== Cycle {cycle} complete ===')

            # Chip is now merged onto the PCB. Hand off to bobby: start belt1 so
            # the PCB (with chip) rides to the belt1 sonar, which publishes
            # 'belt1/object_ready' and triggers the bobby_pick_place node. One
            # PCB/chip pair spawns per launch, so the SCARA stage is done here.
            self.get_logger().info('Chip merged on PCB -> starting belt1')
            self._set_belt_power(self._belt1_client, self._belt_run_power, 'belt1')

            # Wait until the PCB (with chip) reaches the belt1 sonar, stop belt1,
            # then run the bobby pick-place sequence in this same node.
            self._wait_for_belt1_object()
            self._set_belt_power(self._belt1_client, self._belt_stop_power, 'belt1')
            self._run_bobby_sequence()

            self.get_logger().info('=== Full workflow complete ===')
            break


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

