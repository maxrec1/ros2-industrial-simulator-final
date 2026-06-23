#!/usr/bin/env python3
"""
Bobby pick-and-place cycle (executable).

Trigger: PCB detected at the belt-1 sonar (Bool on 'belt1/object_ready',
published by sonar_belt_stopper). On each trigger bobby runs:

    home -> approach -> pre_grab -> grabbed
    -> close gripper -> ATTACHLINK (pcb1 -> bobby_link_6)
    -> ablage -> ready_for_drop_off
    -> DETACHLINK -> open gripper
    -> home

Arm joint targets are loaded from bobby_waypoints.yaml so they stay in sync
with the poses saved from RViz. Gripper open/close uses the finger values
stored in the 'grabbed' waypoint (close) and a fixed open value.
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool
from linkattacher_msgs.srv import AttachLink, DetachLink
from ament_index_python.packages import get_package_share_directory

ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GRIPPER_JOINTS = ['finger_left_joint', 'finger_right_joint']
GRIPPER_OPEN = [0.04, -0.04]

# Order of arm waypoints (names must exist in bobby_waypoints.yaml).
ARM_SEQUENCE = ['approach', 'pre_grab', 'grabbed', 'ablage', 'ready_for_drop_off']


class BobbyPickPlace(Node):

    def __init__(self) -> None:
        super().__init__('bobby_pick_place')

        share = get_package_share_directory('scara_robot_pkg')
        default_wp = os.path.join(share, 'config', 'bobby_waypoints.yaml')
        self.declare_parameter('waypoints_path', default_wp)
        self.declare_parameter('arm_action',
                               '/bobby_arm_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_action',
                               '/bobby_gripper_controller/follow_joint_trajectory')
        self.declare_parameter('trigger_topic', 'belt1/object_ready')
        self.declare_parameter('move_time', 4.0)        # s per arm move
        self.declare_parameter('gripper_time', 1.0)     # s per gripper move
        # Link-attacher targets
        self.declare_parameter('robot_model', 'combined_cell')
        self.declare_parameter('robot_link', 'bobby_link_6')
        self.declare_parameter('object_model', 'pcb1')
        self.declare_parameter('object_link', 'base_link_pcb')
        self.declare_parameter('loop', True)            # run on every trigger

        self.move_time = self.get_parameter('move_time').value
        self.gripper_time = self.get_parameter('gripper_time').value

        wp_path = self.get_parameter('waypoints_path').value
        with open(wp_path) as f:
            self.waypoints = yaml.safe_load(f)['positions']
        self.get_logger().info(f'Loaded waypoints: {list(self.waypoints)}')

        self._arm = ActionClient(self, FollowJointTrajectory,
                                 self.get_parameter('arm_action').value)
        self._grip = ActionClient(self, FollowJointTrajectory,
                                  self.get_parameter('gripper_action').value)
        self._attach = self.create_client(AttachLink, '/ATTACHLINK')
        self._detach = self.create_client(DetachLink, '/DETACHLINK')

        self._busy = False
        self.create_subscription(
            Bool, self.get_parameter('trigger_topic').value,
            self._on_trigger, 10)

        self.get_logger().info('Bobby pick-place ready — waiting for '
                               f"'{self.get_parameter('trigger_topic').value}'.")

    # ------------------------------------------------------------------ #
    def _on_trigger(self, msg: Bool) -> None:
        if not msg.data or self._busy:
            return
        self._busy = True
        try:
            self.run_cycle()
        finally:
            self._busy = not self.get_parameter('loop').value

    # ---- low-level moves --------------------------------------------- #
    def _send_traj(self, client, joints, positions, secs) -> bool:
        if not client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(f'Action server unavailable: {client._action_name}')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(secs),
                                      nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        gh_fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut)
        gh = gh_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return True

    def _move_arm(self, name: str) -> bool:
        wp = self.waypoints[name]
        self.get_logger().info(f'-> arm: {name}')
        return self._send_traj(self._arm, ARM_JOINTS,
                               [wp[j] for j in ARM_JOINTS], self.move_time)

    def _move_arm_home(self) -> bool:
        self.get_logger().info('-> arm: home (zeros)')
        return self._send_traj(self._arm, ARM_JOINTS, [0.0] * 6, self.move_time)

    def _gripper(self, positions, label: str) -> bool:
        self.get_logger().info(f'-> gripper: {label}')
        return self._send_traj(self._grip, GRIPPER_JOINTS, positions,
                               self.gripper_time)

    def _call_link(self, client, srv_type, action: str) -> bool:
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{action} service unavailable')
            return False
        req = srv_type.Request()
        req.model1_name = self.get_parameter('robot_model').value
        req.link1_name = self.get_parameter('robot_link').value
        req.model2_name = self.get_parameter('object_model').value
        req.link2_name = self.get_parameter('object_link').value
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info(f'{action}: {"OK" if ok else "FAILED"}')
        return ok

    # ---- full cycle --------------------------------------------------- #
    def run_cycle(self) -> None:
        self.get_logger().info('=== PCB detected — starting pick-place cycle ===')

        # Start clean: home + gripper open
        self._move_arm_home()
        self._gripper(GRIPPER_OPEN, 'open')

        # home -> approach -> pre_grab -> grabbed
        for name in ['approach', 'pre_grab', 'grabbed']:
            self._move_arm(name)

        # close on the PCB (finger values stored in the 'grabbed' waypoint)
        grab = self.waypoints['grabbed']
        self._gripper([grab['finger_left_joint'], grab['finger_right_joint']],
                      'close')

        # weld PCB to gripper
        self._call_link(self._attach, AttachLink, 'ATTACHLINK')

        # transport: grabbed -> ablage -> ready_for_drop_off
        for name in ['ablage', 'ready_for_drop_off']:
            self._move_arm(name)

        # release
        self._call_link(self._detach, DetachLink, 'DETACHLINK')
        self._gripper(GRIPPER_OPEN, 'open')

        # back home
        self._move_arm_home()
        self.get_logger().info('=== Cycle complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = BobbyPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
