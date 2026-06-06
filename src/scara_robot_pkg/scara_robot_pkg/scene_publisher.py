#!/usr/bin/env python3
"""
Publishes static station collision geometry (pedestal + conveyors) and
continuously mirrors dynamic Gazebo object poses (pcb1, chip1) into the
MoveIt planning scene so RViz shows the full workcell and the planner
can collision-check against it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates

import math


class ScenePublisher(Node):

    def __init__(self):
        super().__init__('scene_publisher')

        # ---------- parameters ----------
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('dynamic_model_names', ['pcb1', 'chip1'])
        self.declare_parameter('dynamic_update_hz', 5.0)

        self.planning_frame = self.get_parameter('planning_frame').value
        self.dynamic_names = self.get_parameter('dynamic_model_names').value
        self.update_hz = self.get_parameter('dynamic_update_hz').value

        # ---------- publisher ----------
        # Latching QoS so late-joining move_group picks up the scene
        latching_qos = QoSProfile(depth=1,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', latching_qos)

        # ---------- publish static geometry once ----------
        self._publish_static_scene()

        # ---------- dynamic Gazebo tracking ----------
        self._model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._model_states_cb, 10)

        # Rate-limit dynamic updates
        self._last_update_time = self.get_clock().now()
        self._update_period_ns = int(1e9 / max(self.update_hz, 0.1))

        # Track which dynamic objects we have already ADDed
        self._dynamic_added = set()

        self.get_logger().info(
            f'Scene publisher ready – frame={self.planning_frame}, '
            f'tracking {self.dynamic_names} @ {self.update_hz} Hz')

    # ------------------------------------------------------------------ #
    #  Static station geometry                                            #
    # ------------------------------------------------------------------ #
    def _publish_static_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        scene.world.collision_objects.extend([
            self._make_pedestal(),
            self._make_conveyor1(),
            self._make_conveyor2(),
        ])

        self._scene_pub.publish(scene)
        self.get_logger().info('Published static collision objects '
                               '(pedestal + 2 conveyors)')

    # -- pedestal: cylinder + mounting block ----------------------------
    def _make_pedestal(self):
        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.id = 'pedestal'
        co.operation = CollisionObject.ADD

        # Cylinder shaft  r=0.12  h=0.7  centred at z=0.35
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.7, 0.12]          # [height, radius]
        cyl_pose = Pose()
        cyl_pose.position = Point(x=-1.0, y=0.0, z=0.35)
        cyl_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Mounting block  0.25 x 0.25 x 0.1  centred at z=0.75
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.25, 0.25, 0.1]    # [x, y, z]
        box_pose = Pose()
        box_pose.position = Point(x=-1.0, y=0.0, z=0.75)
        box_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives = [cyl, box]
        co.primitive_poses = [cyl_pose, box_pose]
        return co

    # -- conveyor belt 1: at origin, runs along Y -----------------------
    def _make_conveyor1(self):
        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.id = 'conveyor_belt_1'
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.425, 1.2, 0.74]   # belt footprint x surface height
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.37)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives = [box]
        co.primitive_poses = [pose]
        return co

    # -- conveyor belt 2: at (-1, -1, 0), rotated 90° about Z ----------
    def _make_conveyor2(self):
        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.id = 'conveyor_belt_2'
        co.operation = CollisionObject.ADD

        # Belt 2 is rotated 90° in world, so swap X/Y dims
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.2, 0.425, 0.74]
        pose = Pose()
        pose.position = Point(x=-1.0, y=-1.0, z=0.37)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives = [box]
        co.primitive_poses = [pose]
        return co

    # ------------------------------------------------------------------ #
    #  Dynamic Gazebo object tracking                                     #
    # ------------------------------------------------------------------ #
    def _model_states_cb(self, msg: ModelStates):
        now = self.get_clock().now()
        if (now - self._last_update_time).nanoseconds < self._update_period_ns:
            return
        self._last_update_time = now

        scene = PlanningScene()
        scene.is_diff = True
        found_any = False

        for name in self.dynamic_names:
            if name not in msg.name:
                continue

            idx = msg.name.index(name)
            gz_pose = msg.pose[idx]

            co = CollisionObject()
            co.header.frame_id = self.planning_frame
            co.id = name

            if name not in self._dynamic_added:
                co.operation = CollisionObject.ADD
                self._dynamic_added.add(name)
            else:
                co.operation = CollisionObject.MOVE

            # Small box approximation for PCB / chip
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.05, 0.05, 0.01]

            pose = Pose()
            pose.position = gz_pose.position
            pose.orientation = gz_pose.orientation

            co.primitives = [box]
            co.primitive_poses = [pose]
            scene.world.collision_objects.append(co)
            found_any = True

        if found_any:
            self._scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
