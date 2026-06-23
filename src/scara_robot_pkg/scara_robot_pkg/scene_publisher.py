#!/usr/bin/env python3
"""
Publishes static station collision geometry (pedestal + conveyors) and
continuously mirrors dynamic Gazebo object poses (pcb1, chip1) into the
MoveIt planning scene so RViz shows the full workcell and the planner
can collision-check against it.
"""

import math
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from moveit_msgs.msg import (PlanningScene, CollisionObject,
                             AllowedCollisionMatrix, AllowedCollisionEntry,
                             PlanningSceneComponents)
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates

try:
    from ament_index_python.packages import get_package_share_directory
    _HAVE_AMENT_INDEX = True
except ImportError:
    _HAVE_AMENT_INDEX = False


def _load_binary_stl_as_mesh(stl_path: str, scale: float,
                              origin_xyz: tuple) -> Mesh:
    """Parse a binary STL file and return shape_msgs/Mesh with scale and
    origin-offset pre-applied so the vertices are in the URDF link frame."""
    ox, oy, oz = origin_xyz
    mesh = Mesh()
    with open(stl_path, 'rb') as f:
        f.read(80)                                   # 80-byte header
        num_tri = struct.unpack('<I', f.read(4))[0]  # triangle count
        for i in range(num_tri):
            f.read(12)                               # face normal (skip)
            base = len(mesh.vertices)
            for _ in range(3):
                x, y, z = struct.unpack('<fff', f.read(12))
                p = Point()
                p.x = x * scale + ox
                p.y = y * scale + oy
                p.z = z * scale + oz
                mesh.vertices.append(p)
            f.read(2)                                # attribute byte count
            t = MeshTriangle()
            t.vertex_indices = [base, base + 1, base + 2]
            mesh.triangles.append(t)
    return mesh


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

        # ---------- pre-load PCB mesh (once at startup) ----------
        self._pcb_mesh: Mesh | None = self._load_pcb_mesh()
        if self._pcb_mesh:
            self.get_logger().info(
                f'PCB mesh loaded: {len(self._pcb_mesh.triangles)} triangles')
        else:
            self.get_logger().warn(
                'PCB mesh unavailable — will use box approximation')

        # ---------- dynamic Gazebo tracking ----------
        # Gazebo publishes with BEST_EFFORT — subscriber must match
        _gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._model_states_sub = self.create_subscription(
            ModelStates, '/model_states',
            self._model_states_cb, _gz_qos)

        # Rate-limit dynamic updates
        self._last_update_time = self.get_clock().now()
        self._update_period_ns = int(1e9 / max(self.update_hz, 0.1))

        # Track which dynamic objects we have already ADDed
        self._dynamic_added = set()

        # ---------- grasp-collision allowance (ACM) ----------
        # The gripper jaws MUST contact the part to grasp it, but MoveIt rejects
        # any state where jaw links touch a collision object. Allow contact
        # between these gripper links and the dynamic parts so grasp poses plan.
        self.declare_parameter(
            'gripper_touch_links',
            ['bobby_finger_left', 'bobby_finger_right', 'bobby_gripper_base'])
        self.gripper_touch_links = self.get_parameter('gripper_touch_links').value
        self._get_scene_cli = self.create_client(GetPlanningScene,
                                                 '/get_planning_scene')
        self._acm_applied = False

        self.get_logger().info(
            f'Scene publisher ready – frame={self.planning_frame}, '
            f'tracking {self.dynamic_names} @ {self.update_hz} Hz')

    # ------------------------------------------------------------------ #
    #  Allowed Collision Matrix: let gripper jaws touch the parts         #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _acm_allow(acm: AllowedCollisionMatrix, a: str, b: str) -> None:
        """Mark collision a<->b as allowed, growing the matrix if needed."""
        for nm in (a, b):
            if nm not in acm.entry_names:
                acm.entry_names.append(nm)
                for row in acm.entry_values:        # extend existing rows
                    row.enabled.append(False)
                new_row = AllowedCollisionEntry()
                new_row.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(new_row)
        ia = acm.entry_names.index(a)
        ib = acm.entry_names.index(b)
        acm.entry_values[ia].enabled[ib] = True
        acm.entry_values[ib].enabled[ia] = True

    def _ensure_grasp_acm(self) -> None:
        """Fetch current ACM, allow gripper<->part contact, republish."""
        if self._acm_applied or not self._get_scene_cli.service_is_ready():
            return
        req = GetPlanningScene.Request()
        req.components.components = \
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        fut = self._get_scene_cli.call_async(req)
        fut.add_done_callback(self._on_acm_response)

    def _on_acm_response(self, fut) -> None:
        try:
            acm = fut.result().scene.allowed_collision_matrix
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f'ACM fetch failed: {exc}')
            return
        for link in self.gripper_touch_links:
            for obj in self.dynamic_names:
                self._acm_allow(acm, link, obj)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        self._scene_pub.publish(scene)
        self._acm_applied = True
        self.get_logger().info(
            f'ACM updated: {self.gripper_touch_links} may contact '
            f'{self.dynamic_names}')

    # ------------------------------------------------------------------ #
    #  Static station geometry                                            #
    # ------------------------------------------------------------------ #
    def _publish_static_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        scene.world.collision_objects.extend([
            self._make_pedestal('scara_pedestal_collision', -1.0, 0.0),
            self._make_pedestal('bobby_pedestal_collision', 0.0, 1.0),
            self._make_pedestal('drop_box_pedestal_collision', 0.0, 1.45),
            self._make_conveyor1(),
            self._make_conveyor2(),
            self._make_drop_box(),
        ])

        self._scene_pub.publish(scene)
        self.get_logger().info('Published static collision objects '
                               '(pedestal + 2 conveyors)')

    # -- pedestal: cylinder + mounting block ----------------------------
    def _make_pedestal(self, object_id: str, x: float, y: float):
        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.id = object_id
        co.operation = CollisionObject.ADD

        # Cylinder shaft  r=0.12  h=0.7  centred at z=0.35
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.7, 0.12]          # [height, radius]
        cyl_pose = Pose()
        cyl_pose.position = Point(x=x, y=y, z=0.35)
        cyl_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Mounting block  0.25 x 0.25 x 0.1  centred at z=0.75
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.25, 0.25, 0.1]    # [x, y, z]
        box_pose = Pose()
        box_pose.position = Point(x=x, y=y, z=0.75)
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

    # -- PCB mesh loader -----------------------------------------------
    def _load_pcb_mesh(self) -> 'Mesh | None':
        if not _HAVE_AMENT_INDEX:
            return None
        try:
            share = get_package_share_directory('conveyorbelt_gazebo')
            stl_path = os.path.join(
                share, 'meshes', 'pcb', 'base_link_PCB.STL')
            # scale=5.0, origin from URDF: xyz="-3.0173 -4.3763 -6.4655"
            return _load_binary_stl_as_mesh(
                stl_path, 5.0, (-3.0173, -4.3763, -6.4655))
        except Exception as exc:
            self.get_logger().warn(f'PCB mesh load error: {exc}')
            return None

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

    def _make_drop_box(self):
        co = CollisionObject()
        co.header.frame_id = self.planning_frame
        co.id = 'drop_box_collision'
        co.operation = CollisionObject.ADD

        parts = []
        poses = []
        # Walls are twice the PCB height (0.0188 m); box base top at z=0.82.
        for dims, xyz in [
            ([0.36, 0.36, 0.02], [0.0, 1.45, 0.81]),
            ([0.02, 0.36, 0.0188], [0.18, 1.45, 0.8294]),
            ([0.02, 0.36, 0.0188], [-0.18, 1.45, 0.8294]),
            ([0.36, 0.02, 0.0188], [0.0, 1.63, 0.8294]),
            ([0.36, 0.02, 0.0188], [0.0, 1.27, 0.8294]),
        ]:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dims
            pose = Pose()
            pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            parts.append(box)
            poses.append(pose)

        co.primitives = parts
        co.primitive_poses = poses
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
            co.operation = CollisionObject.ADD  # ADD is idempotent — updates pose if already present

            pose = Pose()
            pose.position = gz_pose.position
            pose.orientation = gz_pose.orientation

            if name == 'pcb1' and self._pcb_mesh is not None:
                # Use actual STL mesh (scale+offset pre-applied, vertices in link frame)
                co.meshes = [self._pcb_mesh]
                co.mesh_poses = [pose]
            else:
                # Box approximation for chip1 (and fallback for pcb1)
                _DIMS = {
                    'pcb1':  [0.100, 0.060, 0.005],
                    'chip1': [0.025, 0.025, 0.0045],
                }
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = _DIMS.get(name, [0.05, 0.05, 0.01])
                co.primitives = [box]
                co.primitive_poses = [pose]
            scene.world.collision_objects.append(co)
            found_any = True

        if found_any:
            self._scene_pub.publish(scene)
            # Once the parts exist in the scene, allow the jaws to touch them.
            self._ensure_grasp_acm()


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
