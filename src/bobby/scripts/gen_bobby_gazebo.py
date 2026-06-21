#!/usr/bin/env python3
"""Generate the multi-robot Gazebo variant of bobby.

Takes the plain ``bobby.urdf`` (and the MoveIt SRDF) and produces:

* ``bobby/urdf/bobby_gazebo.urdf`` - all LINK names prefixed with ``bobby_``
  (joint names are left untouched - they don't clash with SCARA's ``Joint_*``),
  plus a ``world`` link + fixed joint that anchors the robot on its pedestal,
  a ``gazebo_ros2_control/GazeboSystem`` ``<ros2_control>`` block and the
  ``libgazebo_ros2_control.so`` plugin running in the ``/bobby`` namespace.
* ``bobby_moveit_config_gazebo/config/bobby_gazebo.srdf`` - the SRDF with link
  references prefixed and the ``virtual_joint`` removed (the URDF now owns the
  ``world`` link, so MoveIt's planning frame is ``world`` directly).

Only LINK names are prefixed; this keeps the planning groups / controllers
(which reference the unchanged joint names) valid without further edits.

Run from anywhere; paths are resolved relative to this file.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))            # .../src
BOBBY_URDF = os.path.join(SRC, "bobby", "urdf", "bobby.urdf")
SRDF_IN = os.path.join(SRC, "bobby_moveit_config", "config", "bobby_v3.srdf")
URDF_OUT = os.path.join(SRC, "bobby", "urdf", "bobby_gazebo.urdf")
SRDF_OUT = os.path.join(SRC, "bobby_moveit_config_gazebo", "config", "bobby_gazebo.srdf")

PREFIX = "bobby_"
# Every link defined in bobby.urdf.
LINKS = [
    "base_link",
    "link_1", "link_2", "link_3", "link_4", "link_5", "link_6",
    "gripper_flange", "gripper_base", "finger_left", "finger_right", "TCP",
]
# Arm + gripper actuated joints that get a ros2_control command interface.
CONTROL_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4",
                  "joint_5", "joint_6", "finger_left_joint"]
# Where bobby is anchored in the shared world (top of its pedestal).
PEDESTAL_XYZ = "0 1.0 0.8"
# Placeholder the launch file replaces with the real controllers.yaml path.
CONTROLLERS_PLACEHOLDER = "__BOBBY_CONTROLLERS_YAML__"


def prefix_urdf_links(text: str) -> str:
    """Prefix link names in a URDF: ``name="L"`` (link defs) and ``link="L"``
    (parent/child refs). The closing quote guards against matching joints whose
    names merely start with a link name (e.g. ``finger_left_joint``)."""
    for link in LINKS:
        text = text.replace(f'name="{link}"', f'name="{PREFIX}{link}"')
        text = text.replace(f'link="{link}"', f'link="{PREFIX}{link}"')
    return text


def build_ros2_control_block() -> str:
    joints = "\n".join(
        f'''    <joint name="{j}">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>''' for j in CONTROL_JOINTS
    )
    return f'''  <ros2_control name="bobby_system" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
{joints}
  </ros2_control>
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="bobby_gazebo_ros2_control">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <ros>
        <namespace>/bobby</namespace>
      </ros>
      <parameters>{CONTROLLERS_PLACEHOLDER}</parameters>
    </plugin>
  </gazebo>'''


def build_world_anchor() -> str:
    return f'''  <link name="world"/>
  <joint name="bobby_world_joint" type="fixed">
    <parent link="world"/>
    <child link="{PREFIX}base_link"/>
    <origin xyz="{PEDESTAL_XYZ}" rpy="0 0 0"/>
  </joint>'''


def gen_urdf() -> None:
    with open(BOBBY_URDF, encoding="utf-8") as f:
        urdf = f.read()
    # Strip XML comments + declaration. gazebo_ros2_control forwards the whole
    # robot_description to each controller as a `--param robot_description:=...`
    # CLI rule; comment text (URLs, '@', ':') breaks rcl's argument parser and
    # aborts gzserver. A clean URDF avoids that.
    urdf = re.sub(r"<\?xml.*?\?>", "", urdf, flags=re.DOTALL)
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.DOTALL)
    urdf = urdf.lstrip()
    urdf = prefix_urdf_links(urdf)
    injection = build_world_anchor() + "\n" + build_ros2_control_block() + "\n</robot>"
    urdf = urdf.replace("</robot>", injection)
    os.makedirs(os.path.dirname(URDF_OUT), exist_ok=True)
    with open(URDF_OUT, "w", encoding="utf-8") as f:
        f.write(urdf)
    print(f"wrote {URDF_OUT}")


def gen_srdf() -> None:
    with open(SRDF_IN, encoding="utf-8") as f:
        srdf = f.read()
    # Prefix link references (link-bearing attributes only; joints untouched).
    for attr in ("base_link", "tip_link", "parent_link", "child_link", "link1", "link2"):
        for link in LINKS:
            srdf = srdf.replace(f'{attr}="{link}"', f'{attr}="{PREFIX}{link}"')
    # Drop the virtual_joint: the URDF now provides the real ``world`` link.
    srdf = re.sub(r"\s*<!--VIRTUAL JOINT:.*?-->", "", srdf, flags=re.DOTALL)
    srdf = re.sub(r'\s*<virtual_joint[^>]*/>', "", srdf)
    os.makedirs(os.path.dirname(SRDF_OUT), exist_ok=True)
    with open(SRDF_OUT, "w", encoding="utf-8") as f:
        f.write(srdf)
    print(f"wrote {SRDF_OUT}")


if __name__ == "__main__":
    gen_urdf()
    gen_srdf()
