#!/usr/bin/env python3
"""Generate the single-model combined Gazebo cell: SCARA + bobby in ONE robot
description driven by ONE global gazebo_ros2_control plugin / controller_manager.

Why one model instead of two namespaced ones:
    gazebo_ros2_control 0.4.10 writes a plugin's <ros><namespace> into the
    PROCESS-GLOBAL rcl arguments (rcl_context->global_arguments). With two
    plugins in one gzserver the second Load() clobbers the first, and every
    controller node created afterwards inherits the last namespace -> bobby's
    controllers got dragged into SCARA's namespace and starved of their joint
    params. A single plugin (one controller_manager, global namespace, uniquely
    named controllers) sidesteps the bug entirely.

Produces:
  * scara_robot_pkg/urdf/combined_gazebo.urdf
      - SCARA links/joints (unchanged: base_link, Link_1..4, Joint_1..4)
      - bobby links prefixed bobby_ (joints joint_1..6 / finger_* are already
        distinct from SCARA's Joint_*)
      - one shared `world` link with fixed joints anchoring each robot on its
        pedestal (SCARA at -1 0 0.8, bobby at 0 1.5 0.8)
      - SCARA + bobby <ros2_control> blocks (both gazebo_ros2_control/GazeboSystem)
      - ONE libgazebo_ros2_control.so plugin in the GLOBAL namespace

The launch fills the __COMBINED_CONTROLLERS_YAML__ placeholder and rewrites the
package:// mesh URIs (both scara_robot_pkg/ and bobby/) to file:// paths.

Re-run after editing bobby.urdf or scara_assembly.urdf:
    python3 src/scara_robot_pkg/scripts/gen_combined_gazebo.py
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))            # .../src
BOBBY_URDF = os.path.join(SRC, "bobby", "urdf", "bobby.urdf")
SCARA_URDF = os.path.join(SRC, "scara_robot_pkg", "urdf", "scara_assembly.urdf")
URDF_OUT = os.path.join(SRC, "scara_robot_pkg", "urdf", "combined_gazebo.urdf")

BOBBY_PREFIX = "bobby_"
BOBBY_LINKS = [
    "base_link",
    "link_1", "link_2", "link_3", "link_4", "link_5", "link_6",
    "gripper_flange", "gripper_base", "finger_left", "finger_right", "TCP",
]
BOBBY_CONTROL_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4",
                        "joint_5", "joint_6",
                        "finger_left_joint", "finger_right_joint"]
BOBBY_INITIAL_POSITIONS = {
    "joint_1": "0.0",
    "joint_2": "0.0",
    "joint_3": "0.0",
    "joint_4": "0.0",
    "joint_5": "0.0",
    "joint_6": "0.0",
    "finger_left_joint": "0.02",
    "finger_right_joint": "-0.02",
}

SCARA_PEDESTAL_XYZ = "-1 0 0.8"
BOBBY_PEDESTAL_XYZ = "0 1.0 0.8"
CONTROLLERS_PLACEHOLDER = "__COMBINED_CONTROLLERS_YAML__"


def strip_and_inner(text: str) -> str:
    """Drop the XML declaration + comments, then return the contents inside the
    top-level <robot ...> ... </robot> wrapper."""
    text = re.sub(r"<\?xml.*?\?>", "", text, flags=re.DOTALL)
    text = re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL)
    text = re.sub(r"^\s*<robot\b[^>]*>", "", text, count=1, flags=re.DOTALL)
    text = re.sub(r"</robot>\s*$", "", text, flags=re.DOTALL)
    return text.strip()


def prefix_bobby_links(text: str) -> str:
    for link in BOBBY_LINKS:
        text = text.replace(f'name="{link}"', f'name="{BOBBY_PREFIX}{link}"')
        text = text.replace(f'link="{link}"', f'link="{BOBBY_PREFIX}{link}"')
    return text


def scara_control_joints() -> str:
    # Mirrors scara_ros2_control_gazebo.xacro (limits + initial positions).
    limits = {
        "Joint_1": ("-2.5299", "2.5299", "0.6849"),
        "Joint_2": ("-2.5299", "2.5299", "-2.5299"),
        "Joint_3": ("-0.45", "0.0", "0.0"),
        "Joint_4": ("-6.98", "6.98", "0.0"),
    }
    return "\n".join(
        f'''    <joint name="{j}">
      <command_interface name="position">
        <param name="min">{lo}</param>
        <param name="max">{hi}</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">{init}</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>''' for j, (lo, hi, init) in limits.items()
    )


def bobby_control_joints() -> str:
    return "\n".join(
        f'''    <joint name="{j}">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">{BOBBY_INITIAL_POSITIONS[j]}</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>''' for j in BOBBY_CONTROL_JOINTS
    )


def combined_ros2_control() -> str:
    # ONE <ros2_control> system / ONE GazeboSystem hardware for BOTH robots.
    # Two separate GazeboSystem blocks don't work on gazebo_ros2_control 0.4.x:
    # each instance re-processes the URDF's mimic joints (finger_right ->
    # finger_left), so finger_left_joint/position gets registered twice and the
    # resource_manager rejects the gripper controller ("Not acceptable command
    # interfaces combination"); the second system's joints (bobby joint_1..6)
    # also fail to resolve. A single system registers every interface once.
    return f'''  <ros2_control name="CombinedGazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
{scara_control_joints()}
{bobby_control_joints()}
  </ros2_control>'''


def gazebo_plugin() -> str:
    # ONE plugin, GLOBAL namespace (no <ros> tag) -> one /controller_manager.
    return f'''  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="combined_gazebo_ros2_control">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>{CONTROLLERS_PLACEHOLDER}</parameters>
    </plugin>
  </gazebo>'''


def world_anchors() -> str:
    return f'''  <link name="world"/>
  <joint name="scara_world_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="{SCARA_PEDESTAL_XYZ}" rpy="0 0 0"/>
  </joint>
  <joint name="bobby_world_joint" type="fixed">
    <parent link="world"/>
    <child link="{BOBBY_PREFIX}base_link"/>
    <origin xyz="{BOBBY_PEDESTAL_XYZ}" rpy="0 0 0"/>
  </joint>'''


def main() -> None:
    with open(SCARA_URDF, encoding="utf-8") as f:
        scara_inner = strip_and_inner(f.read())
    with open(BOBBY_URDF, encoding="utf-8") as f:
        bobby_inner = prefix_bobby_links(strip_and_inner(f.read()))

    parts = [
        '<?xml version="1.0"?>',
        '<robot name="combined_cell">',
        world_anchors(),
        scara_inner,
        bobby_inner,
        combined_ros2_control(),
        gazebo_plugin(),
        '</robot>',
    ]
    os.makedirs(os.path.dirname(URDF_OUT), exist_ok=True)
    with open(URDF_OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(parts) + "\n")
    print(f"wrote {URDF_OUT}")


if __name__ == "__main__":
    main()