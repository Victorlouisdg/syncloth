import airo_blender as ab
import numpy as np


def set_joint_angles(joint_angles, arm_joints):
    for joint, joint_angle in zip(arm_joints.values(), joint_angles):
        joint.rotation_euler = (0, 0, joint_angle)


def add_robotiq():
    # load gripper
    urdf_path = "/home/idlab185/urdf-workshop/robotiq/robotiq_2f85_aprice/robotiq_2f85_v3.urdf"
    gripper_joints, _, gripper_links = ab.import_urdf(urdf_path)
    gripper_bases = [link for link in gripper_links.values() if link.parent is None]
    gripper_base = gripper_bases[0]

    gripper_joint = gripper_joints["finger_joint"]
    gripper_joint.rotation_euler = (0, 0, np.deg2rad(42))

    return gripper_base


def add_ur_with_robotiq(name: str = "ur5e"):
    # load arm
    urdf_path = f"/home/idlab185/urdf-workshop/universal_robots/ros/{name}/{name}.urdf"
    arm_joints, _, arm_links = ab.import_urdf(urdf_path)
    arm_bases = [link for link in arm_links.values() if link.parent is None]
    arm_base = arm_bases[0]
    tool_link = arm_links["wrist_3_link"]

    # load gripper
    urdf_path = "/home/idlab185/urdf-workshop/robotiq/robotiq_2f85_aprice/robotiq_2f85_v3.urdf"
    gripper_joints, _, gripper_links = ab.import_urdf(urdf_path)
    gripper_bases = [link for link in gripper_links.values() if link.parent is None]
    gripper_base = gripper_bases[0]
    gripper_base.parent = tool_link

    gripper_joint = gripper_joints["finger_joint"]

    return arm_joints, arm_links, arm_base, tool_link, gripper_joint
