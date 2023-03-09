import airo_blender as ab
import bpy
import numpy as np
from mathutils import Matrix
from ur_analytic_ik import ur5e

from syncloth.visualization.frame import add_frame

bpy.ops.object.delete()


# Load UR5e arm with robotiq gripper attached
def add_robot():
    # load arm
    urdf_path = "/home/idlab185/urdf-workshop/universal_robots/ros/ur5e/ur5e.urdf"
    arm_joints, _, arm_links = ab.import_urdf(urdf_path)
    arm_bases = [link for link in arm_links.values() if link.parent is None]
    arm_base = arm_bases[0]
    tool_link = arm_links["tool0"]

    # load gripper
    urdf_path = "/home/idlab185/urdf-workshop/robotiq/robotiq_2f85_danfoa/robotiq_2f85_v3.urdf"
    gripper_joints, _, gripper_links = ab.import_urdf(urdf_path)
    gripper_bases = [link for link in gripper_links.values() if link.parent is None]
    gripper_base = gripper_bases[0]

    # can be posed
    gripper_joints["finger_joint"].rotation_euler.z = np.deg2rad(30)

    gripper_base.parent = tool_link
    return arm_base, tool_link, arm_joints


arm_base, tool_link, arm_joints = add_robot()

tcp_offset = np.array([0.0, 0.0, 0.17])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset

tcp_frame = add_frame(np.identity(4), name="tcp")
tcp_frame.parent = tool_link
tcp_frame.matrix_parent_inverse = Matrix(tcp_transform)

Z = np.array([0.0, 1.0, 0.0])
X = np.array([0.0, 0.0, 1.0])
Y = np.cross(Z, X)
orientation = np.column_stack((X, Y, Z))
translation = np.array([0.5, 0.0, 0.0])
tcp_pose = np.identity(4)
tcp_pose[:3, :3] = orientation
tcp_pose[:3, 3] = translation


solutions = ur5e.inverse_kinematics_with_tcp(tcp_pose, tcp_transform)

for i, solution in enumerate(solutions):
    for joint, joint_angle in zip(arm_joints.values(), *solution):
        joint.rotation_euler = (0, 0, joint_angle)
        # insert keyframe for rotation_euler at frame i + 1
        joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)


# Set up animation loop
bpy.context.scene.frame_end = len(solutions) + 1
bpy.context.scene.render.fps = 2
