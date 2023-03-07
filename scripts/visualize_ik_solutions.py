import airo_blender as ab
import bpy
import numpy as np
from ur_analytic_ik import ur5e

from syncloth.visualization.frame import add_frame

bpy.ops.object.delete()


# Load UR5e robot arm
def add_arm(left=True):
    urdf_path = "/home/idlab185/urdf-workshop/universal_robots/ros/ur5e/ur5e.urdf"
    free_joint_empties, joint_empties, link_empties = ab.import_urdf(urdf_path)
    base_links = [link for link in link_empties.values() if link.parent is None]
    base_link = base_links[0]
    return base_link, free_joint_empties


arm_base, arm_joints = add_arm(left=True)

Z = np.array([0.0, 1.0, 0.0])
X = np.array([0.0, 0.0, 1.0])
Y = np.cross(Z, X)
orientation = np.column_stack((X, Y, Z))
translation = np.array([0.5, 0.0, 0.0])
eef_pose = np.identity(4)
eef_pose[:3, :3] = orientation
eef_pose[:3, 3] = translation

eef_pose_empty = add_frame(eef_pose, name="eef_pose")

solutions = ur5e.inverse_kinematics(eef_pose)
solution0 = solutions[0]

for i, solution in enumerate(solutions):
    for joint, joint_angle in zip(arm_joints.values(), *solution):
        joint.rotation_euler = (0, 0, joint_angle)
        # insert keyframe for rotation_euler at frame i + 1
        joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)


# Set up animation loop
bpy.context.scene.frame_end = len(solutions) + 1
bpy.context.scene.render.fps = 2
