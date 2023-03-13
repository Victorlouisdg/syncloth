import bpy
import numpy as np
from mathutils import Matrix
from ur_analytic_ik import ur5e

from syncloth.robot_arms import add_ur_with_robotiq
from syncloth.visualization.frame import add_frame
from syncloth.visualization.inverse_kinematics import animate_ik_solutions_cycle

bpy.ops.object.delete()

arm_joints, _, arm_base, tool_link, _ = add_ur_with_robotiq()

tcp_offset = np.array([0.0, 0.0, 0.172])
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

animate_ik_solutions_cycle(arm_joints, solutions)
