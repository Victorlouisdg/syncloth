import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation
from ur_analytic_ik import ur3e, ur5e, ur10e

from syncloth.robot_arms import add_ur_with_robotiq, set_joint_angles
from syncloth.visualization.inverse_kinematics import keyframe_joints

# Main parameters
towel_length = 0.42
ur_type = "ur3e"
distance_between_robots = 0.5
robots_height = 0.9
arm_roll = np.deg2rad(90)


if ur_type == "ur3e":
    ur = ur3e
elif ur_type == "ur5e":
    ur = ur5e
else:
    ur = ur10e


bpy.ops.object.delete()

bpy.context.view_layer.update()


left_arm_joints, _, left_arm_base, _, _ = add_ur_with_robotiq(ur_type)
right_arm_joints, _, right_arm_base, _, right_gripper_joint = add_ur_with_robotiq(ur_type)
right_gripper_joint.rotation_euler = (0, 0, np.deg2rad(36))

y = distance_between_robots / 2
left_arm_base.location = (-0.05, y, robots_height)
left_arm_base.rotation_euler = (-arm_roll, 0, 0)

right_arm_base.location = (-0.05, -y, robots_height)
right_arm_base.rotation_euler = (arm_roll, 0, 0)

bpy.context.view_layer.update()

home_joints_left = np.deg2rad([-180, -30, 75, -135, -45, 0])
home_joints_right = np.deg2rad([0, -150, -75, -45, 45, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

keyframe_joints(left_arm_joints, home_joints_left, 0)
keyframe_joints(right_arm_joints, home_joints_right, 0)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset


urdf_path = "/home/idlab185/partnet-mobility-sample/44853/mobility.urdf"
cabinet_joints, _, cabinet_links = ab.import_urdf(urdf_path)
cabinet_base = [link for link in cabinet_links.values() if link.parent is None][0]
cabinet_base.scale = (0.5, 0.5, 0.5)
# Apply scale
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)


cabinet_base.location = (0.7, 0, 0.4)

top_drawer_joint = cabinet_joints["joint_0"]

openness_range = np.linspace(0, 0.2, 50)


grasp_location = np.array([0.41, 0.0, 0.69])

Z = np.array([0, 0, -1])
X = np.array([1, 0, 0])
Y = np.cross(Z, X)
grasp_orientation = np.column_stack((X, Y, Z))

# rotation orientation 45 degrees around its local y
rotation_Y = Rotation.from_rotvec(np.pi / 4 * Y).as_matrix()
grasp_orientation = rotation_Y @ grasp_orientation

arm_in_world = (np.array(right_arm_base.matrix_world),)
prev_joints = home_joints_right
arm_joints = right_arm_joints


for i, value in enumerate(openness_range):
    print(value)

    grasp = np.identity(4)
    grasp[:3, 3] = grasp_location
    grasp[0, 3] -= value
    grasp[:3, :3] = grasp_orientation

    # add_frame(grasp)

    grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

    solutions = ur.inverse_kinematics_closest_with_tcp(grasp_in_arm, tcp_transform, *prev_joints)

    if len(solutions) == 0:
        print(f"No solution found for pose {i}: \n", grasp_in_arm)
        break

    keyframe_joints(arm_joints, *solutions[0], i + 1)
    prev_joints = solutions[0][0]

    top_drawer_joint.location.z = 2 * value  # not sure why this 2 is needed
    top_drawer_joint.keyframe_insert(data_path="location", index=2, frame=i + 1)

camera = bpy.data.objects["Camera"]
camera.location = (-0.3, 1.26, 1.87)
camera.rotation_euler = (np.deg2rad(54), 0, np.deg2rad(-152))

scene = bpy.context.scene
scene.frame_end = 100
scene.render.engine = "CYCLES"
scene.cycles.samples = 64
