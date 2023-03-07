import airo_blender as ab
import bpy
import numpy as np
from ur_analytic_ik import ur5e

from syncloth.visualization.frame import add_frame

bpy.ops.object.delete()
bpy.ops.mesh.primitive_plane_add()
table = bpy.context.object

table_width = 0.7
table_length = 1.4
table_height = 0.7
table.scale = (table_width / 2.0, table_length / 2.0, 1)
table.rotation_euler = (0, 0, np.pi / 2)

distance_robot_to_table = 0.05
table_x = table_length / 2.0 + distance_robot_to_table
table.location = (table_x, 0, table_height)

towel_width = 0.5
towel_length = 0.7

vertices = [
    np.array([-towel_width / 2, -towel_length / 2, 0.0]),
    np.array([-towel_width / 2, towel_length / 2, 0.0]),
    np.array([towel_width / 2, towel_length / 2, 0.0]),
    np.array([towel_width / 2, -towel_length / 2, 0.0]),
]
edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
faces = [(0, 1, 2, 3)]

name = "Towel"
mesh = bpy.data.meshes.new(name)
mesh.from_pydata(vertices, edges, faces)
mesh.update()
towel = bpy.data.objects.new(name, mesh)
bpy.context.collection.objects.link(towel)

towel.rotation_euler = (0, 0, np.pi / 2)

distance_towel_from_edge = 0.05
towel_x = towel_length / 2.0 + distance_robot_to_table + distance_towel_from_edge
towel.location = (towel_x, 0, table_height + 0.003)

ab.add_material(towel, (0.5, 0.5, 0.8))

bpy.context.view_layer.update()
keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]

# Debug visualization: add a small sphere at each keypoint
for i, keypoint in enumerate(keypoints_3D):
    bpy.ops.mesh.primitive_uv_sphere_add(location=keypoint, scale=(0.01, 0.01, 0.01))
    sphere = bpy.context.object
    ab.add_material(sphere, (0.8, 0, 0))


# Load UR5e robot arm
def add_arm(left=True):
    urdf_path = "/home/idlab185/urdf-workshop/universal_robots/ros/ur5e/ur5e.urdf"
    free_joint_empties, joint_empties, link_empties = ab.import_urdf(urdf_path)
    base_links = [link for link in link_empties.values() if link.parent is None]
    base_link = base_links[0]

    y = 0.35 if left else -0.35
    base_link.location = (0, y, 0.75)
    return base_link, free_joint_empties


left_arm_base, left_arm_joints = add_arm(left=True)
right_arm_base, right_arm_joints = add_arm(left=False)

# Get the two keypoints with the greatest distance to the robot
keypoints_by_distance = sorted(keypoints_3D, key=lambda x: np.linalg.norm(x))
close_keypoints = keypoints_by_distance[:2]
far_keypoints = keypoints_by_distance[-2:]  # sort is ascending, so last two are the farthest

close_keypoints_right_to_left = sorted(close_keypoints, key=lambda x: x[1])
close_right = close_keypoints_right_to_left[0]
close_left = close_keypoints_right_to_left[1]

far_keypoints_right_to_left = sorted(far_keypoints, key=lambda x: x[1])
far_right = far_keypoints_right_to_left[0]
far_left = far_keypoints_right_to_left[1]


def set_joint_angles(joint_angles, arm_joints):
    for joint, joint_angle in zip(arm_joints.values(), joint_angles):
        joint.rotation_euler = (0, 0, joint_angle)


home_joints_left = np.deg2rad([0, -120, -90, 210, 0, 0])
home_joints_right = np.deg2rad([180, -60, 90, -30, 0, 0])

# set_joint_angles(home_joints_left, left_arm_joints)
# set_joint_angles(home_joints_right, right_arm_joints)


def pose_arm(far_self, far_other, arm_in_world, arm_joints, home_joints):
    Z = far_other - far_self
    Z = Z / np.linalg.norm(Z)
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    orientation = np.column_stack((X, Y, Z))
    translation = far_self

    grasp = np.identity(4)
    grasp[:3, :3] = orientation
    grasp[:3, 3] = translation

    add_frame(grasp, name="grasp")

    # arm_in_world = np.array(arm_base.matrix_world)
    grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

    solutions = ur5e.inverse_kinematics(grasp_in_arm)
    print("solutions:")
    print(solutions)
    print("home_joints:")
    print(home_joints)

    # solution_closest_to_home
    min_distance = np.inf
    solution_closest_to_home = None

    for solution in solutions:
        distance = 0
        for i in range(6):
            x = solution[0, i]
            y = home_joints[i]
            # take into account the fact that angles wrap around
            angle_delta = min((2 * np.pi) - abs(x - y), abs(x - y))
            distance += angle_delta

        if distance < min_distance:
            min_distance = distance
            solution_closest_to_home = solution

    for joint, joint_angle in zip(arm_joints.values(), *solution_closest_to_home):
        joint.rotation_euler = (0, 0, joint_angle)
        joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)


pose_arm(far_left, far_right, np.array(left_arm_base.matrix_world), left_arm_joints, home_joints_left)
pose_arm(far_right, far_left, np.array(right_arm_base.matrix_world), right_arm_joints, home_joints_right)
