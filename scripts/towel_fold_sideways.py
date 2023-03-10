import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_analytic_ik import ur5e

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

distance_towel_from_edge = 0.1
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


# Load UR5e arm with robotiq gripper attached
def add_robot():
    # load arm
    urdf_path = "/home/idlab185/urdf-workshop/universal_robots/ros/ur5e/ur5e.urdf"
    arm_joints, _, arm_links = ab.import_urdf(urdf_path)
    arm_bases = [link for link in arm_links.values() if link.parent is None]
    arm_base = arm_bases[0]
    tool_link = arm_links["wrist_3_link"]

    # load gripper
    urdf_path = "/home/idlab185/urdf-workshop/robotiq/robotiq_2f85_aprice/robotiq_2f85_v3.urdf"
    gripper_joints, _, gripper_links = ab.import_urdf(urdf_path)
    gripper_bases = [link for link in gripper_links.values() if link.parent is None]
    gripper_base = gripper_bases[0]

    # can be posed
    gripper_joints["finger_joint"].rotation_euler.z = np.deg2rad(30)

    gripper_base.parent = tool_link
    return arm_base, arm_joints


left_arm_base, left_arm_joints = add_robot()
right_arm_base, right_arm_joints = add_robot()

y = 0.25
left_arm_base.location = (-0.05, y, 1.1)
left_arm_base.rotation_euler = (-np.pi / 2, 0, 0)

right_arm_base.location = (-0.05, -y, 1.1)
right_arm_base.rotation_euler = (np.pi / 2, 0, 0)

bpy.context.view_layer.update()

# Positions the camera
camera = bpy.data.objects["Camera"]
camera.location = (0, 0, 1.4)
camera.rotation_euler = (np.deg2rad(30), 0, np.deg2rad(-90))

# set lens and sensor size
camera.data.lens = 2.12
camera.data.sensor_width = 5.376
camera.data.sensor_height = 3.04

camera.data.display_size = 0.2

# set resolution to 720p
scene = bpy.context.scene
scene.render.resolution_x = 1280
scene.render.resolution_y = 720


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


# home_joints_left = np.deg2rad([0, -150, -70, -30, 130, 30])
home_joints_left = np.deg2rad([-180, -30, 75, -135, -45, 0])
home_joints_right = np.deg2rad([0, -150, -75, -45, 45, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset


def add_edge_string(points):
    """Add a string of edges between the given points."""
    edges = [(i, i + 1) for i in range(len(points) - 1)]
    mesh = bpy.data.meshes.new("Edge string")
    mesh.from_pydata(points, edges, [])
    mesh.update()
    edge_string = bpy.data.objects.new("Edge string", mesh)
    bpy.context.collection.objects.link(edge_string)
    return edge_string


def quadratic_bezier(t, p0, p1, p2):
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


def animate_arm(far_self, close_self, far_other, arm_in_world, arm_joints, home_joints):
    middle = (far_self + close_self) / 2
    middle[2] += 0.3
    points = np.vstack((far_self, middle, close_self))

    num_samples = 100
    t_range = np.linspace(0, 1, num_samples, endpoint=True)
    curve = np.array([quadratic_bezier(t, *points) for t in t_range])

    add_edge_string(curve)

    Z = far_other - far_self
    Z = Z / np.linalg.norm(Z)
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    orientation = np.column_stack((X, Y, Z))

    # rotate 45 degrees around local Y
    rotation_Y = R.from_rotvec(-np.pi / 4 * Y).as_matrix()
    orientation = rotation_Y @ orientation

    # loop through all the IK solutions

    # grasp = np.identity(4)
    # grasp[:3, :3] = orientation
    # grasp[:3, 3] = curve[0]
    # grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

    # solutions = ur5e.inverse_kinematics_with_tcp(grasp_in_arm, tcp_transform)

    # for i, solution in enumerate(solutions):
    #     for joint, joint_angle in zip(arm_joints.values(), *solution):
    #         joint.rotation_euler = (0, 0, joint_angle)
    #         # insert keyframe for rotation_euler at frame i + 1
    #         joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)

    # # Set up animation loop
    # bpy.context.scene.frame_end = len(solutions) + 1
    # bpy.context.scene.render.fps = 2

    prev_joints = home_joints

    for i in range(num_samples):
        translation = curve[i]

        grasp = np.identity(4)
        grasp[:3, :3] = orientation
        grasp[:3, 3] = translation

        grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

        solutions = ur5e.inverse_kinematics_closest_with_tcp(grasp_in_arm, tcp_transform, *prev_joints)
        # print(len(solutions))

        for joint, joint_angle in zip(arm_joints.values(), *solutions[0]):
            joint.rotation_euler = (0, 0, joint_angle)
            joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)

        prev_joints = solutions[0][0]


animate_arm(far_left, close_left, far_right, np.array(left_arm_base.matrix_world), left_arm_joints, home_joints_left)
animate_arm(
    far_right, close_right, far_left, np.array(right_arm_base.matrix_world), right_arm_joints, home_joints_right
)
