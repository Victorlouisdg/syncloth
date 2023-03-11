import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
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
table.location = (0, 0, table_height)

towel_width = 0.5
towel_length = 1.2

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
towel.location = (0, 0, table_height + 0.003)

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

y = 0.5
left_arm_base.location = (0, y, table_height)

right_arm_base.location = (0, -y, table_height)
right_arm_base.rotation_euler = (0, 0, np.pi)

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


# Order the keypoints
def angle_2D(v0, v1):
    # TODO: document.
    x1, y1, *_ = v0
    x2, y2, *_ = v1
    dot = x1 * x2 + y1 * y2  # dot product between [x1, y1] and [x2, y2]
    det = x1 * y2 - y1 * x2  # determinant
    angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle


def get_ordered_keypoints(keypoints):
    """
    orders keypoints according to their angle w.r.t. a frame that is created by translating the world frame to the center of the cloth.
    the first keypoints is the one with the smallest, positive angle and they are sorted counter-clockwise.
    """
    keypoints = np.array(keypoints)
    center = np.mean(keypoints, axis=0)
    x_axis = np.array([1, 0])
    angles = [angle_2D(x_axis, keypoint - center) for keypoint in keypoints]
    angles = [angle % (2 * np.pi) for angle in angles]  # make angles positive from 0 to 2*pi
    keypoints_sorted = keypoints[np.argsort(angles)]
    return list(keypoints_sorted)


def set_joint_angles(joint_angles, arm_joints):
    for joint, joint_angle in zip(arm_joints.values(), joint_angles):
        joint.rotation_euler = (0, 0, joint_angle)


home_joints_left = np.deg2rad([0, -45, -90, 0, 0, 0])
home_joints_right = np.deg2rad([0, -45, -90, 0, 0, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset

ordered_keypoints = get_ordered_keypoints(keypoints_3D)


def skin(edge_mesh: bpy.types.Object, radius: float = 0.01):
    edge_mesh.modifiers.new(name="Skin", type="SKIN")
    edge_mesh.modifiers.new(name="Subdivision", type="SUBSURF")

    for vertex in edge_mesh.data.vertices:
        skin_vertex = edge_mesh.data.skin_vertices[""].data[vertex.index]
        skin_vertex.radius = (radius, radius)


def add_curve_mesh(points):
    """Add a string of edges between the given points."""
    edges = [(i, i + 1) for i in range(len(points) - 1)]
    mesh = bpy.data.meshes.new("Edge string")
    mesh.from_pydata(points, edges, [])
    mesh.update()
    curve_mesh = bpy.data.objects.new("Curve", mesh)
    bpy.context.collection.objects.link(curve_mesh)
    return curve_mesh


def quadratic_bezier(t, p0, p1, p2):
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


def animate_arm(start_self, end_self, start_other, arm_in_world, arm_joints, home_joints):
    middle = (start_self + end_self) / 2
    middle[2] += np.linalg.norm(end_self - start_self)
    points = np.vstack((start_self, middle, end_self))

    num_samples = 100
    t_range = np.linspace(0, 1, num_samples, endpoint=True)
    curve = np.array([quadratic_bezier(t, *points) for t in t_range])

    curve_mesh = add_curve_mesh(curve)
    ab.add_material(curve_mesh, color=(1, 0.5, 0))
    skin(curve_mesh, radius=0.005)

    Z = end_self - start_self
    Z = Z / np.linalg.norm(Z)
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    orientation_flat = np.column_stack((X, Y, Z))

    # rotate 45 degrees around local Y
    rotation_Y45 = Rotation.from_rotvec(-np.pi / 4 * Y).as_matrix()
    orientation_start = rotation_Y45 @ orientation_flat

    # rotate 90 degrees around local Y
    rotation_Y90 = Rotation.from_rotvec(-np.pi / 2 * Y).as_matrix()
    orientation_middle = rotation_Y90 @ orientation_flat

    # rotate 135 degrees around local Y
    rotation_Y135 = Rotation.from_rotvec(-3 * np.pi / 4 * Y).as_matrix()
    orientation_end = rotation_Y135 @ orientation_flat

    # use scipy SLERP to interpolate between the orientations
    orientations = np.array([orientation_start, orientation_middle, orientation_end])
    orientations = Rotation.from_matrix(orientations)
    slerp = Slerp([0.0, 0.5, 1.0], orientations)

    # loop through all the IK solutions

    grasp = np.identity(4)
    grasp[:3, :3] = orientation_start
    grasp[:3, 3] = curve[0]
    grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

    # solutions = ur5e.inverse_kinematics_with_tcp(grasp_in_arm, tcp_transform)

    # for i, solution in enumerate(solutions):
    #     for joint, joint_angle in zip(arm_joints.values(), *solution):
    #         joint.rotation_euler = (0, 0, joint_angle)
    #         # insert keyframe for rotation_euler at frame i + 1
    #         joint.keyframe_insert(data_path="rotation_euler", frame=i+1)

    # # Set up animation loop
    # bpy.context.scene.frame_end = len(solutions) + 1
    # bpy.context.scene.render.fps = 2

    # grasp = np.identity(4)
    # grasp[:3, :3] = orientation_start
    # grasp[:3, 3] = curve[0]

    # add_frame(grasp)

    prev_joints = home_joints

    for i in range(num_samples):
        translation = curve[i]
        interpolated_orientation = slerp(t_range[i]).as_matrix()

        grasp = np.identity(4)

        grasp[:3, :3] = interpolated_orientation
        grasp[:3, 3] = translation

        grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

        solutions = ur5e.inverse_kinematics_closest_with_tcp(grasp_in_arm, tcp_transform, *prev_joints)

        for joint, joint_angle in zip(arm_joints.values(), *solutions[0]):
            joint.rotation_euler = (0, 0, joint_angle)
            joint.keyframe_insert(data_path="rotation_euler", frame=i + 1)

        prev_joints = solutions[0][0]


animate_arm(
    ordered_keypoints[0],
    ordered_keypoints[1],
    ordered_keypoints[3],
    np.array(left_arm_base.matrix_world),
    left_arm_joints,
    home_joints_left,
)
animate_arm(
    ordered_keypoints[3],
    ordered_keypoints[2],
    ordered_keypoints[0],
    np.array(right_arm_base.matrix_world),
    right_arm_joints,
    home_joints_right,
)
