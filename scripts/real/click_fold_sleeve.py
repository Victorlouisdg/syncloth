import os
import sys
from pathlib import Path

import cv2
import numpy as np
from airo_camera_toolkit.cameras.zed2i import Zed2i
from airo_camera_toolkit.reprojection import project_frame_to_image_plane, reproject_to_frame_z_plane
from airo_camera_toolkit.utils import ImageConverter
from airo_dataset_tools.pose import Pose
from airo_robots.grippers.hardware.robotiq_2f85_urcap import Robotiq2F85
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_spatial_algebra import SE3Container

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.geometry import flat_orientation, pitch_gripper_orientation
from syncloth.grasping.slide_grasp import (
    slide_grasp_constant_orientation_pose_trajectory,
    slide_grasp_orientation,
    slide_grasp_orthogonal_approach_direction,
)
from syncloth.paths.circular_arc import circular_arc_orientation_path


def draw_points(image, points):
    for point in clicked_image_points:
        image = cv2.circle(image, point, 5, (0, 255, 0), thickness=2)
    return image


def draw_pose(image, pose_in_base, camera_in_base, intrinsics):
    pose_in_camera = np.linalg.inv(camera_in_base) @ pose_in_base
    rvec = pose_in_camera[:3, :3]
    tvec = pose_in_camera[:3, -1]
    image = cv2.drawFrameAxes(image, intrinsics, np.zeros(4), rvec, tvec, 0.1)
    return image


def draw_line_3d(image, line, intrinsics, robot_to_camera_transform):

    point, direction = line
    start = point
    end = point + direction

    project_points = np.array([start, end])

    start_2d, end_2d = project_frame_to_image_plane(project_points, intrinsics, robot_to_camera_transform).astype(int)
    print(start, end, start_2d, end_2d)

    image = cv2.line(image, start_2d, end_2d, color=(0, 255, 0), thickness=1)

    return image


def calculate_fingertip_displacement_when_open(approach_angle):
    grasp_approach_direction = np.array([1, 0, 0])

    grasp_orientation = slide_grasp_orientation(grasp_approach_direction, angle=approach_angle)
    gripper_X = grasp_orientation[:, 0]  # first column
    gripper_Z = grasp_orientation[:, 2]  # third column

    dx = 0.085 / 2
    dz = 0.0135

    fingertip_displacement = dx * -gripper_X + dz * -gripper_Z
    return fingertip_displacement


def create_fold(keypoints: dict):
    grasp_speed = 0.1
    hover_height = 0.05
    grasp_depth = 0.06
    approach_margin = 0.05
    approach_distance = grasp_depth + approach_margin
    approach_angle = np.pi / 4
    fold_end_height = 0.07
    fold_end_angle = np.deg2rad(60)

    neck_left = keypoints["neck_left"]
    sleeve_left_top = keypoints["sleeve_left_top"]
    sleeve_left_bottom = keypoints["sleeve_left_bottom"]
    armpit_left = keypoints["armpit_left"]

    fold_line_point = armpit_left
    armpit_to_neck = neck_left - armpit_left
    fold_line_direction = armpit_to_neck / np.linalg.norm(armpit_to_neck)
    fold_line = (fold_line_point, fold_line_direction)

    # Get grasp location and the gripper orientation for a perfectly flat grasp
    approach_direction = slide_grasp_orthogonal_approach_direction(sleeve_left_bottom, sleeve_left_top)
    grasp_location = (sleeve_left_top + sleeve_left_bottom) / 2
    grasp_location += grasp_depth * approach_direction

    vector_to_tip = calculate_fingertip_displacement_when_open(approach_angle)
    height_offset = np.array([0, 0, -vector_to_tip[2]])
    fingertip_thickness = 0.008
    thickness_offset = [0, 0, np.cos(approach_angle) * fingertip_thickness]
    margin = 0.001
    grasp_location += height_offset + thickness_offset + margin

    grasp_orientation_flat = flat_orientation(approach_direction)
    circular_orientations = circular_arc_orientation_path(grasp_orientation_flat, fold_line_direction, np.pi)
    end_orientation_flat = circular_orientations.end
    middle_orientation = circular_orientations(np.pi / 2)

    # Pitch the orientation of the start and end
    grasp_orientation_pitched = pitch_gripper_orientation(grasp_orientation_flat, -approach_angle)
    end_orientation_pitched = pitch_gripper_orientation(end_orientation_flat, fold_end_angle)

    orientations = [
        grasp_orientation_pitched,
        middle_orientation,
        end_orientation_pitched,
    ]
    fold_arc_trajectory = fold_arc_circular_slerp_trajectory(
        fold_line, grasp_location, orientations, fold_end_height, peak_speed=0.2
    )

    grasp_trajectory = slide_grasp_constant_orientation_pose_trajectory(
        grasp_location,
        approach_direction,
        approach_distance,
        approach_angle,
        hover_height,
        grasp_speed,
    )

    return fold_line, [grasp_trajectory, fold_arc_trajectory, None]


def execute_open_loop_trajectory(trajectory: Path, robot, control_period=0.01):
    t = 0.0
    while t < trajectory.duration:
        robot.servo_to_tcp_pose(trajectory(t), control_period).wait()
        t += control_period
    robot.rtde_control.servoStop()


def execute_fold(trajectories, robot, gripper):
    grasp_trajectory, fold_arc_trajectory, _ = trajectories

    start_pose = grasp_trajectory(0.0)
    robot.move_to_tcp_pose(start_pose, joint_speed=0.4).wait()

    execute_open_loop_trajectory(grasp_trajectory, robot)

    robot.rtde_control.servoStop()

    gripper.close().wait()

    execute_open_loop_trajectory(fold_arc_trajectory, robot)

    gripper.open().wait()


if __name__ == "__main__":  # noqa: C901
    if not os.path.exists(Path(__file__).parent / "camera_pose.json"):
        print("Please run camera_calibration.py first.")
        sys.exit(0)

    pose_saved = Pose.parse_file("camera_pose.json")
    position = pose_saved.position_in_meters
    euler_angles = pose_saved.rotation_euler_xyz_in_radians

    position_array = np.array([position.x, position.y, position.z])
    euler_angles_array = np.array([euler_angles.roll, euler_angles.pitch, euler_angles.yaw])

    camera_in_base = SE3Container.from_euler_angles_and_translation(
        euler_angles_array, position_array
    ).homogeneous_matrix

    table_height_offset = -0.092  # Table is 92 mm lower than the robot base

    robot_ip = "10.42.0.162"
    robot = URrtde(robot_ip, URrtde.UR3E_CONFIG)

    gripper = Robotiq2F85(robot_ip)

    # home_joints = np.deg2rad([0, -60, -90, -120, 90, 90])
    home_joints_ur5e_split_left = np.deg2rad([-180, -45, -95, -130, 90, 90])

    home_joints = home_joints_ur5e_split_left

    robot.move_to_joint_configuration(home_joints, joint_speed=1.0).wait()
    gripper.open().wait()

    zed = Zed2i(resolution=Zed2i.RESOLUTION_720, fps=30)
    intrinsics_matrix = zed.intrinsics_matrix()

    current_mouse_point = [(0, 0)]  # has to be a list so that the callback can edit it
    clicked_image_points = []

    desired_keypoints = [
        "neck_left",
        "sleeve_left_top",
        "sleeve_left_bottom",
        "armpit_left",
    ]

    def mouse_callback(event, x, y, flags, parm):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_image_points.append((x, y))
        elif event == cv2.EVENT_MOUSEMOVE:
            current_mouse_point[0] = x, y

    window_name = "Camera feed"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    grasp_executed = False

    while True:
        _, h, w = zed.get_rgb_image().shape
        image = zed.get_rgb_image()
        image = ImageConverter(image).image_in_opencv_format
        image = draw_points(image, clicked_image_points)

        image = draw_pose(image, np.identity(4), camera_in_base, intrinsics_matrix)

        i = len(clicked_image_points)
        # print(i, "/", len(desired_keypoints))

        if len(clicked_image_points) != len(desired_keypoints):
            cv2.putText(
                image,
                f"Please click: {desired_keypoints[i]}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            points_in_image = np.array(clicked_image_points)
            points_in_world = reproject_to_frame_z_plane(
                points_in_image, intrinsics_matrix, camera_in_base, table_height_offset
            )
            keypoints = {desired_keypoints[i]: points_in_world[i, :] for i in range(len(desired_keypoints))}

            fold_line, fold_trajectories = create_fold(keypoints)
            image = draw_line_3d(image, fold_line, intrinsics_matrix, np.linalg.inv(camera_in_base))

            grasp_trajectory = fold_trajectories[0]
            image = draw_pose(image, grasp_trajectory.start, camera_in_base, intrinsics_matrix)

            fold_arc_trajectory = fold_trajectories[1]
            for t in np.linspace(0, fold_arc_trajectory.duration, 5):
                image = draw_pose(image, fold_arc_trajectory(t), camera_in_base, intrinsics_matrix)

            cv2.imshow(window_name, image)
            key = cv2.waitKey(10)

            execute_fold(fold_trajectories, robot, gripper)
            robot.move_to_joint_configuration(home_joints, joint_speed=1.0).wait()
            clicked_image_points = []

        # fold here
        # clicked_image_points = []

        cv2.imshow(window_name, image)
        key = cv2.waitKey(10)

        if key == ord("q"):
            cv2.destroyAllWindows()
            break
