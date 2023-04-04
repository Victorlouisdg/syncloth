import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from airo_camera_toolkit.cameras.zed2i import Zed2i
from airo_camera_toolkit.reprojection import reproject_to_frame_z_plane
from airo_camera_toolkit.utils import ImageConverter
from airo_dataset_tools.pose import Pose
from airo_robots.grippers.hardware.robotiq_2f85_urcap import Robotiq2F85
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_spatial_algebra import SE3Container

from syncloth.grasping.slide_grasp import slide_grasp_constant_orientation_pose_trajectory, slide_grasp_orientation


def draw_clicked_grasp(image, clicked_image_points, current_mouse_point):
    """If we don't have tow clicks yet, draw a line between the first point and the current cursor position."""
    for point in clicked_image_points:
        image = cv2.circle(image, point, 5, (0, 255, 0), thickness=2)

    if len(clicked_image_points) >= 1:
        first_point = clicked_image_points[0]
        second_point = current_mouse_point[0]

        if len(clicked_image_points) >= 2:
            second_point = clicked_image_points[1]

        image = cv2.arrowedLine(image, first_point, second_point, color=(0, 255, 0), thickness=1)
    return image


def draw_pose(image, pose_in_base, camera_in_base, intrinsics):
    pose_in_camera = np.linalg.inv(camera_in_base) @ pose_in_base
    rvec = pose_in_camera[:3, :3]
    tvec = pose_in_camera[:3, -1]
    image = cv2.drawFrameAxes(image, intrinsics, np.zeros(4), rvec, tvec, 0.1)
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

    def mouse_callback(event, x, y, flags, parm):
        if len(clicked_image_points) >= 2:
            return

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
        image = draw_clicked_grasp(image, clicked_image_points, current_mouse_point)

        image = draw_pose(image, np.identity(4), camera_in_base, intrinsics_matrix)

        if len(clicked_image_points) == 2:
            points_in_image = np.array(clicked_image_points)
            points_in_world = reproject_to_frame_z_plane(
                points_in_image, intrinsics_matrix, camera_in_base, table_height_offset
            )

            slide_start = points_in_world[0]
            slide_end = points_in_world[1]

            # print(slide_start, slide_end)

            grasp_approach_direction = slide_end - slide_start
            approach_distance = np.linalg.norm(grasp_approach_direction)
            grasp_approach_direction /= approach_distance

            speed = 0.1
            hover_height = 0.05
            approach_angle = np.pi / 4

            grasp_location = slide_end

            # robotiq_gripper_open_filtertip_displacement = np.array([0.085 / 2, 0, 0.0135])

            vector_to_tip = calculate_fingertip_displacement_when_open(approach_angle)
            height_offset = np.array([0, 0, -vector_to_tip[2]])

            fingertip_thickness = 0.008
            thickness_offset = [0, 0, np.cos(approach_angle) * fingertip_thickness]

            margin = 0.001

            grasp_location += height_offset + thickness_offset + margin

            grasp_trajectory = slide_grasp_constant_orientation_pose_trajectory(
                grasp_location,
                grasp_approach_direction,
                approach_distance,
                approach_angle,
                hover_height,
                speed,
            )

            start_pose = grasp_trajectory(0.0)

            robot.move_to_tcp_pose(start_pose, joint_speed=0.4).wait()

            control_period = 0.01

            print(grasp_trajectory.duration)

            t = 0.0
            while t < grasp_trajectory.duration:
                robot.servo_to_tcp_pose(grasp_trajectory(t), control_period).wait()
                t += control_period

            robot.rtde_control.servoStop()

            gripper.close().wait()

            end_pose = grasp_trajectory(grasp_trajectory.duration)
            retreat_pose = end_pose.copy()
            retreat_pose[2, 3] += 0.1

            robot.move_linear_to_tcp_pose(retreat_pose).wait()
            time.sleep(3)
            gripper.open().wait()
            robot.move_to_joint_configuration(home_joints, joint_speed=1.0).wait()
            clicked_image_points = []

        cv2.imshow(window_name, image)
        key = cv2.waitKey(10)

        if key == ord("q"):
            cv2.destroyAllWindows()
            break
