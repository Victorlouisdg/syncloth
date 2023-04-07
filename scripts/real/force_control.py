from airo_robots.grippers.hardware.robotiq_2f85_urcap import Robotiq2F85
from airo_robots.manipulators.hardware.ur_rtde import URrtde

robot_ip = "10.42.0.162"
robot = URrtde(robot_ip, URrtde.UR3E_CONFIG)

gripper = Robotiq2F85(robot_ip)

import numpy as np

task_frame = np.identity(4)
task_frame_ur = URrtde._convert_homegeneous_pose_to_rotvec_pose(task_frame)


selection_vector = [0, 0, 1, 0, 0, 0]  # Compliant in z direction
wrench = [0, 0, -5, 0, 0, 0]  # Slight downward force
type = 2  # No transform
limits = [1, 1, 0.02, 1, 1, 1]  # 2cm/s

robot.rtde_control.forceMode(task_frame_ur, selection_vector, wrench, type, limits)

try:
    while True:
        pass
except KeyboardInterrupt:
    robot.rtde_control.forceModeStop()
