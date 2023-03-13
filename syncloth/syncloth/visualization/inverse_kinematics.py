from typing import List

import bpy
import numpy as np


def keyframe_joints(arm_joints: List[bpy.types.Object], joint_angles: np.ndarray, frame: int):
    for joint, joint_angle in zip(arm_joints.values(), joint_angles):
        joint.rotation_euler = (0, 0, joint_angle)
        joint.keyframe_insert(data_path="rotation_euler", index=2, frame=frame)


def animate_ik_solutions_cycle(arm_joints: List[bpy.types.Object], solutions: List[np.ndarray]):
    for i, solution in enumerate(solutions):
        keyframe_joints(arm_joints, solution[0], i + 1)  # TODO remove need for [0]

    # Set up animation loop
    bpy.context.scene.frame_end = len(solutions) + 1
    bpy.context.scene.render.fps = 2
