import numpy as np
import torch

import pr2
from pr2.envs import Env
from pr2.motion_generator.keyboardinput import KeyboardCmd
from pr2.object import CubeConfig
from pr2.pathcfg import DataPath, load_config

"""
This tutorial demonstrates how to use keyboard to control the robot to sort the cubes.

- It is using motion modules in pr2/motion_generator for motions of the robot.
  As for the hotkeys to control the robot, please refer to doc/keyboard-layout.png for details.

- Once the robot picks up the cube, use motions with pre-fix "box" to control the robot to move with the cube.

- Be ware of this tutorial includes contents from "scene" and "object" modules in pr2.

"""

START_POSITION = torch.tensor([[0.0, -0.592, 0.194], [0.0, 0.593, 0.194]])
START_ORIENT = torch.tensor([[0.7, 0, 0, -0.7], [0.7, 0, 0, -0.7]])


def add_objects_to_scene(env):
    red_cube = CubeConfig(
        object_id="red_cube",
        semantic_label="red_cube",
        position=(0.0, -1.167, 0.0),
        scale=(0.1, 0.1, 0.1),
        color=torch.tensor([1.0, 0.0, 0.0]),
    )
    green_cube = CubeConfig(
        object_id="green_cube",
        semantic_label="green_cube",
        position=(0.33, 1.167, 0.0),
        scale=(0.1, 0.1, 0.1),
        color=torch.tensor([0, 0.18, -26]),
    )
    env.add_cube(red_cube)
    env.add_cube(green_cube)


def main():
    # Load the default environment configuration from the YAML file
    default_env_cfg = load_config(f"{DataPath.cfg}/default_env_cfg.yaml")

    # Set up the environment to use the "pick_place" scene
    default_env_cfg["scene"]["folder_name"] = "pick_place"
    default_env_cfg["scene"]["file_name"] = "pick_place.usd"

    # Set the number of environments (robots) to 2
    default_env_cfg["env"]["num_robots"] = 2

    # Initialize the environment with the configuration
    env = Env(default_env_cfg)

    # Load the robot into the environment
    env.load_robot()

    # Add objects to the scene for the robot to interact with
    add_objects_to_scene(env)

    # Note: Always call `env.reset()` after initializing the environment
    # and loading the robot or objects.
    env.reset()

    # Get the robot and set its initial pose (position and orientation)
    robots = env.get_robot()
    # WARNING: You MUST set the robot's initial position and orientation AFTER calling `env.reset()`.
    # Otherwise, the robot's initial pose will be set according to the root state configuration
    # in `biped_aelos_cfg.yaml`.
    robots.set_root_poses_w(
        positions=START_POSITION,
        orientations=START_ORIENT,
    )

    # Initialize the keyboard command input handler after reset env
    kb_input = KeyboardCmd(env, START_POSITION, START_ORIENT)
    kb_input.initialize()

    # Initialize action array for the robots (2 robots, 17 dof each)
    action = np.zeros((env.num_robots, env.num_actions))

    while pr2.app.is_running:
        kb_input.get_keyboard_cmd()
        action_cmd = kb_input.action_cmd
        action_name, robot_id, motion_sequence = (
            action_cmd[0],
            action_cmd[1],
            action_cmd[2],
        )

        # execute motion sequence for each frame
        while motion_sequence:
            action[robot_id, :] = motion_sequence.popleft()
            env.step(action)
            if robot_id is not None and not motion_sequence:
                print(f"Robot_{robot_id} {action_name}")
        # If simulation is stopped, then exit.
        # Important: Please do not remove this line.
        if env.is_stopped:
            break

        env.step(action)


if __name__ == "__main__":
    main()
    pr2.app.close()
