import argparse
import os
import sys

import numpy as np

import constants
from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    QuikPlan,
    StoppedPoseConstraint,
    PoseConstraint,
    Action,
    ActionType,
    SpeedConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=3500, MAX_TORQUE=0.7)

    # Configure the optimizer
    start_pose = constants.FAST_RACE_END_POSE
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        start_action=Action(ActionType.DEPLOY_INTAKE),
        apply_boundaries=False,
    )

    start_sweep_pose = (8.25, 4.25, 0.5 * np.pi)
    qp.add_waypoint(
        start_sweep_pose,
        10,
        intermediate_constraints=[SpeedConstraint(2.0)],
        end_constraints=[PoseConstraint(start_sweep_pose)],
    )

    mid_sweep_pose = (7, 4.5, 1.1 * np.pi)
    qp.add_waypoint(
        mid_sweep_pose,
        10,
        intermediate_constraints=[SpeedConstraint(2.0)],
        end_constraints=[PoseConstraint(mid_sweep_pose)],
    )

    hub_sweep_pose = (6, 4, 1.5 * np.pi)
    qp.add_waypoint(
        hub_sweep_pose,
        10,
        intermediate_constraints=[SpeedConstraint(2.0)],
        end_constraints=[PoseConstraint(hub_sweep_pose)],
    )

    second_mid_sweep_pose = (7, 3.5, 1.9 * np.pi)
    qp.add_waypoint(
        second_mid_sweep_pose,
        10,
        end_constraints=[PoseConstraint(second_mid_sweep_pose)],
    )

    sweep_rotate_pose = (8, 3, 1.5 * np.pi)
    qp.add_waypoint(
        sweep_rotate_pose,
        10,
        end_constraints=[PoseConstraint(sweep_rotate_pose)],
    )

    pre_bump_pose = (5.5, 2.5, np.pi)
    qp.add_waypoint(
        pre_bump_pose,
        10,
        end_constraints=[PoseConstraint(pre_bump_pose)],
    )

    qp.add_waypoint(
        constants.SCORING_POSE,
        10,
        end_constraints=[StoppedPoseConstraint(constants.SCORING_POSE)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # Plan the trajectory
    traj = qp.plan(quiet)
    path_name = os.path.basename(__file__)
    file_name = os.path.splitext(path_name)[0][4:]
    write_to_csv(traj, f"{file_name}")

    # Plot
    field.plot_traj(robot, traj, f"{file_name}.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, f"{file_name}.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
