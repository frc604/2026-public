import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

import constants
from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    AngularConstraint,
    QuikPlan,
    StoppedPoseConstraint,
    PoseConstraint,
    Action,
    ActionType,
    XConstraint,
    YConstraint,
    SpeedConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=3500, MAX_TORQUE=0.7)

    # Configure the optimizer
    start_pose = (8.2, 2, 2.0 * np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    second_sweep_start_pose = (9, 2, 2.5 * np.pi)
    qp.add_waypoint(
        second_sweep_start_pose,
        10,
        end_constraints=[PoseConstraint(second_sweep_start_pose)],
    )

    second_sweep_end_pose = (9, 6.5, 2.0 * np.pi)
    qp.add_waypoint(
        second_sweep_end_pose,
        10,
        end_constraints=[PoseConstraint(second_sweep_end_pose)],
    )

    turnaround = (10.5, 6.5, 1.5 * np.pi)
    qp.add_waypoint(
        turnaround,
        10,
        end_constraints=[PoseConstraint(turnaround)],
    )

    bump_sweep_end = (10.5, 0.7, 1.5 * np.pi)
    qp.add_waypoint(
        bump_sweep_end,
        10,
        intermediate_constraints=[XConstraint(bump_sweep_end)],
        end_constraints=[PoseConstraint(bump_sweep_end)],
    )

    bump_sweep_end2 = (8, 0.7, 1.5 * np.pi)
    qp.add_waypoint(
        bump_sweep_end2,
        10,
        intermediate_constraints=[YConstraint(bump_sweep_end2)],
        end_constraints=[PoseConstraint(bump_sweep_end2)],
    )

    bump_return_pose = (6.5, 2.5, np.pi)
    qp.add_waypoint(
        bump_return_pose,
        10,
        end_constraints=[PoseConstraint(bump_return_pose)],
    )

    qp.add_waypoint(
        constants.SCORING_POSE,
        10,
        intermediate_constraints=[
            YConstraint(constants.SCORING_POSE),
            AngularConstraint(constants.SCORING_POSE),
        ],
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
