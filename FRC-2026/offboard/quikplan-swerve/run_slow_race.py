import argparse
import os
import sys

import numpy as np

import constants
from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    AngularConstraint,
    QuikPlan,
    SpeedConstraint,
    StoppedPoseConstraint,
    PoseConstraint,
    Action,
    ActionType,
    YConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=3500, MAX_TORQUE=0.7)

    # Configure the optimizer
    start_pose = (3.5, 2.5, 0)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    after_bump_pose = (5.5, 2.5, 0 * np.pi)
    qp.add_waypoint(
        after_bump_pose,
        20,
        intermediate_constraints=[
            YConstraint(after_bump_pose),
            AngularConstraint(after_bump_pose),
        ],
        end_constraints=[PoseConstraint(after_bump_pose)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    sweep_start_pose = (7.8, 1.25, 0.25 * np.pi)
    qp.add_waypoint(
        sweep_start_pose,
        20,
        intermediate_constraints=[SpeedConstraint(2.8)],
        end_constraints=[PoseConstraint(sweep_start_pose)],
    )

    sweep_turnaround_pose = (8.2, 3.6, np.pi)
    qp.add_waypoint(
        sweep_turnaround_pose,
        20,
        end_constraints=[PoseConstraint(sweep_turnaround_pose)],
    )

    bump_return_pose = (6.5, 2.5, np.pi)
    qp.add_waypoint(
        bump_return_pose,
        20,
        # intermediate_constraints=[SpeedConstraint(2.0)],
        end_constraints=[PoseConstraint(bump_return_pose)],
    )

    qp.add_waypoint(
        constants.SCORING_POSE,
        20,
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
