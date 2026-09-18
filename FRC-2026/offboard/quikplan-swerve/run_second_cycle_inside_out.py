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
    robot = Robot(MAX_RPM=3500, MAX_TORQUE=0.6)

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        constants.SCORING_POSE,
        [StoppedPoseConstraint(constants.SCORING_POSE)],
        start_action=Action(ActionType.DISABLE_SHOOT),
        apply_boundaries=False,
    )

    pre_bump_pose = (3.5, 2.5, 0.5 * np.pi)
    qp.add_waypoint(
        pre_bump_pose,
        15,
        end_constraints=[PoseConstraint(pre_bump_pose)],
    )

    after_bump_pose = (5.75, 2.5, 0.5 * np.pi)
    qp.add_waypoint(
        after_bump_pose,
        15,
        intermediate_constraints=[
            YConstraint(after_bump_pose),
            AngularConstraint(after_bump_pose),
        ],
        end_constraints=[PoseConstraint(after_bump_pose)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    sweep_turnaround_pose = (6, 4, 0.25 * np.pi)
    qp.add_waypoint(
        sweep_turnaround_pose,
        15,
        end_constraints=[PoseConstraint(sweep_turnaround_pose)],
    )

    sweep_turnaround_poseB = (8, 4, -0.25 * np.pi)
    qp.add_waypoint(
        sweep_turnaround_poseB,
        15,
        end_constraints=[PoseConstraint(sweep_turnaround_poseB)],
    )

    sweep_end_pose = (8, 2.5, -np.pi)
    qp.add_waypoint(
        sweep_end_pose,
        15,
        end_constraints=[PoseConstraint(sweep_end_pose)],
    )

    bump_return_pose = (6.5, 2.5, -np.pi)
    qp.add_waypoint(
        bump_return_pose,
        15,
        end_constraints=[PoseConstraint(bump_return_pose)],
    )

    scoring_pose = (3.0, 2.5, -np.pi)
    qp.add_waypoint(
        scoring_pose,
        15,
        intermediate_constraints=[
            YConstraint(scoring_pose),
            AngularConstraint(scoring_pose),
        ],
        end_constraints=[StoppedPoseConstraint(scoring_pose)],
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
