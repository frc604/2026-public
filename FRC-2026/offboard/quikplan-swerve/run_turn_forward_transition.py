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
    XYConstraint,
    SpeedConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=3500, MAX_TORQUE=0.5)

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        constants.SCORING_POSE,
        [StoppedPoseConstraint(constants.SCORING_POSE)],
        start_action=Action(ActionType.ENABLE_SHOOT),
        apply_boundaries=False,
    )

    qp.add_waypoint(
        constants.SECOND_SWEEP_90_POSE,
        10,
        intermediate_constraints=[
            XYConstraint(constants.SECOND_SWEEP_90_POSE),
            SpeedConstraint(0.21),
        ],
        end_constraints=[PoseConstraint(constants.SECOND_SWEEP_90_POSE)],
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
