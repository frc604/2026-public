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
    start_pose = constants.GO_AFTER_PARTNER_END_POSE
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        start_action=Action(ActionType.DEPLOY_AND_ENABLE_SHOOT),
        apply_boundaries=False,
    )

    for _ in range(4):
        down_start_pose = (8.5, 6.5, -0.25 * np.pi)
        qp.add_waypoint(
            down_start_pose,
            20,
            end_constraints=[PoseConstraint(down_start_pose)],
        )

        down_end_pose = (8.5, 1.5, -0.25 * np.pi)
        qp.add_waypoint(
            down_end_pose,
            20,
            intermediate_constraints=[
                XConstraint(down_end_pose),
                AngularConstraint(down_end_pose),
            ],
            end_constraints=[PoseConstraint(down_end_pose)],
        )

        up_start_pose = (8.5, 1.5, 0.25 * np.pi)
        qp.add_waypoint(
            up_start_pose,
            20,
            end_constraints=[PoseConstraint(up_start_pose)],
        )

        up_end_pose = (8.5, 6.5, 0.25 * np.pi)
        qp.add_waypoint(
            up_end_pose,
            20,
            intermediate_constraints=[
                XConstraint(up_end_pose),
                AngularConstraint(up_end_pose),
            ],
            end_constraints=[PoseConstraint(up_end_pose)],
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
