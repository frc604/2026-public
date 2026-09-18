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
    robot = Robot(MAX_RPM=2500, MAX_TORQUE=0.7)

    # Configure the optimizer
    scoring_pose = (2.4, Field.WIDTH - 2.5, -np.pi + 2.0 * np.pi)
    qp = QuikPlan(
        field,
        robot,
        scoring_pose,
        [StoppedPoseConstraint(scoring_pose)],
        start_action=Action(ActionType.DEPLOY_AND_ENABLE_SHOOT),
        apply_boundaries=False,
    )

    pre_depot_pose = (1.6, 5.0, -1.25 * np.pi + 2.0 * np.pi)
    qp.add_waypoint(
        pre_depot_pose,
        10,
        end_constraints=[PoseConstraint(pre_depot_pose)],
    )

    depot_start_pose = (0.8, 5.0, -1.25 * np.pi + 2.0 * np.pi)
    qp.add_waypoint(
        depot_start_pose,
        10,
        intermediate_constraints=[YConstraint(depot_start_pose)],
        end_constraints=[PoseConstraint(depot_start_pose)],
    )

    depot_end_pose = (0.8, 6.7, -1.25 * np.pi + 2.0 * np.pi)
    qp.add_waypoint(
        depot_end_pose,
        10,
        intermediate_constraints=[XConstraint(depot_end_pose), SpeedConstraint(1.0)],
        end_constraints=[PoseConstraint(depot_end_pose)],
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
