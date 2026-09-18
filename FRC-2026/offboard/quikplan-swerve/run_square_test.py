import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import QuikPlan, StoppedPoseConstraint
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    start_pose = (0, 0, 0)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    waypoint2 = (2.0, 0.0, 0.5 * np.pi)
    qp.add_waypoint(waypoint2, 10, end_constraints=[StoppedPoseConstraint(waypoint2)])

    waypoint3 = (2.0, 2.0, np.pi)
    qp.add_waypoint(waypoint3, 10, end_constraints=[StoppedPoseConstraint(waypoint3)])

    waypoint4 = (0.0, 2.0, 1.5 * np.pi)
    qp.add_waypoint(waypoint4, 10, end_constraints=[StoppedPoseConstraint(waypoint4)])

    waypoint5 = (0.0, 0.0, 2.0 * np.pi)
    qp.add_waypoint(waypoint5, 10, end_constraints=[StoppedPoseConstraint(waypoint5)])

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
