import argparse
import os
import sys

import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    GoalConstraint,
    DistanceConstraint,
    QuikPlan,
    StoppedPoseConstraint,
    StoppedXYConstraint,
    XYConstraint,
    in2m,
)
from utils.helpers import write_to_csv


GOAL = (0.0, 0.0)  # m
SHOT_VELOCITY = 10.0  # m/s


def plan(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    start_pose = (4.0, 0.0, np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    waypoint1 = (0.0, 4.0, 1.5 * np.pi)
    qp.add_waypoint(
        waypoint1,
        10,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[XYConstraint(waypoint1)],
    )

    waypoint2 = (-4.0, 0.0, 2.0 * np.pi)
    qp.add_waypoint(
        waypoint2,
        10,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[XYConstraint(waypoint2)],
    )

    waypoint3 = (0.0, -4.0, 2.5 * np.pi)
    qp.add_waypoint(
        waypoint3,
        10,
        intermediate_constraints=[
            GoalConstraint(GOAL, SHOT_VELOCITY),
            DistanceConstraint(GOAL, 4.0),
        ],
        end_constraints=[XYConstraint(waypoint3)],
    )

    waypoint4 = (4.0, 0.0, 3.0 * np.pi)
    qp.add_waypoint(
        waypoint4,
        10,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(waypoint4)],
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
