import argparse
import os
import sys

import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    AngularConstraint,
    GoalConstraint,
    PoseConstraint,
    QuikPlan,
    StoppedPoseConstraint,
    StoppedXYConstraint,
    XYConstraint,
    in2m,
)
from utils.helpers import write_to_csv


GOAL = (in2m(6.0), in2m(218.5))  # m
SHOT_VELOCITY = 15.0  # m/s


def plan(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    start_pose = (1.3, 5.55, np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    # get piece 2
    waypoint1 = (2.6, 5.55, np.pi)
    qp.add_waypoint(
        waypoint1,
        15,
        end_constraints=[
            StoppedXYConstraint(waypoint1),
            GoalConstraint(GOAL, SHOT_VELOCITY),
        ],
    )

    # approach piece 3
    waypoint2 = (2.1, 4.8, np.pi)
    qp.add_waypoint(
        waypoint2,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[XYConstraint(waypoint2)],
    )

    # get piece 3
    waypoint3 = (2.6, 4.3, np.pi)
    qp.add_waypoint(
        waypoint3,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(waypoint3)],
    )

    # approach note 3
    waypoint4 = (1.9, 5.8, np.pi)
    qp.add_waypoint(
        waypoint4,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[XYConstraint(waypoint4)],
    )

    # get piece 1
    waypoint5 = (2.9, 6.5, np.pi)
    qp.add_waypoint(
        waypoint5,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(waypoint5)],
    )

    # get 4
    waypoint6 = (8.2, 7.4, np.pi)
    qp.add_waypoint(
        waypoint6,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(waypoint6)],
    )

    # drive into wing
    wing_shot_waypoint = (3.0, 6.3, np.pi)
    qp.add_waypoint(
        wing_shot_waypoint,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(wing_shot_waypoint)],
    )

    # approach 5
    waypoint8a = (5.0, 6.4, np.pi)
    qp.add_waypoint(
        waypoint8a,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[XYConstraint(waypoint8a)],
    )

    waypoint8b = (7.0, 6.4, np.pi * 0.8)
    qp.add_waypoint(
        waypoint8b,
        15,
        end_constraints=[PoseConstraint(waypoint8b)],
    )

    # get 5
    waypoint8 = (8.2, 5.8, np.pi * 0.8)
    qp.add_waypoint(
        waypoint8,
        15,
        intermediate_constraints=[AngularConstraint(waypoint8)],
        end_constraints=[StoppedPoseConstraint(waypoint8)],
    )

    # leave 5
    qp.add_waypoint(
        waypoint8a,
        15,
        end_constraints=[PoseConstraint(waypoint8a)],
    )

    # drive into wing
    qp.add_waypoint(
        wing_shot_waypoint,
        15,
        end_constraints=[
            StoppedXYConstraint(wing_shot_waypoint),
            GoalConstraint(GOAL, SHOT_VELOCITY),
        ],
    )

    # go to centerline
    centerline_waypoint = (7.0, 6.4, np.pi)
    qp.add_waypoint(
        centerline_waypoint,
        15,
        intermediate_constraints=[GoalConstraint(GOAL, SHOT_VELOCITY)],
        end_constraints=[StoppedXYConstraint(centerline_waypoint)],
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
