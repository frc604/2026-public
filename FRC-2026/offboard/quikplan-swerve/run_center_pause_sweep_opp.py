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
    robot = Robot(MAX_RPM=4000, MAX_TORQUE=0.7)

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        constants.CENTER_PAUSE_POSE,
        [StoppedPoseConstraint(constants.CENTER_PAUSE_POSE)],
        start_action=Action(ActionType.DEPLOY_INTAKE),
        apply_boundaries=False,
    )

    # Feed on
    sweep_start_pose = (9.5, 4.5, 0.5 * np.pi)
    qp.add_waypoint(
        sweep_start_pose,
        10,
        end_constraints=[PoseConstraint(sweep_start_pose)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    sweep_turn_pose_A = (9.5, 6.2, 0.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_A,
        10,
        intermediate_constraints=[
            XConstraint(sweep_turn_pose_A),
            AngularConstraint(sweep_turn_pose_A),
        ],
        end_constraints=[PoseConstraint(sweep_turn_pose_A)],
    )

    # Sweep along opponent's bump
    sweep_turn_pose_B = (10.6, 6.2, -0.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_B, 10, end_constraints=[PoseConstraint(sweep_turn_pose_B)]
    )

    sweep_turn_pose_C = (10.6, 1.8, -0.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_C,
        10,
        intermediate_constraints=[
            XConstraint(sweep_turn_pose_C),
            AngularConstraint(sweep_turn_pose_C),
        ],
        end_constraints=[PoseConstraint(sweep_turn_pose_C)],
    )

    # Double back
    # Feed off
    sweep_turn_pose_D = (9.5, 1.8, -1.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_D,
        10,
        end_constraints=[PoseConstraint(sweep_turn_pose_D)],
    )

    sweep_turn_pose_E = (9.5, 5.8, -1.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_E,
        10,
        intermediate_constraints=[
            XConstraint(sweep_turn_pose_E),
            AngularConstraint(sweep_turn_pose_E),
        ],
        end_constraints=[PoseConstraint(sweep_turn_pose_E)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # Centerline sweep
    sweep_turn_pose_F = (8.5, 5.8, -0.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_F, 10, end_constraints=[PoseConstraint(sweep_turn_pose_F)]
    )

    sweep_turn_pose_G = (8.5, 2.1, -0.5 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_G,
        10,
        intermediate_constraints=[
            XConstraint(sweep_turn_pose_G),
            AngularConstraint(sweep_turn_pose_G),
        ],
        end_constraints=[PoseConstraint(sweep_turn_pose_G)],
    )

    sweep_turn_pose_H = (7.5, 2.1, -1.2 * np.pi)
    qp.add_waypoint(
        sweep_turn_pose_H,
        10,
        end_constraints=[PoseConstraint(sweep_turn_pose_H)],
    )

    sweep_turn_pose_I = (6.5, 2.5, -np.pi)
    qp.add_waypoint(
        sweep_turn_pose_I,
        10,
        end_constraints=[PoseConstraint(sweep_turn_pose_I)],
    )

    bump_return_pose = (6.0, 2.5, -np.pi)
    qp.add_waypoint(
        bump_return_pose,
        10,
        end_constraints=[PoseConstraint(bump_return_pose)],
    )

    scoring_pose = (2.4, 2.5, -np.pi)
    qp.add_waypoint(
        scoring_pose,
        20,
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
