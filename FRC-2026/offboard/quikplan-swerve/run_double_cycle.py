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
    StoppedXYConstraint,
    YConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=2500, MAX_TORQUE=0.5)

    # Configure the optimizer
    # TODO change placeholder start position
    start_pose = (3.5, 2.5, np.pi / 4)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        start_action=Action(ActionType.ENABLE_SHOOT),
        apply_boundaries=False,
    )

    neutral_side_bump_pose = (6, 2.5, np.pi / 4)
    qp.add_waypoint(
        neutral_side_bump_pose,
        10,
        intermediate_constraints=[
            YConstraint(neutral_side_bump_pose),
            AngularConstraint(neutral_side_bump_pose),
        ],
        end_constraints=[PoseConstraint(neutral_side_bump_pose)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    intake_pose_1 = (7, 2.2, np.pi / 4)
    qp.add_waypoint(
        intake_pose_1,
        10,
        end_constraints=[PoseConstraint(intake_pose_1)],
    )

    intake_pose_2 = (7.5, 3.5, 7 * np.pi / 12)
    qp.add_waypoint(
        intake_pose_2,
        10,
        end_constraints=[PoseConstraint(intake_pose_2)],
    )

    intake_pose_3 = (7.5, 5, 7 * np.pi / 12)
    qp.add_waypoint(
        intake_pose_3,
        10,
        end_constraints=[PoseConstraint(intake_pose_3)],
        end_action=Action(ActionType.STOP_ROLLERS),
    )

    enter_bump_to_alliance_pose = (6.5, 2.5, 3 * np.pi / 4)
    qp.add_waypoint(
        enter_bump_to_alliance_pose,
        10,
        end_constraints=[PoseConstraint(enter_bump_to_alliance_pose)],
    )

    leave_bump_on_alliance_pose = (3.5, 2.5, 3 * np.pi / 4)
    qp.add_waypoint(
        leave_bump_on_alliance_pose,
        10,
        intermediate_constraints=[
            AngularConstraint(leave_bump_on_alliance_pose),
            YConstraint(leave_bump_on_alliance_pose),
        ],
        end_constraints=[PoseConstraint(leave_bump_on_alliance_pose)],
    )

    shoot_pose = (2.8, 2.5, 0)
    qp.add_waypoint(
        shoot_pose,
        10,
        end_constraints=[StoppedXYConstraint(shoot_pose)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # # # Second cycle

    second_sweep_alliance_bump_pose = (3.5, 2.5, 3 * np.pi / 4)
    qp.add_waypoint(
        second_sweep_alliance_bump_pose,
        10,
        end_constraints=[PoseConstraint(second_sweep_alliance_bump_pose)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    second_sweep_neutral_bump_pose = (6, 2.5, 3 * np.pi / 4)
    qp.add_waypoint(
        second_sweep_neutral_bump_pose,
        10,
        intermediate_constraints=[
            YConstraint(second_sweep_neutral_bump_pose),
            AngularConstraint(second_sweep_neutral_bump_pose),
        ],
        end_constraints=[PoseConstraint(second_sweep_neutral_bump_pose)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    qp.add_waypoint(
        intake_pose_1,
        10,
        end_constraints=[PoseConstraint(intake_pose_1)],
    )

    qp.add_waypoint(
        intake_pose_2,
        10,
        end_constraints=[PoseConstraint(intake_pose_2)],
    )

    qp.add_waypoint(
        intake_pose_3,
        10,
        end_constraints=[PoseConstraint(intake_pose_3)],
        end_action=Action(ActionType.STOP_ROLLERS),
    )

    qp.add_waypoint(
        enter_bump_to_alliance_pose,
        10,
        end_constraints=[PoseConstraint(enter_bump_to_alliance_pose)],
    )

    qp.add_waypoint(
        leave_bump_on_alliance_pose,
        10,
        intermediate_constraints=[
            AngularConstraint(leave_bump_on_alliance_pose),
            YConstraint(leave_bump_on_alliance_pose),
        ],
        end_constraints=[PoseConstraint(leave_bump_on_alliance_pose)],
    )

    qp.add_waypoint(
        shoot_pose,
        10,
        end_constraints=[StoppedXYConstraint(shoot_pose)],
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
