import argparse
import sys

import matplotlib.pyplot as plt
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
    StoppedXYConstraint,
    XConstraint,
    YConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot(MAX_RPM=2500, MAX_TORQUE=0.5)

    # Configure the optimizer
    start_pose = (3.5, 2.5, 0)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )

    after_bump_pose = (5.5, 2.5, 0)
    qp.add_waypoint(
        after_bump_pose,
        10,
        intermediate_constraints=[
            YConstraint(after_bump_pose),
            AngularConstraint(after_bump_pose),
        ],
        end_constraints=[PoseConstraint(after_bump_pose)],
    )

    # enter_sweep_pose = (6.5, 3, 0.25 * np.pi)
    # qp.add_waypoint(
    #     enter_sweep_pose,
    #     20,
    #     end_constraints=[PoseConstraint(enter_sweep_pose)],
    #     end_action=Action(ActionType.DEPLOY_AND_ENABLE_SHOOT),
    # )

    # # Need to use negative pi because the robot will try to turn the other way if pi is positive
    # sweep_1_pose = (8, 3.5, -0.25 * np.pi)
    # qp.add_waypoint(
    #     sweep_1_pose,
    #     20,
    #     end_constraints=[PoseConstraint(sweep_1_pose)],
    # )

    # sweep_2_pose = (9, 2.5, -0.75 * np.pi)
    # qp.add_waypoint(
    #     sweep_2_pose,
    #     20,
    #     end_constraints=[PoseConstraint(sweep_2_pose)],
    # )

    # sweep_3_pose = (8, 2, -0.9 * np.pi)
    # qp.add_waypoint(
    #     sweep_3_pose,
    #     20,
    #     end_constraints=[PoseConstraint(sweep_3_pose)],
    # )

    # exit_sweep_pose = (7.3, 2, -np.pi)
    # qp.add_waypoint(
    #     exit_sweep_pose,
    #     20,
    #     end_constraints=[PoseConstraint(exit_sweep_pose)],
    #     end_action=Action(ActionType.RETRACT_INTAKE),
    # )

    enter_sweep_pose = (6.5, 2.3, 0)
    qp.add_waypoint(
        enter_sweep_pose,
        10,
        end_constraints=[PoseConstraint(enter_sweep_pose)],
        end_action=Action(ActionType.DEPLOY_AND_ENABLE_SHOOT),
    )

    sweep_1_pose = (8.5, 2.3, 0)
    qp.add_waypoint(
        sweep_1_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_1_pose)],
    )

    sweep_2_pose = (9, 2.5, np.pi / 2)
    qp.add_waypoint(
        sweep_2_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_2_pose)],
    )

    sweep_3_pose = (9, 3.5, 0.6 * np.pi)
    qp.add_waypoint(
        sweep_3_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_3_pose)],
    )

    sweep_4_pose = (9, 4, 0.75 * np.pi)
    qp.add_waypoint(
        sweep_4_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_4_pose)],
    )
    sweep_5_pose = (7.5, 4, 1.2 * np.pi)
    qp.add_waypoint(
        sweep_5_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_5_pose)],
    )

    sweep_6_pose = (7.5, 3.3, np.pi * 1.5)
    qp.add_waypoint(
        sweep_6_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(sweep_6_pose)],
    )

    exit_sweep_pose = (7.5, 2.8, np.pi * 1.25)
    qp.add_waypoint(
        exit_sweep_pose,
        10,
        intermediate_constraints=[SpeedConstraint(1.6)],
        end_constraints=[PoseConstraint(exit_sweep_pose)],
    )

    bump_return_pose = (5.5, 2.5, np.pi)
    qp.add_waypoint(
        bump_return_pose,
        10,
        intermediate_constraints=[SpeedConstraint(2.0)],
        end_constraints=[PoseConstraint(bump_return_pose)],
    )

    enter_alliance_pose = (3.5, 2.5, np.pi)
    qp.add_waypoint(
        enter_alliance_pose,
        10,
        intermediate_constraints=[
            YConstraint(enter_alliance_pose),
            AngularConstraint(enter_alliance_pose),
        ],
        end_constraints=[PoseConstraint(enter_alliance_pose)],
    )

    start_cleanup_pose = (2.5, 0.5, np.pi)
    qp.add_waypoint(
        start_cleanup_pose,
        10,
        end_constraints=[PoseConstraint(start_cleanup_pose)],
    )

    corner_pose = (0.6, 0.5, np.pi)
    qp.add_waypoint(
        corner_pose,
        10,
        intermediate_constraints=[YConstraint(corner_pose)],
        end_constraints=[StoppedPoseConstraint(corner_pose)],
    )

    # stop_cleanup_pose = (0.6, 2.5, np.pi / 2)
    # qp.add_waypoint(
    #     stop_cleanup_pose,
    #     20,
    #     intermediate_constraints = [XConstraint(stop_cleanup_pose)],
    #     end_constraints=[StoppedPoseConstraint(stop_cleanup_pose)],
    # )

    # end_pose = (1.5, 2.5, 0)
    # qp.add_waypoint(
    #     end_pose,
    #     20,
    #     end_constraints=[StoppedPoseConstraint(end_pose)],
    #     end_action=Action(ActionType.RETRACT_AND_DISABLE_SHOOT),
    # )

    # Plan the trajectory
    traj = qp.plan(quiet)
    write_to_csv(traj, "feed_cleanup")

    # Plot
    field.plot_traj(robot, traj, "feed_cleanup.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "feed_cleanup.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
