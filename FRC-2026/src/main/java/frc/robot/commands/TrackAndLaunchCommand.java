// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.localization.QuixSwerveLocalizer;
import frc.quixlib.math.LaunchCalculator;
import frc.robot.Constants;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrackAndLaunchCommand extends Command {
  private final TurretSubsystem m_turret;
  private final SerializerSubsystem m_serializer;
  private final DoubleConsumer m_setDriveSupplyCurrentLimit;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<Twist2d> m_robotVelocitySupplier;
  private final Supplier<Twist2d> m_robotAccelerationSupplier;
  private final Supplier<Twist2d> m_robotFieldVelocitySupplier;
  private final Supplier<Twist2d> m_robotFieldAccelerationSupplier;
  private final DoubleSupplier m_imuRollSupplier;
  private final DoubleSupplier m_imuPitchSupplier;
  private final LaunchCalculator m_launchCalculator;
  private final BooleanSupplier m_leftBumperSupplier;

  private Alliance m_alliance;

  // This is static to make it easier to enable/disable launch from other parts of the codebase.
  private static boolean m_launchEnabled = false;

  public TrackAndLaunchCommand(
      TurretSubsystem turret,
      SerializerSubsystem serializer,
      DoubleConsumer setDriveSupplyCurrentLimit,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Twist2d> robotVelocitySupplier,
      Supplier<Twist2d> robotAccelerationSupplier,
      Supplier<Twist2d> robotFieldVelocitySupplier,
      Supplier<Twist2d> robotFieldAccelerationSupplier,
      DoubleSupplier imuRollSupplier,
      DoubleSupplier imuPitchSupplier,
      LaunchCalculator launchCalculator,
      BooleanSupplier leftBumperSupplier) {
    m_turret = turret;
    m_serializer = serializer;
    m_setDriveSupplyCurrentLimit = setDriveSupplyCurrentLimit;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    m_robotAccelerationSupplier = robotAccelerationSupplier;
    m_robotFieldVelocitySupplier = robotFieldVelocitySupplier;
    m_robotFieldAccelerationSupplier = robotFieldAccelerationSupplier;
    m_imuRollSupplier = imuRollSupplier;
    m_imuPitchSupplier = imuPitchSupplier;
    m_launchCalculator = launchCalculator;
    m_leftBumperSupplier = leftBumperSupplier;
    addRequirements(turret, serializer);
  }

  public static void enableLaunch() {
    m_launchEnabled = true;
  }

  public static void disableLaunch() {
    m_launchEnabled = false;
  }

  @Override
  public void initialize() {
    // Set target position based on alliance color
    final var alliance = DriverStation.getAlliance();
    m_alliance =
        alliance.isPresent() && alliance.get() == Alliance.Blue ? Alliance.Blue : Alliance.Red;
    m_launchEnabled = false;
  }

  @Override
  public void execute() {
    final Pose2d robotPose = m_robotPoseSupplier.get();
    final Twist2d robotVelocity = m_robotVelocitySupplier.get();
    final Twist2d robotAcceleration = m_robotAccelerationSupplier.get();
    final Twist2d robotFieldVelocity = m_robotFieldVelocitySupplier.get();
    final Twist2d robotFieldAcceleration = m_robotFieldAccelerationSupplier.get();

    final boolean isFeed = getIsFeed(robotPose);
    final Pose2d targetPose = getTargetPose(isFeed, robotPose);

    final boolean useIMULaunch = m_leftBumperSupplier.getAsBoolean();
    final var launchInfo =
        LaunchCalculator.computeLaunchInfo(
            isFeed,
            robotPose,
            targetPose,
            m_launchCalculator,
            robotVelocity,
            robotAcceleration,
            robotFieldVelocity,
            robotFieldAcceleration,
            useIMULaunch ? -m_imuRollSupplier.getAsDouble() : 0.0,
            useIMULaunch ? -m_imuPitchSupplier.getAsDouble() : 0.0);

    final boolean hasRecentCameraUpdate = QuixSwerveLocalizer.hasRecentMultitagUpdate();

    if (m_launchEnabled) {
      m_setDriveSupplyCurrentLimit.accept(Constants.Swerve.reducedDriveSupplyLimit);
      m_turret.setTargetYaw(launchInfo.yaw, launchInfo.yawVelocity, launchInfo.yawAcceleration);
      m_turret.setLaunchAngle(launchInfo.elevation);
      m_turret.setLaunchVelocity(launchInfo.launchVelocity);
      setSerializerState(turretReady(isFeed), isFeed, hasRecentCameraUpdate, robotPose);
    } else {
      m_setDriveSupplyCurrentLimit.accept(Constants.Swerve.defaultDriveSupplyLimit);
      m_turret.setTargetYaw(Constants.Turret.startingYawAngle, 0, 0);
      m_turret.setHoodAngle(Constants.Turret.startingHoodAngle);
      m_turret.setLaunchVelocity(0.0);
      setSerializerState(false, isFeed, hasRecentCameraUpdate, robotPose);
    }
  }

  // ------- HELPER FUNCTIONS -------

  private boolean getIsFeed(Pose2d robotPose) {
    return !Constants.FieldZones.BLOCKED_ZONE.contains(robotPose)
        && (Constants.FieldZones.RIGHT_ZONE.contains(robotPose)
            || Constants.FieldZones.LEFT_ZONE.contains(robotPose));
  }

  // Decide whether to feed or not and fetch pose
  private Pose2d getTargetPose(boolean isFeed, Pose2d robotPose) {
    final Pose2d targetPose = isFeed ? getFeedingTargetPose(robotPose) : getScoringTargetPose();
    Logger.recordOutput("TrackAndLaunch/Target Pose", targetPose);
    Logger.recordOutput("TrackAndLaunch/Is Feed", isFeed);
    return targetPose;
  }

  // Determine feeding pose based on robot's position
  private Pose2d getFeedingTargetPose(Pose2d robotPose) {
    if (m_alliance == Alliance.Red) {
      if (Constants.FieldZones.RIGHT_ZONE.contains(robotPose)) {
        return Constants.Turret.redRightFeedPose;
      } else if (Constants.FieldZones.LEFT_ZONE.contains(robotPose)) {
        return Constants.Turret.redLeftFeedPose;
      }
    } else {
      if (Constants.FieldZones.LEFT_ZONE.contains(robotPose)) {
        return Constants.Turret.blueLeftFeedPose;
      } else if (Constants.FieldZones.RIGHT_ZONE.contains(robotPose)) {
        return Constants.Turret.blueRightFeedPose;
      }
    }

    // We don't actually want to score, but using the scoring target pose puts the
    // turret in the middle.
    return getScoringTargetPose();
  }

  private Pose2d getScoringTargetPose() {
    return m_alliance == Alliance.Blue
        ? Constants.Turret.blueHubCenterPose
        : Constants.Turret.redHubCenterPose;
  }

  private boolean turretReady(boolean isFeed) {
    final boolean atYaw =
        m_turret.isAtYaw(
            isFeed ? Constants.Turret.feedYawTolerance : Constants.Turret.yawTolerance);
    final boolean atAngle =
        m_turret.isAtAngle(
            isFeed ? Constants.Turret.feedAngleTolerance : Constants.Turret.launchAngleTolerance);
    final boolean atVelocity =
        m_turret.isAtVelocity(
            isFeed
                ? Constants.Turret.feedVelocityTolerance
                : Constants.Turret.launchVelocityTolerance);

    Logger.recordOutput("TrackAndLaunch/At Yaw", atYaw);
    Logger.recordOutput("TrackAndLaunch/At Angle", atAngle);
    Logger.recordOutput("TrackAndLaunch/At Velocity", atVelocity);

    return atYaw && atAngle && atVelocity;
  }

  private void setSerializerState(
      boolean turretReady, boolean isFeed, boolean hasRecentCameraUpdate, Pose2d robotPose) {
    final boolean inLaunchZone =
        !Constants.FieldZones.TOWER_ZONE.contains(robotPose)
            && !Constants.FieldZones.BUMP_ZONE.contains(robotPose)
            && !Constants.FieldZones.BLOCKED_ZONE.contains(robotPose);

    if (inLaunchZone && turretReady && hasRecentCameraUpdate) {
      m_serializer.setRollerVelocity(
          isFeed
              ? Constants.Serializer.feedingRollerVelocity.get()
              : Constants.Serializer.rollerVelocity.get());
    } else {
      m_serializer.setRollerVelocity(0.0);
    }

    Logger.recordOutput("TrackAndLaunch/Launch Enabled", m_launchEnabled);
    Logger.recordOutput("TrackAndLaunch/In Launch Zone", inLaunchZone);
    Logger.recordOutput("TrackAndLaunch/Has Recent Camera Update", hasRecentCameraUpdate);
  }

  @Override
  public void end(boolean interrupted) {
    m_setDriveSupplyCurrentLimit.accept(Constants.Swerve.defaultDriveSupplyLimit);
    m_turret.setLaunchVelocity(0.0);
    m_serializer.setRollerVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
