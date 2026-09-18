// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.math.LaunchCalculator;
import frc.quixlib.math.LaunchParameter;
import frc.quixlib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

public class LaunchFuelHubTest extends Command {
  private final TurretSubsystem m_turret;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<Twist2d> m_robotVelocitySupplier;
  private final Supplier<Twist2d> m_robotAccelerationSupplier;
  private final LaunchCalculator m_launchCalculator;
  private final SerializerSubsystem m_serializer;

  private Pose2d targetPosition;
  private double robotRelativeYaw;
  private LaunchParameter launchParams;

  public LaunchFuelHubTest(
      TurretSubsystem turret,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Twist2d> robotVelocitySupplier,
      Supplier<Twist2d> robotAccelerationSupplier,
      LaunchCalculator launchCalculator,
      SerializerSubsystem serializer) {
    m_turret = turret;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    m_robotAccelerationSupplier = robotAccelerationSupplier;
    m_launchCalculator = launchCalculator;
    m_serializer = serializer;
    addRequirements(turret, serializer);
  }

  @Override
  public void initialize() {
    // Set target position based on alliance color
    final var alliance = DriverStation.getAlliance();
    targetPosition =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? Constants.Turret.blueHubCenterPose
            : Constants.Turret.redHubCenterPose;
  }

  @Override
  public void execute() {
    final Pose2d robotPose = m_robotPoseSupplier.get();
    // Transform robot pose to turret base pose
    final Pose2d turretPose = robotPose.transformBy(Constants.Turret.robotToTurretBaseT2d);
    final Twist2d robotVelocity = m_robotVelocitySupplier.get();
    final Twist2d robotAcceleration = m_robotAccelerationSupplier.get();

    // Calculate distance from turret to target
    final Translation2d robotToTarget =
        targetPosition.getTranslation().minus(turretPose.getTranslation());
    final double distanceToTarget = robotToTarget.getNorm();

    // Get optimal launch parameters from calculator
    launchParams = m_launchCalculator.interpolateLaunchEntry(distanceToTarget);

    final Twist2d turretVelocity =
        MathUtils.getVelocityAtOffset(
            robotPose, Constants.Turret.robotToTurretBaseT2d, robotVelocity);

    // Convert field-relative params to robot-relative params.
    // TODO: Move to a helper and add tests.
    final double v0 = launchParams.launchVelocityMps();
    final double phi0 = Math.toRadians(launchParams.launchAngleDeg());
    final double theta0 = Math.atan2(robotToTarget.getY(), robotToTarget.getX());

    // Desired field-relative launch velocity components
    final double vx_target = v0 * Math.cos(phi0) * Math.cos(theta0);
    final double vy_target = v0 * Math.cos(phi0) * Math.sin(theta0);
    final double vz_target = v0 * Math.sin(phi0);

    // Required robot-relative launch velocity components
    final double vx_rel = vx_target - turretVelocity.dx;
    final double vy_rel = vy_target - turretVelocity.dy;
    final double vz_rel = vz_target;

    // Convert back to robot-relative launch parameters
    final double v_rel = Math.sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
    final double phi_rel = Math.asin(vz_rel / v_rel);
    final double theta_rel = Math.atan2(vy_rel, vx_rel);

    robotRelativeYaw = theta_rel - turretPose.getRotation().getRadians();
    final Twist2d turretAcceleration =
        MathUtils.getAccelAtOffset(
            robotPose, Constants.Turret.robotToTurretBaseT2d, robotVelocity, robotAcceleration);
    final double targetYawVelocity =
        LaunchCalculator.getTargetYawVelocity(
            turretPose, turretVelocity, targetPosition.getTranslation());
    final double targetYawAcceleration =
        LaunchCalculator.getTargetYawAcceleration(
            turretPose, turretVelocity, turretAcceleration, targetPosition.getTranslation());

    // Update turret with calculated values
    m_turret.setTargetYaw(robotRelativeYaw, targetYawVelocity, targetYawAcceleration);
    m_turret.setLaunchAngle(phi_rel);
    m_turret.setLaunchVelocity(v_rel);

    if (m_turret.isAtAngle(Constants.Turret.launchAngleTolerance)
        && m_turret.isAtYaw(Constants.Turret.yawTolerance)
        && m_turret.isAtVelocity(Constants.Turret.launchVelocityTolerance)) {
      m_serializer.setRollerVelocity(Constants.Serializer.rollerVelocity.get());
    } else {
      m_serializer.setRollerVelocity(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setLaunchVelocity(0);
    m_serializer.setRollerVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
