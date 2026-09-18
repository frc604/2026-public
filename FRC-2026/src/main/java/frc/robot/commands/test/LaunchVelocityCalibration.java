// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.math.LaunchCalculator;
import frc.robot.Constants;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LaunchVelocityCalibration extends Command {
  private final SerializerSubsystem m_serializer;
  private final TurretSubsystem m_turret;
  private final LaunchCalculator m_launchCalculator;

  public static final LoggedNetworkNumber m_launchDistanceInches =
      new LoggedNetworkNumber("Launch Calibration/Launch Distance (inches)", 120);
  public static final LoggedNetworkNumber m_launchVelocity =
      new LoggedNetworkNumber("Launch Calibration/Launch Velocity (rad per sec)", 150);

  public LaunchVelocityCalibration(
      SerializerSubsystem serializer, TurretSubsystem turret, LaunchCalculator launchCalculator) {
    m_serializer = serializer;
    m_turret = turret;
    m_launchCalculator = launchCalculator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(serializer, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double distanceToTarget = Units.inchesToMeters(m_launchDistanceInches.get());

    // Get optimal launch parameters from calculator
    final var launchParams = m_launchCalculator.interpolateLaunchEntry(distanceToTarget);

    m_turret.setTargetYaw(0, 0, 0);
    m_turret.setLaunchAngle(Math.toRadians(launchParams.launchAngleDeg()));
    m_turret.setRawLaunchVelocity(m_launchVelocity.get());
    Logger.recordOutput(
        "Launch Calibration/Desired Velocity (meters per sec)", launchParams.launchVelocityMps());

    if (m_turret.isAtAngle(Constants.Turret.launchAngleTolerance)
        && m_turret.isAtVelocity(Constants.Turret.launchVelocityTolerance)) {
      m_serializer.setRollerVelocity(Constants.Serializer.rollerVelocity.get());
    } else {
      m_serializer.setRollerVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setLaunchVelocity(0);
    m_serializer.setRollerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
