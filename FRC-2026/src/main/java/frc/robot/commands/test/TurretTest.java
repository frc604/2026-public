// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTest extends Command {
  private final TurretSubsystem m_turret;
  private final double m_yawAngle;
  private final double m_launchAngle;

  public TurretTest(TurretSubsystem subsystem, double yawAngle, double launchAngle) {
    m_turret = subsystem;
    m_yawAngle = yawAngle;
    m_launchAngle = launchAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setTargetYaw(m_yawAngle, 0, 0);
    m_turret.setLaunchAngle(m_launchAngle);
    m_turret.setLaunchVelocity(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setTargetYaw(0, 0, 0);
    m_turret.setLaunchVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
