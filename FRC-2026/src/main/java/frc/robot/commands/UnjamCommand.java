// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class UnjamCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final SerializerSubsystem m_serializer;

  public UnjamCommand(IntakeSubsystem intake, SerializerSubsystem serializer) {
    m_intake = intake;
    m_serializer = serializer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setAngle(Constants.Intake.deployAngle, false);
    m_intake.setRollerPercentOutput(Constants.Intake.rollerUnjamPercentOutput);

    m_serializer.setRollerVelocity(Constants.Serializer.rollerUnjamVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerPercentOutput(0.0);

    m_serializer.setRollerVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
