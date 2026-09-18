// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
  private final IntakeSubsystem m_intake;
  private final XboxController m_xbox;

  public DeployIntake(IntakeSubsystem intakeSubsystem, XboxController xbox) {
    m_intake = intakeSubsystem;
    m_xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setAngle(Constants.Intake.deployAngle, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_xbox.getRightTriggerAxis() > 0.2) {
      m_intake.setAngle(Constants.Intake.deployAngle, false);
      if (m_intake.isAtAngle(Constants.Intake.wristTolerance)) {
        m_intake.setRollerPercentOutput(Constants.Intake.rollerPercentOutput);
      }
    } else {
      m_intake.setAngle(Constants.Intake.standbyAngle, false);
      m_intake.setRollerPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
