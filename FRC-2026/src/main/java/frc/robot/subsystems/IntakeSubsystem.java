// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private static final int kWristPositionAvgSlot = 0;
  private static final int kWristPositionDiffSlot = 1;

  private final QuixTalonFX m_leftWristMotor =
      new QuixTalonFX(
          Constants.Intake.leftWristMotorID,
          Constants.Intake.wristMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.leftWristMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(30.0)
              .setRotorBootOffset(Constants.Intake.leftWristRotorBootOffset)
              .setBootPositionOffset(Constants.Intake.startingAngle)
              .setReverseSoftLimit(Constants.Intake.minAngle)
              .setForwardSoftLimit(Constants.Intake.maxAngle)
              .setPIDConfig(kWristPositionAvgSlot, new PIDConfig(1.0, 0, 0, 0, 0.12, 0, 0))
              .setPIDConfig(kWristPositionDiffSlot, new PIDConfig(1.0, 0, 0, 0, 0.12, 0, 0)));

  private final QuixTalonFX m_rightWristMotor =
      new QuixTalonFX(
          Constants.Intake.rightWristMotorID,
          Constants.Intake.wristMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.rightWristMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(30.0)
              .setRotorBootOffset(Constants.Intake.rightWristRotorBootOffset)
              .setBootPositionOffset(Constants.Intake.startingAngle)
              .setReverseSoftLimit(Constants.Intake.minAngle)
              .setForwardSoftLimit(Constants.Intake.maxAngle)
              .setPIDConfig(kWristPositionAvgSlot, new PIDConfig(1.0, 0, 0, 0, 0.12, 0, 0)));

  //   private final QuixDifferentialMechanism m_differentialWrist =
  //       new QuixDifferentialMechanism(m_leftWristMotor, m_rightWristMotor);

  private final QuixTalonFX m_leftRollerMotor =
      new QuixTalonFX(
          Constants.Intake.leftRollerMotorID,
          Constants.Intake.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.leftRollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0));

  private final QuixTalonFX m_rightRollerMotor =
      new QuixTalonFX(
          Constants.Intake.rightRollerMotorID,
          m_leftRollerMotor,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.rightRollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0));

  private double m_targetAngle = Constants.Intake.startingAngle;
  private boolean m_isSlow = false;

  private IntakeSimulation m_intakeSim;

  public IntakeSubsystem(IntakeSimulation intakeSim) {
    m_intakeSim = intakeSim;
    m_isSlow = false;
  }

  public void setRotorTrims() {
    m_leftWristMotor.setRotorBootOffset(
        Constants.Intake.leftWristRotorBootOffset + Constants.RotorTrim.leftIntake.get());
    m_rightWristMotor.setRotorBootOffset(
        Constants.Intake.rightWristRotorBootOffset + Constants.RotorTrim.rightIntake.get());
  }

  public void setRollerPercentOutput(double percentOutput) {
    m_leftRollerMotor.setPercentOutput(percentOutput);
  }

  public void setAngle(double angleSetpoint, boolean isSlow) {
    m_targetAngle =
        MathUtils.clamp(angleSetpoint, Constants.Intake.minAngle, Constants.Intake.maxAngle);
    m_isSlow = isSlow;
  }

  public double getLeftWristAngle() {
    return m_leftWristMotor.getSensorPosition();
  }

  public double getRightWristAngle() {
    return m_rightWristMotor.getSensorPosition();
  }

  public boolean isAtAngle(double tolerance) {
    return Math.abs(getLeftWristAngle() - m_targetAngle) < tolerance
        && Math.abs(getRightWristAngle() - m_targetAngle) < tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LoggerHelper.recordCurrentCommand(this);

    // m_differentialWrist.setMotionMagicPositionSetpoint(
    //     kWristPositionAvgSlot, Math.toRadians(-10), kWristPositionDiffSlot, 0.0);
    m_leftWristMotor.setDynamicMotionMagicPositionSetpoint(
        kWristPositionAvgSlot,
        m_targetAngle,
        m_isSlow ? Constants.Intake.slowMaxWristVelocity : Constants.Intake.maxWristVelocity,
        Constants.Intake.maxWristAcceleration,
        Constants.Intake.maxWristJerk);
    m_rightWristMotor.setDynamicMotionMagicPositionSetpoint(
        kWristPositionAvgSlot,
        m_targetAngle,
        m_isSlow ? Constants.Intake.slowMaxWristVelocity : Constants.Intake.maxWristVelocity,
        Constants.Intake.maxWristAcceleration,
        Constants.Intake.maxWristJerk);

    Logger.recordOutput(
        "Intake/Current Left Wrist Angle (deg)",
        Units.radiansToDegrees(m_leftWristMotor.getSensorPosition()));
    Logger.recordOutput(
        "Intake/Target Left Wrist Angle (deg)",
        Units.radiansToDegrees(m_leftWristMotor.getClosedLoopReference()));
    Logger.recordOutput(
        "Intake/Current Right Wrist Angle (deg)",
        Units.radiansToDegrees(m_rightWristMotor.getSensorPosition()));
    Logger.recordOutput(
        "Intake/Target Right Wrist Angle (deg)",
        Units.radiansToDegrees(m_rightWristMotor.getClosedLoopReference()));

    Logger.recordOutput(
        "Intake/Current Left Roller Velocity (meters per sec)",
        m_leftRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Intake/Current Right Roller Velocity (meters per sec)",
        m_rightRollerMotor.getSensorVelocity());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_leftWristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44Foc(1),
          Constants.Intake.wristMotorRatio.reduction(),
          Constants.Intake.simWristMOI,
          Constants.Intake.simWristCGLength,
          Constants.Intake.minAngle,
          Constants.Intake.maxAngle,
          true, // Simulate gravity
          Constants.Intake.startingAngle);

  private static final SingleJointedArmSim m_rightWristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44Foc(1),
          Constants.Intake.wristMotorRatio.reduction(),
          Constants.Intake.simWristMOI,
          Constants.Intake.simWristCGLength,
          Constants.Intake.minAngle,
          Constants.Intake.maxAngle,
          true, // Simulate gravity
          Constants.Intake.startingAngle);

  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(2),
              Constants.Intake.simRollerMOI,
              Constants.Intake.rollerMotorRatio.reduction()),
          DCMotor.getKrakenX60Foc(2));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_leftWristSim.setInput(
        m_leftWristMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_leftWristSim.update(LoggedRobot.defaultPeriodSecs);
    m_leftWristMotor.setSimSensorPositionAndVelocity(
        m_leftWristSim.getAngleRads() - Constants.Intake.startingAngle,
        m_leftWristSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Intake.wristMotorRatio);

    m_rollerSim.setInput(
        m_leftRollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(LoggedRobot.defaultPeriodSecs);
    final double rollerMetersPerSecond =
        m_rollerSim.getAngularVelocityRadPerSec()
            * Constants.Intake.intakeWheelCircumfrence
            / (2.0 * Math.PI);
    m_leftRollerMotor.setSimSensorVelocity(
        rollerMetersPerSecond, LoggedRobot.defaultPeriodSecs, Constants.Intake.rollerMotorRatio);
    m_rightRollerMotor.setSimSensorVelocity(
        rollerMetersPerSecond, LoggedRobot.defaultPeriodSecs, Constants.Intake.rollerMotorRatio);

    m_rightWristSim.setInput(
        m_rightWristMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rightWristSim.update(LoggedRobot.defaultPeriodSecs);
    m_rightWristMotor.setSimSensorPositionAndVelocity(
        m_rightWristSim.getAngleRads() - Constants.Intake.startingAngle,
        m_rightWristSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Intake.wristMotorRatio);

    if (m_rollerSim.getAngularVelocityRadPerSec() > 0.0) {
      m_intakeSim.startIntake();
    } else {
      m_intakeSim.stopIntake();
    }
  }
}

// --- END STUFF FOR SIMULATION ---
