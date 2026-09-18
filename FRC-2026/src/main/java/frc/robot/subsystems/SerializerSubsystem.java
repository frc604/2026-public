// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class SerializerSubsystem extends SubsystemBase {
  private final int kVelocitySlot = 0;
  private final QuixTalonFX m_rotorMotor =
      new QuixTalonFX(
          Constants.Serializer.rotorMotorID,
          Constants.Serializer.rotorMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Serializer.rotorMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(kVelocitySlot, new PIDConfig(0.1, 0, 0, 0, 0.125, 0, 0)));
  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Serializer.rollerMotorID,
          Constants.Serializer.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Serializer.rollerMotorInvert)
              .setSupplyCurrentLimit(60.0)
              .setStatorCurrentLimit(100.0)
              .setPIDConfig(kVelocitySlot, new PIDConfig(0.1, 0, 0, 0, 0.125, 0, 0)));

  public enum JamState {
    NORMAL,
    JAMMED
  }

  private double m_targetRotorVelocity = 0.0;
  private double m_targetRollerVelocity = 0.0;
  private boolean m_rollerActive = false;
  private final CircularBuffer<Double> m_rollerVelocityBuffer = new CircularBuffer<>(10);

  private JamState m_jamState = JamState.NORMAL;
  private final Timer m_stateTimer = new Timer();

  public SerializerSubsystem() {}

  public void setRollerVelocity(double linearBallVelocity) {
    // Convert linear ball velocity (m/s) into a rotary velocity (rad/s).
    // Roller is only on one side of the ball, travels half the roller circumference per rotation.
    m_targetRollerVelocity = linearBallVelocity / (0.5 * Constants.Serializer.rollerWheelRadius);
    m_rollerActive = m_targetRollerVelocity != 0;

    if (m_jamState == JamState.JAMMED) {
      return;
    }

    if (!m_rollerActive) {
      m_rollerMotor.setPercentOutput(0.0);
      m_rollerVelocityBuffer.clear();
      m_targetRotorVelocity = 0.0;
    } else {
      m_rollerMotor.setVelocitySetpoint(kVelocitySlot, m_targetRollerVelocity);
    }
  }

  public double getRotorAngle() {
    return m_rotorMotor.getSensorPosition();
  }

  public double getSecondsPerBall() {
    // Approximate seconds per ball at the current rotor velocity.
    final double rotorVelocity = m_rotorMotor.getSensorVelocity();
    if (rotorVelocity == 0.0) {
      return Double.POSITIVE_INFINITY;
    }

    final double kBallDiameter = 0.15; // m
    return kBallDiameter / (Constants.Serializer.effectiveRotorRadius * rotorVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LoggerHelper.recordCurrentCommand(this);

    // Update Jam State
    final boolean rotorJammed =
        Math.abs(m_rotorMotor.getStatorCurrent()) > 40.0 && m_rotorMotor.getSensorVelocity() < 2.0;
    final boolean rollerJammed =
        Math.abs(m_rollerMotor.getStatorCurrent()) > 80.0
            && m_rollerMotor.getSensorVelocity() < 0.1 * m_targetRollerVelocity;
    if (m_jamState == JamState.NORMAL) {
      if (m_rollerActive && (rotorJammed || rollerJammed)) {
        m_stateTimer.start();
        if (m_stateTimer.hasElapsed(0.3)) {
          m_jamState = JamState.JAMMED;
          m_stateTimer.restart();
          // Apply jammed state immediately
          m_rollerMotor.setVelocitySetpoint(
              kVelocitySlot, Constants.Serializer.rollerUnjamVelocity);
          m_rotorMotor.setPercentOutput(0.0);
          m_targetRotorVelocity = 0.0;
        }
      } else {
        m_stateTimer.stop();
        m_stateTimer.reset();
      }
    } else if (m_jamState == JamState.JAMMED) {
      if (m_stateTimer.hasElapsed(0.1)) {
        m_jamState = JamState.NORMAL;
        m_stateTimer.stop();
        m_stateTimer.reset();

        // Re-apply target roller velocity since we ignored commands while jammed
        if (!m_rollerActive) {
          m_rollerMotor.setPercentOutput(0.0);
          m_rollerVelocityBuffer.clear();
          m_targetRotorVelocity = 0.0;
        } else {
          m_rollerMotor.setVelocitySetpoint(kVelocitySlot, m_targetRollerVelocity);
        }
      }
    }

    Logger.recordOutput("Serializer/JamState", m_jamState);

    // Keep normal logic, but ONLY if NORMAL
    if (m_jamState == JamState.NORMAL) {
      // Set rotor velocity based on measured roller velocity (rolling average over 10 samples)
      // such that roller linear velocity = 1.2 * rotor linear velocity.
      if (m_rollerActive) {
        final double measuredRollerLinearVelocity =
            m_rollerMotor.getSensorVelocity() * (0.5 * Constants.Serializer.rollerWheelRadius);
        m_rollerVelocityBuffer.addLast(measuredRollerLinearVelocity);

        double sum = 0.0;
        for (int i = 0; i < m_rollerVelocityBuffer.size(); i++) {
          sum += m_rollerVelocityBuffer.get(i);
        }
        final double avgRollerLinearVelocity = sum / m_rollerVelocityBuffer.size();

        final double targetRotorLinearVelocity = avgRollerLinearVelocity / 1.2;
        m_targetRotorVelocity =
            Math.max(0.0, targetRotorLinearVelocity / Constants.Serializer.effectiveRotorRadius);
        m_rotorMotor.setVelocitySetpoint(kVelocitySlot, m_targetRotorVelocity);
      } else {
        m_rotorMotor.setPercentOutput(0.0);
      }
    }

    Logger.recordOutput(
        "Serializer/Current Rotor Velocity (rad per sec)", m_rotorMotor.getSensorVelocity());
    Logger.recordOutput("Serializer/Target Rotor Velocity (rad per sec)", m_targetRotorVelocity);

    Logger.recordOutput(
        "Serializer/Current Roller Velocity (rad per sec)", m_rollerMotor.getSensorVelocity());
    Logger.recordOutput("Serializer/Target Roller Velocity (rad per sec)", m_targetRollerVelocity);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final FlywheelSim m_rotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              Constants.Serializer.simRotorMOI,
              Constants.Serializer.rotorMotorRatio.reduction()),
          DCMotor.getKrakenX60Foc(1));

  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              Constants.Serializer.simRollerMOI,
              Constants.Serializer.rollerMotorRatio.reduction()),
          DCMotor.getKrakenX60Foc(1));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_rotorSim.setInput(m_rotorMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rotorSim.update(LoggedRobot.defaultPeriodSecs);
    m_rotorMotor.setSimSensorVelocity(
        m_rotorSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Serializer.rotorMotorRatio);

    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(LoggedRobot.defaultPeriodSecs);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Serializer.rollerMotorRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
