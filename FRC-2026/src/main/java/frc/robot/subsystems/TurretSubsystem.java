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
import frc.quixlib.math.LaunchCalculator;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final int kPositionSlot = 0;
  private final int kVelocitySlot = 0;

  private double m_targetYaw = Constants.Turret.startingYawAngle;
  private double m_targetYawVelocity = 0;
  // Hysteresis state: true when the goal is clamped at the respective yaw limit.
  private boolean m_clampedAtMaxYaw = false;
  private boolean m_clampedAtMinYaw = false;
  private double m_targetYawAcceleration = 0;
  private double m_targetHoodAngle = Constants.Turret.startingHoodAngle;
  private double m_targetLaunchVel = 0.0;

  private final QuixTalonFX m_yawMotor =
      new QuixTalonFX(
          Constants.Turret.yawMotorID,
          Constants.Turret.yawMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Turret.yawMotorInvert)
              .setBrakeMode()
              .setRotorBootOffset(Constants.Turret.yawMotorBootOffset)
              .setBootPositionOffset(Constants.Turret.startingYawAngle)
              .setReverseSoftLimit(Constants.Turret.minYawAngle)
              .setForwardSoftLimit(Constants.Turret.maxYawAngle)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0)
              .setPIDConfig(
                  kPositionSlot,
                  Robot.isSimulation()
                      ? new PIDConfig(2, 0, 0, 0, 0.13, 0, 0)
                      : new PIDConfig(10, 0, 0, 0, 0.13, 0, 0))
              .setMotionMagicConfig(10.0, 50, 0));

  private final QuixTalonFX m_hoodMotor =
      new QuixTalonFX(
          Constants.Turret.hoodMotorID,
          Constants.Turret.hoodMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Turret.hoodMotorInvert)
              .setBrakeMode()
              .setRotorBootOffset(Constants.Turret.hoodMotorBootOffset)
              .setBootPositionOffset(Constants.Turret.startingHoodAngle)
              .setReverseSoftLimit(Constants.Turret.minHoodAngle)
              .setForwardSoftLimit(Constants.Turret.maxHoodAngle)
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(20.0)
              .setPIDConfig(kPositionSlot, new PIDConfig(4, 0, 0, 0, 0.1, 0, 0))
              .setMotionMagicConfig(2.0, 20, 0));

  private final QuixTalonFX m_launchMotor1 =
      new QuixTalonFX(
          Constants.Turret.launchMotor1ID,
          Constants.Turret.launchMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Turret.launchMotor1Invert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(kVelocitySlot, new PIDConfig(0.2, 0, 0, 0, 0.12, 0, 0)));

  private final QuixTalonFX m_launchMotor2 =
      new QuixTalonFX(
          Constants.Turret.launchMotor2ID,
          m_launchMotor1,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Turret.launchMotor2Invert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0));

  public TurretSubsystem() {}

  public void setRotorTrims() {
    m_yawMotor.setRotorBootOffset(
        Constants.Turret.yawMotorBootOffset + Constants.RotorTrim.turretYaw.get());
    m_hoodMotor.setRotorBootOffset(
        Constants.Turret.hoodMotorBootOffset + Constants.RotorTrim.turretHood.get());
  }

  public double getYaw() {
    return m_yawMotor.getLatencyCompensatedSensorPosition()
        + LaunchCalculator.kSystemLatencySec * m_yawMotor.getSensorVelocity();
  }

  public void setTargetYaw(double rotation, double velocity, double acceleration) {
    m_targetYaw =
        MathUtils.placeInScope(rotation + Math.toRadians(Constants.Turret.yawTrim.get()), Math.PI);
    m_targetYawVelocity = velocity;
    m_targetYawAcceleration = acceleration;
  }

  public boolean isAtYaw(double tolerance) {
    return Math.abs(MathUtils.placeInScope(getYaw(), m_targetYaw) - m_targetYaw) < tolerance;
  }

  public double getHoodAngle() {
    return m_hoodMotor.getSensorPosition();
  }

  public double getLaunchAngle() {
    return 0.5 * Math.PI - getHoodAngle();
  }

  public void setLaunchAngle(double angle) {
    // Launch angle is 90 deg from hood angle
    m_targetHoodAngle = 0.5 * Math.PI - angle;
  }

  public void setHoodAngle(double angle) {
    m_targetHoodAngle = angle;
  }

  public boolean isAtAngle(double tolerance) {
    return Math.abs(getLaunchAngle() - (0.5 * Math.PI - m_targetHoodAngle)) < tolerance;
  }

  public void setLaunchVelocity(double targetLinearVelocity) {
    m_targetLaunchVel =
        Constants.Turret.linearVelocityToRadsPerSecLookup.get(targetLinearVelocity)
            * Constants.Turret.launchVelocityMultiplier.get();
    Logger.recordOutput("Debug/linearVel", targetLinearVelocity);
    Logger.recordOutput("Debug/radsPerSec", m_targetLaunchVel);
    if (m_targetLaunchVel == 0) {
      m_launchMotor1.setPercentOutput(0.0);
    } else {
      m_launchMotor1.setVelocitySetpoint(kVelocitySlot, m_targetLaunchVel);
    }
  }

  public void setRawLaunchVelocity(double targetLaunchVelocity) {
    // This should only be used during calibration.
    m_targetLaunchVel = targetLaunchVelocity;
    if (m_targetLaunchVel == 0) {
      m_launchMotor1.setPercentOutput(0.0);
    } else {
      m_launchMotor1.setVelocitySetpoint(kVelocitySlot, m_targetLaunchVel);
    }
  }

  public double getLaunchVelocity() {
    return Constants.Turret.radsPerSecToLinearVelocityLookup.get(
        m_launchMotor1.getSensorVelocity());
  }

  public boolean isAtVelocity(double tolerance) {
    if (m_targetLaunchVel >= Constants.Turret.maxLaunchVelocityRadsPerSec) {
      return false;
    }
    return Math.abs(
            Constants.Turret.radsPerSecToLinearVelocityLookup.get(m_targetLaunchVel)
                - getLaunchVelocity())
        < tolerance;
  }

  @Override
  public void periodic() {
    LoggerHelper.recordCurrentCommand(this);

    // Yaw
    double kV = 1.5;
    double kA = 0.001;
    double yawFF = kV * m_targetYawVelocity + kA * m_targetYawAcceleration;
    double goal = m_targetYaw;

    // Hysteresis for yaw clamping.
    // The dead zone is from maxYawAngle (310 deg) to minYawAngle (50 deg) going through 0.
    // Once clamped to one side, we require 30 deg of hysteresis past the opposite boundary
    // (measured through the dead zone) before snapping to the other side.
    final double hysteresis = Math.toRadians(30);
    final double maxAngle = Constants.Turret.maxYawAngle;
    final double minAngle = Constants.Turret.minYawAngle;
    boolean inDeadZone = goal > maxAngle || goal < minAngle;

    if (inDeadZone) {
      if (!m_clampedAtMaxYaw && !m_clampedAtMinYaw) {
        // First time entering dead zone — snap to the closer boundary.
        // Compute angular distance through the dead zone (via 0/2π wraparound) to each boundary.
        double distToMax = goal > maxAngle ? goal - maxAngle : (2 * Math.PI - maxAngle) + goal;
        double distToMin = goal < minAngle ? minAngle - goal : minAngle + (2 * Math.PI - goal);
        if (distToMax <= distToMin) {
          m_clampedAtMaxYaw = true;
        } else {
          m_clampedAtMinYaw = true;
        }
      }

      if (m_clampedAtMaxYaw) {
        // Clamped at max. Check if goal has traveled far enough to snap to min.
        // Distance from minAngle going backward through 0 into the dead zone.
        double distPastMin = goal < minAngle ? minAngle - goal : minAngle + (2 * Math.PI - goal);
        if (distPastMin <= hysteresis) {
          // Within hysteresis band of min — snap to min.
          m_clampedAtMaxYaw = false;
          m_clampedAtMinYaw = true;
          goal = minAngle;
          yawFF = 0;
        } else {
          goal = maxAngle;
          yawFF = 0;
        }
      } else {
        // Clamped at min. Check if goal has traveled far enough to snap to max.
        // Distance from maxAngle going forward through 0 into the dead zone.
        double distPastMax = goal > maxAngle ? goal - maxAngle : (2 * Math.PI - maxAngle) + goal;
        if (distPastMax <= hysteresis) {
          // Within hysteresis band of max — snap to max.
          m_clampedAtMinYaw = false;
          m_clampedAtMaxYaw = true;
          goal = maxAngle;
          yawFF = 0;
        } else {
          goal = minAngle;
          yawFF = 0;
        }
      }
    } else {
      // Goal is within valid range — clear any clamping state.
      m_clampedAtMaxYaw = false;
      m_clampedAtMinYaw = false;
    }

    m_yawMotor.setMotionMagicPositionSetpoint(
        kPositionSlot,
        MathUtils.clamp(goal, Constants.Turret.minYawAngle, Constants.Turret.maxYawAngle),
        yawFF);

    Logger.recordOutput("Turret/Current Yaw Angle (deg)", Math.toDegrees(getYaw()));
    Logger.recordOutput(
        "Turret/Target Yaw Angle (deg)", Math.toDegrees(m_yawMotor.getClosedLoopReference()));
    Logger.recordOutput("Turret/Raw Target Yaw Angle (deg)", Math.toDegrees(goal));
    Logger.recordOutput("Turret/yawFF", yawFF);

    // Hood
    m_hoodMotor.setMotionMagicPositionSetpoint(kPositionSlot, m_targetHoodAngle);

    Logger.recordOutput("Turret/Current Hood Angle (deg)", Math.toDegrees(getHoodAngle()));
    Logger.recordOutput(
        "Turret/Target Hood Angle (deg)", Math.toDegrees(m_hoodMotor.getClosedLoopReference()));

    // Rollers
    Logger.recordOutput(
        "Turret/Current Roller Velocity (rps)",
        Units.radiansToRotations(m_launchMotor1.getSensorVelocity()));
    Logger.recordOutput(
        "Turret/Target Roller Velocity (rps)", Units.radiansToRotations(m_targetLaunchVel));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_yawSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44Foc(1),
          Constants.Turret.yawMotorRatio.reduction(),
          Constants.Turret.simTurretMOI,
          0,
          Constants.Turret.minYawAngle,
          Constants.Turret.maxYawAngle,
          false, // Simulate gravity
          Constants.Turret.startingYawAngle);

  private static final SingleJointedArmSim m_hoodSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44Foc(1),
          Constants.Turret.hoodMotorRatio.reduction(),
          Constants.Turret.simHoodMOI,
          Units.inchesToMeters(2.0),
          Constants.Turret.minHoodAngle,
          Constants.Turret.maxHoodAngle,
          true, // Simulate gravity
          Constants.Turret.startingHoodAngle);

  private static final FlywheelSim m_launchSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(2),
              Constants.Turret.simLaunchMOI,
              Constants.Turret.launchMotorRatio.reduction()),
          DCMotor.getKrakenX60Foc(1));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_yawSim.setInput(m_yawMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_yawSim.update(LoggedRobot.defaultPeriodSecs);
    m_yawMotor.setSimSensorPositionAndVelocity(
        m_yawSim.getAngleRads() - Constants.Turret.startingYawAngle,
        m_yawSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Turret.yawMotorRatio);

    m_hoodSim.setInput(m_hoodMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_hoodSim.update(LoggedRobot.defaultPeriodSecs);
    m_hoodMotor.setSimSensorPositionAndVelocity(
        m_hoodSim.getAngleRads() - Constants.Turret.startingHoodAngle,
        m_hoodSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Turret.hoodMotorRatio);

    m_launchSim.setInput(m_launchMotor1.getPercentOutput() * RobotController.getBatteryVoltage());
    m_launchSim.update(LoggedRobot.defaultPeriodSecs);
    m_launchMotor1.setSimSensorVelocity(
        m_launchSim.getAngularVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Turret.launchMotorRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
