package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;

public class QuixSwerveModuleFactory {
  private final double m_wheelCircumference;
  private final MechanismRatio m_driveRatio;
  private final MechanismRatio m_steeringRatio;
  private final double m_steerDriveCouplingRatio;
  private final PIDConfig m_driveOpenLoopPIDConfig;
  private final PIDConfig m_driveClosedLoopPIDConfig;
  private final PIDConfig m_steeringPIDConfig;

  public QuixSwerveModuleFactory(
      final double wheelCircumference,
      final MechanismRatio driveRatio,
      final MechanismRatio steeringRatio,
      final double steerDriveCouplingRatio,
      final PIDConfig driveOpenLoopPIDConfig,
      final PIDConfig driveClosedLoopPIDConfig,
      final PIDConfig steeringPIDConfig) {
    m_wheelCircumference = wheelCircumference;
    m_steeringRatio = steeringRatio;
    m_driveRatio = driveRatio;
    m_steerDriveCouplingRatio = steerDriveCouplingRatio;
    m_driveOpenLoopPIDConfig = driveOpenLoopPIDConfig;
    m_driveClosedLoopPIDConfig = driveClosedLoopPIDConfig;
    m_steeringPIDConfig = steeringPIDConfig;
  }

  public QuixSwerveModule createModule(
      final Translation2d position,
      final CANDeviceID driveMotorID,
      final CANDeviceID steeringMotorID,
      final CANDeviceID absEncoderID,
      final double absEncoderOffsetRad) {
    return new QuixSwerveModule(
        position,
        driveMotorID,
        steeringMotorID,
        absEncoderID,
        m_driveOpenLoopPIDConfig,
        m_driveClosedLoopPIDConfig,
        m_steeringPIDConfig,
        m_driveRatio,
        m_steeringRatio,
        m_steerDriveCouplingRatio,
        absEncoderOffsetRad,
        m_wheelCircumference);
  }
}
