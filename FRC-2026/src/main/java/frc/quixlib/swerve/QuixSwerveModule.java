package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Robot;

public class QuixSwerveModule {
  private static final int kDriveOpenLoopVelocityPIDSlot = 0;
  private static final int kDriveClosedLoopVelocityPIDSlot = 1;
  private static final int kSteeringPIDSlot = 0;

  private final Translation2d m_position;
  private final double m_absEncoderOffsetRad;
  protected final QuixTalonFX m_driveMotor;
  protected final QuixTalonFX m_steeringMotor;
  private final MechanismRatio m_driveRatio;
  private final MechanismRatio m_steeringRatio;
  // For every steering rotation with the wheel fixed, the drive motor turns this much. May be
  // negative.
  private final double m_steerDriveCouplingRatio;
  private final QuixCANCoder m_absSteeringEncoder;
  private final double m_wheelCircumference;

  private SwerveModuleState m_lastCommandedState;
  private double m_steeringZeroPosition = 0.0;

  private double m_prevStateTime;

  public QuixSwerveModule(
      final Translation2d position,
      final CANDeviceID driveMotorID,
      final CANDeviceID steeringMotorID,
      final CANDeviceID absEncoderID,
      final PIDConfig driveOpenLoopPIDConfig,
      final PIDConfig driveClosedLoopPIDConfig,
      final PIDConfig steeringPIDConfig,
      final MechanismRatio driveRatio,
      final MechanismRatio steeringRatio,
      final double steerDriveCouplingRatio,
      final double absEncoderOffsetRad,
      final double wheelCircumference) {

    m_position = position;
    m_absEncoderOffsetRad = absEncoderOffsetRad;
    m_driveMotor =
        new QuixTalonFX(
            driveMotorID,
            driveRatio,
            QuixTalonFX.makeDefaultConfig()
                .setBrakeMode()
                .setSupplyCurrentLimit(40)
                .setStatorCurrentLimit(120)
                .setPIDConfig(kDriveOpenLoopVelocityPIDSlot, driveOpenLoopPIDConfig)
                .setPIDConfig(kDriveClosedLoopVelocityPIDSlot, driveClosedLoopPIDConfig)
                .setEnableFOC(true));
    m_steeringMotor =
        new QuixTalonFX(
            steeringMotorID,
            steeringRatio,
            QuixTalonFX.makeDefaultConfig()
                .setInverted(true)
                .setSupplyCurrentLimit(30)
                .setStatorCurrentLimit(60)
                .setPIDConfig(kSteeringPIDSlot, steeringPIDConfig));
    m_driveRatio = driveRatio;
    m_steeringRatio = steeringRatio;
    m_steerDriveCouplingRatio = steerDriveCouplingRatio;
    m_absSteeringEncoder = new QuixCANCoder(absEncoderID, new MechanismRatio());
    m_wheelCircumference = wheelCircumference;

    zeroToAbsPosition();
    m_lastCommandedState = getState();

    // For simulation
    if (Robot.isSimulation()) {
      m_steeringSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(1),
              steeringRatio.reduction(),
              0.03, // MOI
              0.0, // Length (m)
              Double.NEGATIVE_INFINITY, // Min angle
              Double.POSITIVE_INFINITY, // Max angle
              false, // Simulate gravity
              0.0 // Starting angle (rads)
              );
      m_simTimer = new Timer();
      m_simTimer.start();
    }
  }

  public Translation2d getPosition() {
    return m_position;
  }

  public void zeroToAbsPosition() {
    final double absAngle = m_absSteeringEncoder.getAbsPosition() - m_absEncoderOffsetRad;
    m_steeringZeroPosition = m_steeringMotor.getSensorPosition() - absAngle;
  }

  public double getAbsoluteSensorAngle() {
    return m_absSteeringEncoder.getAbsPosition();
  }

  public double getSteeringAngle() {
    return m_steeringMotor.getSensorPosition() - m_steeringZeroPosition;
  }

  public void setDesiredState(final SwerveModuleState desiredState, final boolean isClosedLoop) {
    double curTime = Timer.getTimestamp();
    double dT = curTime - m_prevStateTime;
    double a =
        m_prevStateTime != 0.0
            ? ((desiredState.speedMetersPerSecond - m_lastCommandedState.speedMetersPerSecond) / dT)
            : 0.0;
    m_driveMotor.setVelocitySetpoint(
        isClosedLoop ? kDriveClosedLoopVelocityPIDSlot : kDriveOpenLoopVelocityPIDSlot,
        desiredState.speedMetersPerSecond,
        a,
        0.0);

    m_steeringMotor.setPositionSetpoint(
        kSteeringPIDSlot, desiredState.angle.getRadians() + m_steeringZeroPosition);

    // Save this state
    m_lastCommandedState = desiredState;
    m_prevStateTime = curTime;
  }

  public SwerveModuleState getState() {
    final double velocity = m_driveMotor.getSensorVelocity();
    final double angle = getSteeringAngle();
    return new SwerveModuleState(velocity, new Rotation2d(angle));
  }

  public SwerveModuleState getLastCommandedState() {
    return m_lastCommandedState;
  }

  public SwerveModulePosition getPositionState() {
    final double position = m_driveMotor.getLatencyCompensatedSensorPosition();
    final double angle =
        m_steeringMotor.getLatencyCompensatedSensorPosition() - m_steeringZeroPosition;

    // Compute drive position offset (in meters) due to steer coupling.
    final double steerCouplingDriveOffset =
        m_driveMotor
            .getMechanismRatio()
            .sensorRadiansToMechanismPosition(m_steerDriveCouplingRatio * angle);

    return new SwerveModulePosition(position - steerCouplingDriveOffset, new Rotation2d(angle));
  }

  public Translation2d getGroundForceVector(double maxFrictionForce) {
    final double motorTorque =
        DCMotor.getKrakenX60Foc(1).getTorque(m_driveMotor.getTorqueCurrent());
    final double groundForce =
        Math.abs(motorTorque)
            * m_driveMotor.getMechanismRatio().reduction()
            / (m_wheelCircumference / (2.0 * Math.PI));
    return new Translation2d(
        MathUtils.clamp(groundForce, -maxFrictionForce, maxFrictionForce),
        new Rotation2d(getSteeringAngle() + (motorTorque < 0.0 ? Math.PI : 0.0)));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private Timer m_simTimer;
  private SingleJointedArmSim m_steeringSim;

  /** Simulate one module with naive physics model. */
  public void updateSimSubtick(double velocity) {
    final double dt = m_simTimer.get();
    m_simTimer.reset();

    // Set drive speed from QuixSwerve sim
    m_driveMotor.setSimSensorVelocity(velocity, dt, m_driveRatio);

    // Simulate steering
    m_steeringSim.setInput(
        m_steeringMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_steeringSim.update(dt);
    m_steeringMotor.setSimSensorPositionAndVelocity(
        m_steeringSim.getAngleRads(), m_steeringSim.getVelocityRadPerSec(), dt, m_steeringRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
