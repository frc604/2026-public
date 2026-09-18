package frc.quixlib.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.localization.QuixSwerveLocalizer;
import frc.quixlib.localization.SwerveDriveOdometryMeasurement;
import frc.quixlib.maplesim.QuixSwerveDriveSimulation;
import frc.quixlib.math.MathUtils;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineVisionPacket;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.ArrayList;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public abstract class QuixSwerve extends SubsystemBase {
  private final QuixPigeon2 m_imu;
  private final ArrayList<QuixVisionCamera> m_cameras;
  private final QuixSwerveModule[] m_modules;
  private final SwerveDriveKinematics m_kinematics;
  private final QuixSwerveModuleSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint m_prevSetpoint;

  private static final SwerveSetpointGenerator.KinematicLimits m_kinematicLimits =
      new SwerveSetpointGenerator.KinematicLimits();
  private final SwerveSetpointGenerator m_setpointGenerator254;
  private SwerveSetpoint m_prevSetpoint254;

  private final QuixSwerveLocalizer m_localizer;
  private final QuixSwerveController m_driveController;

  private final QuixVisionSim m_visionSim;
  private final QuixSwerveDriveSimulation m_swerveDriveSim;

  // For computing simulated IMU acceleration
  private ChassisSpeeds m_prevSimChassisSpeeds = new ChassisSpeeds();

  // For computing velocities
  private Twist2d m_velocity = new Twist2d();
  private Twist2d m_fieldRelativeVelocity = new Twist2d();

  // For computing robot and field-relative acceleration
  private Twist2d m_prevVelocity = new Twist2d();
  private Twist2d m_acceleration = new Twist2d();
  private Twist2d m_prevFieldRelativeVelocity = new Twist2d();
  private Twist2d m_fieldRelativeAcceleration = new Twist2d();
  private double m_prevPeriodicTimestamp = Timer.getTimestamp();

  private final Field2d m_fieldViz;

  /**
   * Swerve drive class that handles all swerve-related functions, including kinematics, control,
   * and localization.
   */
  public QuixSwerve(
      final QuixPigeon2 imu,
      final ArrayList<QuixVisionCamera> cameras,
      final double maxDriveSpeed,
      final double maxModuleAcceleration,
      final double maxModuleSteeringRate,
      final QuixSwerveController driveController,
      final Fiducial[] targets,
      final QuixVisionSim visionSim,
      final Field2d fieldViz) {
    m_imu = imu;
    m_cameras = cameras;
    m_modules = createModules();
    m_kinematics = new SwerveDriveKinematics(getModulePositions());
    m_setpointGenerator =
        new QuixSwerveModuleSetpointGenerator(
            m_kinematics, maxDriveSpeed, maxModuleAcceleration, maxModuleSteeringRate);
    m_prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

    m_kinematicLimits.kMaxDriveVelocity = maxDriveSpeed;
    m_kinematicLimits.kMaxDriveAcceleration = maxModuleAcceleration;
    m_kinematicLimits.kMaxSteeringVelocity = maxModuleSteeringRate;
    m_setpointGenerator254 = new SwerveSetpointGenerator(m_kinematics, getModulePositions());
    m_prevSetpoint254 = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

    m_localizer =
        new QuixSwerveLocalizer(
            m_kinematics,
            new Rotation2d(m_imu.getContinuousYaw()),
            getModulePositionStates(),
            new Pose2d(),
            targets,
            m_cameras);
    m_driveController = driveController;
    zeroModuleEncoders();

    m_visionSim = visionSim;

    // Setup Viz
    m_fieldViz = fieldViz;

    m_swerveDriveSim =
        Robot.isSimulation()
            ? new QuixSwerveDriveSimulation(
                DriveTrainSimulationConfig.Default()
                    .withRobotMass(Pounds.of(140))
                    .withGyro(COTS.ofPigeon2())
                    .withBumperSize(Inches.of(31), Inches.of(31)),
                m_modules,
                1.0, // Wheel CoF
                new Pose2d(1, 1, Rotation2d.kZero))
            : null;
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveDriveSim);
    }
  }

  /**
   * Returns an array of QuixSwerveModules that define the individual module configurations. Called
   * on construction.
   */
  protected abstract QuixSwerveModule[] createModules();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (var module : m_modules) {
      module.m_driveMotor.checkFaultsAndReconfigureIfNecessary();
      module.m_steeringMotor.checkFaultsAndReconfigureIfNecessary();
      // TODO: Handle reconfiguration if necessary.
    }

    final var chassisSpeeds = getChassisSpeeds();

    // Update localization only when enabled.
    final double startTimestamp = Timer.getFPGATimestamp();
    m_localizer.updateWithLatestPoseEstimate();
    if (DriverStation.isEnabled() || true) {
      final double yaw = m_imu.getLatencyCompensatedContinuousYaw();
      final var odometryMeasurement =
          new SwerveDriveOdometryMeasurement(new Rotation2d(yaw), getModulePositionStates());

      // Combining measurements from all cameras.
      final ArrayList<PipelineVisionPacket> visionMeasurements = new ArrayList<>();
      for (var camera : m_cameras) {
        visionMeasurements.add(camera.getLatestMeasurement());
      }

      m_localizer.update(odometryMeasurement, visionMeasurements, chassisSpeeds);
    }
    final double endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("Swerve/LocalizerUpdateMs", (endTimestamp - startTimestamp) * 1000.0);

    // Plot
    m_fieldViz.getObject("Odometry").setPose(m_localizer.getOdometryPose());
    m_fieldViz.getObject("Localizer Raw").setPose(m_localizer.getRawPose());
    m_fieldViz.getObject("Localizer").setPose(m_localizer.getPose());

    // Compute robot-relative velocity
    m_velocity =
        new Twist2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);

    // Compute field-relative velocity
    final double c = getPose().getRotation().getCos();
    final double s = getPose().getRotation().getSin();
    m_fieldRelativeVelocity =
        new Twist2d(
            m_velocity.dx * c - m_velocity.dy * s,
            m_velocity.dx * s + m_velocity.dy * c,
            m_velocity.dtheta);

    final double currentTimestamp = Timer.getTimestamp();
    final double dt = currentTimestamp - m_prevPeriodicTimestamp;
    m_prevPeriodicTimestamp = currentTimestamp;

    // Compute robot-relative acceleration by differentiating robot-relative velocity
    m_acceleration =
        new Twist2d(
            (m_velocity.dx - m_prevVelocity.dx) / dt,
            (m_velocity.dy - m_prevVelocity.dy) / dt,
            (m_velocity.dtheta - m_prevVelocity.dtheta) / dt);
    m_prevVelocity = m_velocity;

    // Compute field-relative acceleration by differentiating field-relative velocity
    m_fieldRelativeAcceleration =
        new Twist2d(
            (m_fieldRelativeVelocity.dx - m_prevFieldRelativeVelocity.dx) / dt,
            (m_fieldRelativeVelocity.dy - m_prevFieldRelativeVelocity.dy) / dt,
            (m_fieldRelativeVelocity.dtheta - m_prevFieldRelativeVelocity.dtheta) / dt);
    m_prevFieldRelativeVelocity = m_fieldRelativeVelocity;

    // Log
    LoggerHelper.recordCurrentCommand(this);
    Logger.recordOutput("Swerve/Odometry", m_localizer.getOdometryPose());
    Logger.recordOutput("Swerve/Localizer Raw", m_localizer.getRawPose());
    Logger.recordOutput("Swerve/Localizer", m_localizer.getPose());
    Logger.recordOutput("Swerve/TrigPose", m_localizer.getTrigPose());
    Logger.recordOutput("Swerve/MultitagPose", m_localizer.getMultitagPose());
    Logger.recordOutput("Swerve/Velocity", m_velocity);
    Logger.recordOutput("Swerve/Acceleration", m_acceleration);
    Logger.recordOutput("Swerve/Field Relative Velocity", m_fieldRelativeVelocity);
    Logger.recordOutput("Swerve/Field Relative Acceleration", m_fieldRelativeAcceleration);
  }

  /** Sets the IMU and localizer to the given pose. */
  public void resetPose(final Pose2d pose) {
    m_imu.setContinuousYaw(pose.getRotation().getRadians());
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()), getModulePositionStates(), pose);
  }

  public void trackAllTags() {
    m_localizer.trackAllTags();
  }

  public void setTagsToTrack(int[] tagIDs) {
    m_localizer.setTagsToTrack(tagIDs);
  }

  /**
   * Drive with open loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveOpenLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, false);
  }

  /**
   * Drive with closed loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveClosedLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, true);
  }

  /**
   * Drive with the given translation and rotation velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether the provided velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   * @param isClosedLoop Whether to use closed-loop module velocities
   */
  private void drive(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub,
      final boolean isClosedLoop) {
    final ChassisSpeeds desiredChassisSpeeds =
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVel, yVel, thetaVel, new Rotation2d(m_imu.getContinuousYaw()))
                : new ChassisSpeeds(xVel, yVel, thetaVel),
            LoggedRobot.defaultPeriodSecs);
    final var curStates = getModuleStates();

    // Compute desired module states without antiscrub for diagnostics.
    final var noAntiscrubModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    // Unwrap states
    for (int i = 0; i < noAntiscrubModuleStates.length; i++) {
      noAntiscrubModuleStates[i] =
          QuixSwerveModuleSetpointGenerator.optimizeModule(
              noAntiscrubModuleStates[i], curStates[i]);
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(
        noAntiscrubModuleStates,
        desiredChassisSpeeds,
        Constants.Swerve.maxDriveSpeed,
        Constants.Swerve.maxDriveSpeed,
        Constants.Swerve.maxAngularVelocity);

    final double startTimestamp = Timer.getFPGATimestamp();
    final int antiScrubMode = 0;
    switch (antiScrubMode) {
      case 604:
        {
          m_prevSetpoint =
              m_setpointGenerator.getFeasibleSetpoint(
                  m_prevSetpoint, desiredChassisSpeeds, allowedScrub);
          setDesiredModuleStates(m_prevSetpoint.moduleStates, isClosedLoop);
          break;
        }
      case 254:
        {
          var setpoint254 =
              m_setpointGenerator254.generateSetpoint(
                  m_kinematicLimits,
                  m_prevSetpoint254,
                  desiredChassisSpeeds,
                  LoggedRobot.defaultPeriodSecs);

          // Unwrap states
          var newStates254 = new SwerveModuleState[setpoint254.moduleStates.length];
          for (int i = 0; i < setpoint254.moduleStates.length; i++) {
            newStates254[i] = new SwerveModuleState();
            newStates254[i].speedMetersPerSecond = setpoint254.moduleStates[i].speedMetersPerSecond;
            newStates254[i].angle =
                new Rotation2d(
                    MathUtils.placeInScope(
                        setpoint254.moduleStates[i].angle.getRadians(),
                        curStates[i].angle.getRadians()));
          }
          setpoint254.moduleStates = newStates254;
          m_prevSetpoint254 = setpoint254;
          setDesiredModuleStates(m_prevSetpoint254.moduleStates, isClosedLoop);
          break;
        }
      default:
        {
          setDesiredModuleStates(noAntiscrubModuleStates, isClosedLoop);
          break;
        }
    }
    final double endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("Swerve/SetpointGeneratorTimeMs", (endTimestamp - startTimestamp) * 1000.0);
    Logger.recordOutput("Swerve/Module States/Raw", noAntiscrubModuleStates);
    Logger.recordOutput("Swerve/Module States/Target", getLastCommandedModuleStates());
    Logger.recordOutput("Swerve/Module States/Actual", getModuleStates());
  }

  /**
   * Drives the swerve to the target pose.
   *
   * @param targetPose The target pose in field-frame
   * @param xVelocityRef Field-relative x-velocity feed-forward
   * @param yVelocityRef Field-relative y-velocity feed-forward
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   * @param useTrigPose Use trig-based localization
   * @return the error between the current pose and the target pose
   */
  public Pose2d driveToPose(
      final Pose2d targetPose,
      final double xVelocityRef,
      final double yVelocityRef,
      final double thetaVelocityRef,
      final double allowedScrub) {
    final Pose2d currentPose = getPose();
    final Pose2d poseError =
        new Pose2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY(),
            targetPose.getRotation().minus(currentPose.getRotation()));

    m_fieldViz.getObject("Target Pose").setPose(targetPose);
    Logger.recordOutput("Swerve/Drive to Pose/Target Pose", targetPose);
    Logger.recordOutput("Swerve/Drive to Pose/X actual", currentPose.getX());
    Logger.recordOutput("Swerve/Drive to Pose/X target", targetPose.getX());
    Logger.recordOutput("Swerve/Drive to Pose/X error", poseError.getX());
    Logger.recordOutput("Swerve/Drive to Pose/Y actual", currentPose.getY());
    Logger.recordOutput("Swerve/Drive to Pose/Y target", targetPose.getY());
    Logger.recordOutput("Swerve/Drive to Pose/Y error", poseError.getY());
    Logger.recordOutput(
        "Swerve/Drive to Pose/Theta actual (degrees)", currentPose.getRotation().getDegrees());
    Logger.recordOutput(
        "Swerve/Drive to Pose/Theta target (degrees)", targetPose.getRotation().getDegrees());
    Logger.recordOutput(
        "Swerve/Drive to Pose/Theta error (degrees)", poseError.getRotation().getDegrees());
    final ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(
            currentPose, targetPose, xVelocityRef, yVelocityRef, thetaVelocityRef);
    driveClosedLoop(
        fieldRelativeChassisSpeeds.vxMetersPerSecond,
        fieldRelativeChassisSpeeds.vyMetersPerSecond,
        fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
        /* fieldRelative= */ false,
        allowedScrub);
    return poseError;
  }

  /**
   * Turns the swerve in-place to the target angle.
   *
   * @param angle The target angle in field-frame
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void turnToAngle(
      final Rotation2d angle, final double thetaVelocityRef, final double allowedScrub) {
    final Pose2d targetPose = new Pose2d(getPose().getTranslation(), angle);
    final ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(m_localizer.getPose(), targetPose, 0.0, 0.0, thetaVelocityRef);
    driveOpenLoop(
        0.0,
        0.0,
        fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
        /* fieldRelative= */ false,
        allowedScrub);
  }

  /** Drive open loop with zero velocity. */
  public void stop() {
    driveOpenLoop(0.0, 0.0, 0.0, /* fieldRelative= */ false, Double.POSITIVE_INFINITY);
  }

  /** Turns modules to form an X pointing at the origin with zero velocity. */
  public void stopWithX() {
    final var moduleStates = getModuleStates();
    final SwerveModuleState[] desiredStates = new SwerveModuleState[moduleStates.length];
    for (int i = 0; i < moduleStates.length; i++) {
      final double angle =
          Math.atan2(m_modules[i].getPosition().getY(), m_modules[i].getPosition().getX());
      desiredStates[i] =
          new SwerveModuleState(
              0.0,
              new Rotation2d(MathUtils.placeInScope(angle, moduleStates[i].angle.getRadians())));
    }
    setDesiredModuleStates(desiredStates, false);
  }

  private ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    final Rotation2d rotation = getPose().getRotation();
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    final double vxField =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    final double vyField =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();
    return new ChassisSpeeds(vxField, vyField, chassisSpeeds.omegaRadiansPerSecond);
  }

  public FieldSpeeds getFieldSpeeds() {
    final Rotation2d rotation = getPose().getRotation();
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    final double vxField =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    final double vyField =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();
    return new FieldSpeeds(vxField, vyField, chassisSpeeds.omegaRadiansPerSecond);
  }

  private Translation2d[] getModulePositions() {
    final Translation2d[] positions = new Translation2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  private SwerveModuleState[] getLastCommandedModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getLastCommandedState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositionStates() {
    final SwerveModulePosition[] states = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getPositionState();
    }
    return states;
  }

  public Pose2d getPose() {
    return m_localizer.getMultitagPose();
  }

  /** Returns the velocity (m/s for translation & rad/s for rotation) in the robot frame. */
  public Twist2d getVelocity() {
    return m_velocity;
  }

  /** Returns the acceleration (m/s/s for translation & rad/s/s for rotation) in the robot frame. */
  public Twist2d getAcceleration() {
    return new Twist2d();
    // return m_acceleration;
  }

  /** Returns the velocity (m/s for translation & rad/s for rotation) in the field frame. */
  public Twist2d getFieldRelativeVelocity() {
    return m_fieldRelativeVelocity;
  }

  /** Returns the acceleration (m/s/s for translation & rad/s/s for rotation) in the field frame. */
  public Twist2d getFieldRelativeAcceleration() {
    return new Twist2d();
    // return m_fieldRelativeAcceleration;
  }

  public void setContinuousYaw(double yaw) {
    m_imu.setContinuousYaw(yaw);
    final Pose2d curPose = getPose();
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()),
        getModulePositionStates(),
        new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(yaw)));
  }

  /** Sets the supply current limit for all drive motors. */
  public void setDriveSupplyCurrentLimit(final double amps) {
    for (var module : m_modules) {
      module.m_driveMotor.setSupplyCurrentLimit(amps);
    }
  }

  private void zeroModuleEncoders() {
    for (var module : m_modules) {
      module.zeroToAbsPosition();
    }
  }

  private void setDesiredModuleStates(
      final SwerveModuleState[] desiredStates, final boolean isClosedLoop) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i], isClosedLoop);
    }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  /** Sets the simulated pose to the given pose. */
  public void resetSimPose(final Pose2d pose) {
    m_swerveDriveSim.setSimulationWorldPose(pose);
    m_visionSim.resetSimPose(pose);
  }

  public QuixSwerveDriveSimulation getDriveSim() {
    return m_swerveDriveSim;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Treat replay as non-simulation.
    if (Constants.isReplay) {
      return;
    }

    final Pose2d simPose = m_swerveDriveSim.getSimulatedDriveTrainPose();
    Logger.recordOutput("FieldSimulation/Robot Pose", simPose);

    // Update vision sim with sim pose.
    m_visionSim.updatePose(simPose);

    // Update IMU based on swerve drive sim.
    final var gyroSim = m_swerveDriveSim.getGyroSimulation();
    m_imu.setSimContinuousYaw(
        MathUtils.placeInScope(gyroSim.getGyroReading().getRadians(), m_imu.getContinuousYaw()));
    m_imu.setSimYawRate(gyroSim.getMeasuredAngularVelocity().in(RadiansPerSecond));

    // Compute and set simulated IMU accelerations (robot-relative).
    final ChassisSpeeds currentSpeeds =
        m_swerveDriveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    final double dt = LoggedRobot.defaultPeriodSecs;
    final double accelX =
        (currentSpeeds.vxMetersPerSecond - m_prevSimChassisSpeeds.vxMetersPerSecond) / dt;
    final double accelY =
        (currentSpeeds.vyMetersPerSecond - m_prevSimChassisSpeeds.vyMetersPerSecond) / dt;
    m_imu.setSimAccelX(accelX);
    m_imu.setSimAccelY(accelY);
    m_imu.setSimAccelZ(9.81);
    m_prevSimChassisSpeeds = currentSpeeds;

    // Always set the pose to the ground truth sim pose if not simming localization.
    if (!Constants.simLocalization) {
      resetPose(simPose);
    }
  }
  // --- END STUFF FOR SIMULATION ---
}
