package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.quixlib.math.CameraMathUtils;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineVisionPacket;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.wpilib.InterpolateableChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Fiducials;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

/**
 * The QuixSwerveLocalizer class is responsible for managing the localization of a swerve drive
 * robot. It integrates odometry and vision measurements to provide an accurate estimate of the
 * robot's pose.
 *
 * <p>This class handles:
 *
 * <ul>
 *   <li>Sending and receiving data from NetworkTables
 *   <li>Tracking AprilTags for vision-based localization
 *   <li>Maintaining buffers for odometry and chassis speeds
 *   <li>Updating the robot's pose using odometry and vision measurements
 *   <li>Publishing measurements and pose estimates to NetworkTables
 * </ul>
 *
 * <p>Key components:
 *
 * <ul>
 *   <li>NTManager: Manages NetworkTables communication
 *   <li>SwerveDriveOdometry: Handles odometry calculations
 *   <li>TimeInterpolatableBuffer: Buffers for interpolating odometry and chassis speeds
 *   <li>TreeMap: Stores measurements for processing
 * </ul>
 *
 * <p>Usage:
 *
 * <ul>
 *   <li>Initialize with kinematics, initial gyro angle, module positions, initial pose, targets,
 *       and cameras
 *   <li>Call update() method with odometry, vision packets, and chassis speeds to update the pose
 *   <li>Use getPose() to retrieve the latest pose estimate
 *   <li>Use resetPose() to reset the localizer to a specific pose
 * </ul>
 *
 * <p>Note: This class assumes that the robot is equipped with vision cameras and AprilTags for
 * localization.
 *
 * @param kinematics The kinematics of the swerve drive.
 * @param initialGyroAngle The initial angle of the gyro.
 * @param modulePositions The positions of the swerve modules.
 * @param priori The initial pose of the robot.
 * @param targets The fiducial targets for vision-based localization.
 * @param cameras The vision cameras used for detecting fiducial targets.
 */
public class QuixSwerveLocalizer {
  // Manages sending and receiving from NetworkTables.
  private final NTManager m_networkTable = new NTManager();
  private final ArrayList<QuixVisionCamera> m_cameras;

  // If empty, uses all tags.
  private final HashSet<Integer> m_tagsToTrack = new HashSet<>();

  // Allowed tags for trig pose computation (hub tags)
  // Red alliance hub: 2, 3, 4, 5 and 8, 9, 10, 11
  // Blue alliance hub: 18, 19, 20, 21 and 24, 25, 26, 27
  private static final Set<Integer> RED_TRIG_POSE_TAGS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
  private static final Set<Integer> BLUE_TRIG_POSE_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

  // ID of the current measurement. Used to sync between Robot and DriverStation.
  private int m_currentID = 0;
  // Map of {id: time}
  private final HashMap<Integer, Double> m_idToTimeMap = new HashMap<>();
  // Map of {time : SwerveDriveOdometryMeasurement}
  private final ConcurrentSkipListMap<Double, SwerveDriveOdometryMeasurement> m_timeToOdometryMap =
      new ConcurrentSkipListMap<>();
  // Buffer of poses so we can get the interpolated pose at the time of a vision measurement.
  private final double kBufferHistorySeconds = 10.0; // s
  private final TimeInterpolatableBuffer<Pose2d> m_rawOdometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferHistorySeconds);
  // Buffer of chassis speeds so we can get the interpolated chassis speed at the time of a vision
  // measurement.
  private final TimeInterpolatableBuffer<InterpolateableChassisSpeeds> m_chassisSpeedsBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferHistorySeconds);
  // Map of {time: Pair<NTOdometryMeasurement, NTVisionMeasurement>}
  private final TreeMap<Double, Measurement> m_timeToMeasurementMap = new TreeMap<>();
  // ID of the last measurement that was updated.
  private int m_lastUpdatedID = -1;

  // Continuous odometry from the last reset. Used as input to the localizer.
  private final SwerveDriveOdometry m_rawOdometry;
  // Odometry played back on top of the latest localiation estimate.
  private final SwerveDriveOdometry m_playbackOdometry;
  // Odometry played back on top of the latest single-tag localiation estimate.
  private final SwerveDriveOdometry m_trigPosePlaybackOdometry;
  // Odometry played back on top of the latest multitag localiation estimate.
  private final SwerveDriveOdometry m_multitagPlaybackOdometry;
  // Latest raw localization estimate from DS.
  private PoseEstimate m_latestRawEstimate = new PoseEstimate();

  // Measurements within |kMutableTimeBuffer| of the current time are not considered final.
  // This gives us a chance to associate new vision measurements with an past interpolated
  // odometry measurements.
  private final double kMutableTimeBuffer = 0.05; // seconds

  public QuixSwerveLocalizer(
      final SwerveDriveKinematics kinematics,
      final Rotation2d initialGyroAngle,
      final SwerveModulePosition[] modulePositions,
      final Pose2d priori,
      final Fiducial[] targets,
      final ArrayList<QuixVisionCamera> cameras) {
    m_rawOdometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_playbackOdometry =
        new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_trigPosePlaybackOdometry =
        new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_multitagPlaybackOdometry =
        new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_networkTable.publishTargets(targets);
    m_cameras = cameras;
    trackAllTags();
  }

  /** Resets the localizer to the given pose. */
  public void resetPose(
      final Rotation2d gyroAngle, final SwerveModulePosition[] modulePositions, final Pose2d pose) {
    m_rawOdometry.resetPosition(gyroAngle, modulePositions, pose);
    m_playbackOdometry.resetPosition(gyroAngle, modulePositions, pose);
    m_trigPosePlaybackOdometry.resetPosition(gyroAngle, modulePositions, pose);
    m_multitagPlaybackOdometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  public void trackAllTags() {
    m_tagsToTrack.clear();
    for (var tag : Fiducials.aprilTagFiducials) {
      m_tagsToTrack.add(tag.id());
    }
  }

  public void setTagsToTrack(int[] tagIDs) {
    m_tagsToTrack.clear();
    for (int id : tagIDs) {
      m_tagsToTrack.add(id);
    }
  }

  /** Raw odometry pose. */
  public Pose2d getOdometryPose() {
    return m_rawOdometry.getPoseMeters();
  }

  /** Localizer latency-compensated pose. */
  public Pose2d getPose() {
    return m_playbackOdometry.getPoseMeters();
  }

  /** Trig-based latency-compensated pose. */
  public Pose2d getTrigPose() {
    return m_trigPosePlaybackOdometry.getPoseMeters();
  }

  /** Multitag-based latency-compensated pose. */
  public Pose2d getMultitagPose() {
    return m_multitagPlaybackOdometry.getPoseMeters();
  }

  /** Localizer pose from DS. Use for plotting/debugging only. */
  public Pose2d getRawPose() {
    return m_latestRawEstimate.getPose();
  }

  /** Update with odometry and optional vision. */
  public void update(
      final SwerveDriveOdometryMeasurement odometry,
      final ArrayList<PipelineVisionPacket> visionPackets,
      final ChassisSpeeds chassisSpeeds) {
    // m_networkTable.publishCameras(m_cameras);

    final double startTimestamp = Timer.getFPGATimestamp();
    final double currentTime = Timer.getTimestamp();

    m_rawOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
    m_playbackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
    m_trigPosePlaybackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
    m_multitagPlaybackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
    m_timeToOdometryMap.put(currentTime, odometry);

    final var curPose = m_rawOdometry.getPoseMeters();
    m_rawOdometryPoseBuffer.addSample(currentTime, curPose);
    // m_chassisSpeedsBuffer.addSample(
    //     currentTime, InterpolateableChassisSpeeds.fromChassisSpeeds(chassisSpeeds));

    // // Always save latest odometry.
    // m_timeToMeasurementMap.put(currentTime, new Measurement(curPose));

    // Save data from each camera.
    for (int cameraID = 0; cameraID < visionPackets.size(); cameraID++) {
      //   final ArrayList<Translation3d> detectedTags = new ArrayList<>();
      final var vision = visionPackets.get(cameraID);

      final double measurementTime = vision.getCaptureTimestamp();
      if (measurementTime > 0.0) {
        Logger.recordOutput(
            "Localizer/measurementLatency[" + cameraID + "]", currentTime - measurementTime);
      }

      //   if (!vision.hasTargets()) {
      //     Translation3d[] array = new Translation3d[0];
      //     Logger.recordOutput("Localizer/detectedTags[" + cameraID + "]", array);
      //     continue;
      //   }

      //   // Merge with the existing the measurement if it already exists.
      //   Measurement existingMeasurement = m_timeToMeasurementMap.get(measurementTime);

      //   // If there is no existing measurement, create a new one by interpolating pose.
      //   if (existingMeasurement == null) {
      //     final Pose2d interpolatedPose =
      // m_rawOdometryPoseBuffer.getSample(measurementTime).get();
      //     existingMeasurement = new Measurement(interpolatedPose);
      //     m_timeToMeasurementMap.put(measurementTime, existingMeasurement);
      //   }

      //   // Set vision uncertainty based on chassis speeds.
      //   // The fast we are moving, the more uncertain we are.
      //   // TODO: Tune
      //   final var interpolatedChassisSpeeds =
      // m_chassisSpeedsBuffer.getSample(measurementTime).get();
      //   final double pixelSigma =
      //       Math.max(
      //           100.0,
      //           5.0
      //               + 10.0
      //                   * Math.sqrt(
      //                       interpolatedChassisSpeeds.vxMetersPerSecond
      //                               * interpolatedChassisSpeeds.vxMetersPerSecond
      //                           + interpolatedChassisSpeeds.vyMetersPerSecond
      //                               * interpolatedChassisSpeeds.vyMetersPerSecond)
      //               + 20.0 * Math.abs(interpolatedChassisSpeeds.omegaRadiansPerSecond));
      //   existingMeasurement.setVisionUncertainty(pixelSigma);

      //   for (final var target : vision.getTargets()) {
      //     if (m_tagsToTrack.contains(target.getFiducialId())) {
      //       // Use AprilTag corners.
      //       for (int cornerID = 0; cornerID < target.getDetectedCorners().size(); cornerID++) {
      //         existingMeasurement.addVisionMeasurement(
      //             cameraID,
      //             target.getFiducialId(),
      //             cornerID,
      //             target.getDetectedCorners().get(cornerID));
      //       }
      //       if (target.getFiducialId() <= Fiducials.aprilTagFiducials.length) {
      //         detectedTags.add(
      //             new Pose3d(getPose())
      //                 .transformBy(m_cameras.get(cameraID).getTransform())
      //                 .getTranslation());
      //         detectedTags.add(
      //             Fiducials.aprilTagFiducials[target.getFiducialId() -
      // 1].getPose().getTranslation());
      //       }
      //     }
      //   }
      //   Translation3d[] array = new Translation3d[detectedTags.size()];
      //   detectedTags.toArray(array);
      //   Logger.recordOutput("Localizer/detectedTags[" + cameraID + "]", array);
    }
    // publishImmutableEntries();

    computeMultitagPose();
    // computeTrigPose();

    final double endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("Localizer/UpdateMs", (endTimestamp - startTimestamp) * 1000.0);
  }

  // TODO: Using static here is a bit hacky but gets the job done for now.
  private static double m_lastMultitagUpdateTimestamp = 0.0;

  public static boolean hasRecentMultitagUpdate() {
    return Timer.getTimestamp() - m_lastMultitagUpdateTimestamp < 0.5;
  }

  // Compute the field pose using PhotonVision's multitag result. Prefer the multitag result from
  // the camera that contains the most visible tags.
  private void computeMultitagPose() {
    double latestTimestamp = 0.0;
    int bestCameraID = -1;
    String bestCameraName = null;
    MultiTargetPNPResult bestResult = null;
    int bestActiveTagCount = -1;

    final var alliance = DriverStation.getAlliance();
    final var activeTags =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? BLUE_TRIG_POSE_TAGS
            : RED_TRIG_POSE_TAGS;

    for (int cameraID = 0; cameraID < m_cameras.size(); cameraID++) {
      final var camera = m_cameras.get(cameraID);
      final var measurement = camera.getLatestMeasurement();
      final var multitagResult = measurement.getMultitagResult();

      // Skip measurements too old. Not entirely necessary, but here for safety.
      if (measurement.getCaptureTimestamp() < Timer.getFPGATimestamp() - 0.25) {
        continue;
      }

      if (multitagResult.isEmpty()) {
        continue;
      }

      final var result = multitagResult.get();
      if (result.estimatedPose.ambiguity > 0) {
        continue;
      }

      // Count active tags used.
      int activeTagCount = 0;
      for (final short id : result.fiducialIDsUsed) {
        if (activeTags.contains((int) id)) {
          activeTagCount++;
        }
      }

      if (bestResult == null
          || activeTagCount > bestActiveTagCount
          || (activeTagCount == bestActiveTagCount
              && result.fiducialIDsUsed.size() > bestResult.fiducialIDsUsed.size())
          || (activeTagCount == bestActiveTagCount
              && result.fiducialIDsUsed.size() == bestResult.fiducialIDsUsed.size()
              && measurement.getCaptureTimestamp() > latestTimestamp)) {
        latestTimestamp = measurement.getCaptureTimestamp();
        bestCameraID = cameraID;
        bestCameraName = camera.getName();
        bestResult = result;
        bestActiveTagCount = activeTagCount;
      }
    }

    Logger.recordOutput("Swerve/MultitagPoseCamera", bestCameraName);

    if (bestResult == null) {
      return;
    }

    m_lastMultitagUpdateTimestamp = latestTimestamp;

    // Latest estimate of camera pose in field frame.
    final PnpResult pnp = bestResult.estimatedPose;
    final Pose3d cameraFieldPose = new Pose3d(pnp.best.getTranslation(), pnp.best.getRotation());

    // Get interpolated pose and rotation at measurement time
    final Pose2d interpolatedPose = m_rawOdometryPoseBuffer.getSample(latestTimestamp).get();
    final Rotation2d interpolatedRotation = interpolatedPose.getRotation();

    // Robot pose = Camera pose * (Robot to Camera transform)^-1
    final Transform3d robotToCamera = m_cameras.get(bestCameraID).getTransform();
    final Pose3d robotFieldPose3d = cameraFieldPose.transformBy(robotToCamera.inverse());
    final Pose2d robotFieldPose =
        Constants.Turret.useVisionYaw.get()
            ? robotFieldPose3d.toPose2d()
            : new Pose2d(robotFieldPose3d.getTranslation().toTranslation2d(), interpolatedRotation);

    Logger.recordOutput("Swerve/MultitagPoseCameraPose", cameraFieldPose);
    Logger.recordOutput("Swerve/MultitagPoseRaw", robotFieldPose);
    Logger.recordOutput("Swerve/MultitagPoseTagCount", bestResult.fiducialIDsUsed.size());

    // Visualization rays: Camera position to each tag position.
    Pose2d[] rays = new Pose2d[bestResult.fiducialIDsUsed.size() * 2];
    for (int i = 0; i < bestResult.fiducialIDsUsed.size(); i++) {
      int tagID = bestResult.fiducialIDsUsed.get(i);
      Pose3d tagPose = Fiducials.aprilTagFiducials[tagID - 1].getPose();
      rays[i * 2] = cameraFieldPose.toPose2d();
      rays[i * 2 + 1] = tagPose.toPose2d();
    }
    Logger.recordOutput("Swerve/MultitagPoseRays", rays);

    // Replay odometry on top of latest estimate
    Double curTime = m_timeToOdometryMap.ceilingKey(latestTimestamp);
    if (curTime == null) {
      return;
    }
    final var measurement = m_timeToOdometryMap.get(curTime);
    m_multitagPlaybackOdometry.resetPosition(
        measurement.getGyroAngle(), measurement.getModulePositionStates(), robotFieldPose);

    // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
    // playback odometry.
    while (curTime != null) {
      final SwerveDriveOdometryMeasurement lastMeasurment = m_timeToOdometryMap.get(curTime);
      m_multitagPlaybackOdometry.update(
          lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
      curTime = m_timeToOdometryMap.higherKey(curTime);
    }
  }

  // Compute field pose using least squares ray intersection from multiple tags.
  private void computeTrigPose() {
    // Get latest measurement from the camera with the most targets from the allowed set.
    double latestTimestamp = 0.0;
    QuixVisionCamera cam = null;
    ArrayList<PhotonTrackedTarget> validTargets = null;

    // TODO: Hard coded for current alliance hub. Should be selectable.
    final var alliance = DriverStation.getAlliance();
    final var activeTags =
        alliance.isPresent() && alliance.get() == Alliance.Blue
            ? BLUE_TRIG_POSE_TAGS
            : RED_TRIG_POSE_TAGS;

    for (final var camera : m_cameras) {
      final var measurement = camera.getLatestMeasurement();
      if (!measurement.hasTargets()) {
        continue;
      }

      // Collect all targets from allowed set
      ArrayList<PhotonTrackedTarget> cameraTargets = new ArrayList<>();
      for (final var target : measurement.getTargets()) {
        if (activeTags.contains(target.getFiducialId())) {
          cameraTargets.add(target);
        }
      }

      // Use this camera if it has enough valid targets
      final double numTargets = cameraTargets.size();
      if (numTargets >= 2) {
        if (validTargets == null
            || numTargets > validTargets.size()
            || (numTargets == validTargets.size()
                && measurement.getCaptureTimestamp() > latestTimestamp)) {
          latestTimestamp = measurement.getCaptureTimestamp();
          cam = camera;
          validTargets = cameraTargets;
        }
      }
    }

    // Need at least 2 tags for triangulation
    if (validTargets == null) {
      return;
    }

    // Get interpolated pose and rotation at measurement time
    final Pose2d interpolatedPose = m_rawOdometryPoseBuffer.getSample(latestTimestamp).get();
    final Rotation2d interpolatedRotation = interpolatedPose.getRotation();
    final Transform3d robotToCam = cam.getTransform();
    final Rotation3d robotToFieldRotation = new Rotation3d(0, 0, interpolatedRotation.getRadians());

    // Collect tag positions and ray angles
    ArrayList<Translation2d> tagPositions = new ArrayList<>();
    ArrayList<Double> rayAngles = new ArrayList<>();

    for (final var target : validTargets) {
      // Get tag position (tag ID is 1-indexed, array is 0-indexed)
      final Translation2d tagPos =
          Fiducials.aprilTagFiducials[target.getFiducialId() - 1]
              .getPose()
              .toPose2d()
              .getTranslation();

      // Convert yaw/pitch to 3D direction vector in camera frame
      final var dirCam =
          CameraMathUtils.pinholeBE2Cart(
              Math.toRadians(target.getYaw()), Math.toRadians(target.getPitch()));

      // Transform: camera frame → robot frame → field frame
      final Translation3d dirRobot =
          new Translation3d(dirCam.get(0, 0), dirCam.get(1, 0), dirCam.get(2, 0))
              .rotateBy(robotToCam.getRotation());
      final Translation3d dirField = dirRobot.rotateBy(robotToFieldRotation);

      // Project to 2D and compute ray angle
      final double alpha = Math.atan2(dirField.getY(), dirField.getX());

      tagPositions.add(tagPos);
      rayAngles.add(alpha);
    }

    // Least squares ray intersection: find C that minimizes sum of squared distances to all rays
    // For each ray i: normal n_i = (-sin(alpha_i), cos(alpha_i))
    // Solve: A * C = b, where A = sum(n_i * n_i^T), b = sum(n_i * (p_i dot n_i))
    double a00 = 0, a01 = 0, a11 = 0; // A matrix (symmetric)
    double b0 = 0, b1 = 0; // b vector

    for (int i = 0; i < tagPositions.size(); i++) {
      final Translation2d p = tagPositions.get(i);
      final double alpha = rayAngles.get(i);

      // Normal to ray (perpendicular direction)
      final double nx = -Math.sin(alpha);
      final double ny = Math.cos(alpha);

      // p dot n
      final double pDotN = p.getX() * nx + p.getY() * ny;

      // Accumulate A = sum(n * n^T)
      a00 += nx * nx;
      a01 += nx * ny;
      a11 += ny * ny;

      // Accumulate b = sum(n * (p dot n))
      b0 += nx * pDotN;
      b1 += ny * pDotN;
    }

    // Solve 2x2 system: C = A^(-1) * b
    final double det = a00 * a11 - a01 * a01;

    // Guard against degenerate case (all rays nearly parallel)
    if (Math.abs(det) < 1e-6) {
      return;
    }

    // Camera position in field frame
    final Translation2d cameraFieldPosition =
        new Translation2d((a11 * b0 - a01 * b1) / det, (a00 * b1 - a01 * b0) / det);

    // Transform camera position to robot pose
    final Pose3d cameraPose = Pose3d.kZero.transformBy(robotToCam);
    Pose2d robotPose =
        new Pose2d(
                cameraFieldPosition, interpolatedRotation.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
    // Use gyro angle for robot rotation
    robotPose = new Pose2d(robotPose.getTranslation(), interpolatedRotation);

    Logger.recordOutput("Swerve/TrigPoseRaw", robotPose);
    Logger.recordOutput("Swerve/TrigPoseTagCount", validTargets.size());

    // // Calibration
    // final Pose3d rawCameraPose = new Pose3d(robotPose).transformBy(robotToCam);
    // Logger.recordOutput("Swerve/RawCameraPose", rawCameraPose);
    // final Transform3d calculatedRobotToCameraT =
    //     rawCameraPose.minus(Constants.Cameras.blueCalibrationPose);
    // System.out.println(
    //     Math.toDegrees(calculatedRobotToCameraT.getRotation().getX())
    //         + ", "
    //         + Math.toDegrees(calculatedRobotToCameraT.getRotation().getY())
    //         + ", "
    //         + Math.toDegrees(calculatedRobotToCameraT.getRotation().getZ()));

    // Replay odometry on top of latest estimate
    Double curTime = m_timeToOdometryMap.ceilingKey(latestTimestamp);
    if (curTime == null) {
      return;
    }
    final var measurement = m_timeToOdometryMap.get(curTime);
    m_trigPosePlaybackOdometry.resetPosition(
        measurement.getGyroAngle(), measurement.getModulePositionStates(), robotPose);

    // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
    // playback odometry.
    while (curTime != null) {
      final SwerveDriveOdometryMeasurement lastMeasurment = m_timeToOdometryMap.get(curTime);
      m_trigPosePlaybackOdometry.update(
          lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
      curTime = m_timeToOdometryMap.higherKey(curTime);
    }
  }

  /**
   * Uses the latest pose estimate over NetworkTables and replays the latest odometry on top of it.
   */
  public void updateWithLatestPoseEstimate() {
    final double startTimestamp = Timer.getFPGATimestamp();
    final PoseEstimate estimate = m_networkTable.getLatestPoseEstimate();

    // Save for plotting/debugging purposes.
    m_latestRawEstimate = estimate;

    // Only incorporate estimate if it is new.
    if (m_idToTimeMap.size() == 0 || estimate.getID() == m_lastUpdatedID) {
      final double endTimestamp = Timer.getFPGATimestamp();
      Logger.recordOutput(
          "Localizer/UpdateWithLatestPoseEstimateMs", (endTimestamp - startTimestamp) * 1000.0);
      Logger.recordOutput("Localizer/visionCorrection (m)", 0.0);
      return;
    }
    m_lastUpdatedID = estimate.getID();

    // Save the pose before correction.
    final Pose2d preCorrectionPose = getPose();

    // Start playback odometry at the first time >= the current estimate.
    final double estimateTime = m_idToTimeMap.get(estimate.getID());
    Double curTime = m_timeToOdometryMap.ceilingKey(estimateTime);

    final var measurement = m_timeToOdometryMap.get(curTime);
    m_playbackOdometry.resetPosition(
        measurement.getGyroAngle(), measurement.getModulePositionStates(), estimate.getPose());

    // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
    // playback odometry.
    while (curTime != null) {
      final SwerveDriveOdometryMeasurement lastMeasurment = m_timeToOdometryMap.get(curTime);
      m_playbackOdometry.update(
          lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
      curTime = m_timeToOdometryMap.higherKey(curTime);
    }
    final double endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput(
        "Localizer/UpdateWithLatestPoseEstimateMs", (endTimestamp - startTimestamp) * 1000.0);

    // Log magnitude of correction
    final Pose2d postCorrectionPose = getPose();
    Logger.recordOutput(
        "Localizer/visionCorrection (m)",
        postCorrectionPose.minus(preCorrectionPose).getTranslation().getNorm());
  }

  /** Handles NT publishing, ID finalization, and cleanup. */
  private void publishImmutableEntries() {
    final double currentTime = Timer.getTimestamp();

    // Times are in ascending order.
    final ArrayList<Double> times = new ArrayList<>(m_timeToMeasurementMap.keySet());
    for (final double time : times) {
      // Entries within |kMutableTimeBuffer| of the current time are not considered final.
      // Once we reach this point we are done.
      if (currentTime - time < kMutableTimeBuffer) {
        break;
      }

      // Entries older than |kMutableTimeBuffer| are considered immutable.
      // Assign them an ID and publish them.
      final var measurement = m_timeToMeasurementMap.get(time);
      m_networkTable.publishMeasurement(measurement, m_currentID);
      m_timeToMeasurementMap.remove(time);

      m_idToTimeMap.put(m_currentID, time);
      m_currentID += 1;
    }
  }
}
