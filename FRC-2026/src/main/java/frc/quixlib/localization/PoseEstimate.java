package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The PoseEstimate class represents an estimate of a robot's pose (position and orientation) at a
 * given point in time. It includes an identifier, the pose itself, and a flag indicating whether
 * vision data was used in the estimation.
 */
public class PoseEstimate {
  private final int m_id;
  private final Pose2d m_pose;
  private final boolean m_hasVision;

  public PoseEstimate() {
    this(0, Pose2d.kZero, false);
  }

  public PoseEstimate(final int id, final Pose2d pose, final boolean hasVision) {
    m_id = id;
    m_pose = pose;
    m_hasVision = hasVision;
  }

  public int getID() {
    return m_id;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public boolean hasVision() {
    return m_hasVision;
  }

  @Override
  public String toString() {
    return String.format("PoseEstimate(%s, %s, %s)", m_id, m_pose, m_hasVision);
  }

  // Struct for serialization.
  public static final PoseEstimateStruct struct = new PoseEstimateStruct();
}
