package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import org.photonvision.targeting.TargetCorner;

/**
 * The Measurement class represents a set of odometry and vision measurements. It includes the
 * robot's pose and associated uncertainties, as well as vision measurements from multiple cameras.
 */
public class Measurement {
  // Odom measurement
  private final Pose2d m_pose;
  private final Pose2d m_poseSigmas; // TODO: Actually use this

  // Vision measurements
  private final ArrayList<Integer> m_cameraIDs;
  private final ArrayList<Integer> m_targetIDs;
  // Fiducial corner ID is valid only for AprilTags. -1 denotes the center. IDs are numbered as
  // follows:
  // * -> +X  3 ----- 2
  // * |      |       |
  // * V      |       |
  // * +Y     0 ----- 1
  private final ArrayList<Integer> m_fiducialCornerIDs;
  private final ArrayList<TargetCorner> m_corners;
  // Default to a moderate uncertainty.
  private double m_pixelSigma = 50.0;

  /**
   * Constructs a new Measurement object with the specified pose.
   *
   * @param pose The initial pose of the measurement.
   */
  public Measurement(final Pose2d pose) {
    m_pose = pose;
    m_poseSigmas = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)); // TODO: Don't hardcode
    m_cameraIDs = new ArrayList<>();
    m_targetIDs = new ArrayList<>();
    m_fiducialCornerIDs = new ArrayList<>();
    m_corners = new ArrayList<>();
  }

  /**
   * Adds a vision measurement to the system.
   *
   * @param cameraID The ID of the camera that captured the measurement.
   * @param targetID The ID of the target being measured.
   * @param pixelXY The pixel coordinates of the target corner in the image.
   */
  public void addVisionMeasurement(
      final int cameraID, final int targetID, final TargetCorner pixelXY) {
    addVisionMeasurement(cameraID, targetID, -1, pixelXY);
  }

  /**
   * Adds a vision measurement to the localization system.
   *
   * @param cameraID The ID of the camera that captured the measurement.
   * @param targetID The ID of the target being measured.
   * @param fiducialCornerID The ID of the fiducial corner being measured.
   * @param pixelXY The pixel coordinates of the target corner in the image.
   */
  public void addVisionMeasurement(
      final int cameraID,
      final int targetID,
      final int fiducialCornerID,
      final TargetCorner pixelXY) {
    m_cameraIDs.add(cameraID);
    m_targetIDs.add(targetID);
    m_fiducialCornerIDs.add(fiducialCornerID);
    m_corners.add(pixelXY);
  }

  /**
   * Sets the uncertainty of the vision measurement.
   *
   * @param pixelSigma The standard deviation of the pixel measurement noise.
   */
  public void setVisionUncertainty(final double pixelSigma) {
    m_pixelSigma = pixelSigma;
  }

  /**
   * Converts the measurement data to an array of doubles. Necessary for sending arbitrary-lengthed
   * data over NetworkTables.
   *
   * @param id The identifier for the measurement.
   * @return A double array containing the measurement data. The array consists of static data
   *     followed by vision data for each camera. The structure of the array is as follows: -
   *     data[0]: Measurement ID - data[1]: X coordinate of the pose - data[2]: Y coordinate of the
   *     pose - data[3]: Rotation in radians of the pose - data[4]: X coordinate sigma of the pose -
   *     data[5]: Y coordinate sigma of the pose - data[6]: Rotation sigma in radians of the pose -
   *     data[7]: Pixel sigma - For each camera: - data[8 + 5 * i]: Camera ID - data[9 + 5 * i]:
   *     Target ID - data[10 + 5 * i]: Fiducial corner ID - data[11 + 5 * i]: X coordinate of the
   *     corner - data[12 + 5 * i]: Y coordinate of the corner
   */
  public double[] toArray(final int id) {
    final int kStaticDataLength = 8;
    final int kVisionDataLength = 5;
    final double[] data = new double[kStaticDataLength + kVisionDataLength * m_cameraIDs.size()];
    data[0] = (double) id;
    data[1] = m_pose.getX();
    data[2] = m_pose.getY();
    data[3] = m_pose.getRotation().getRadians();
    data[4] = m_poseSigmas.getX();
    data[5] = m_poseSigmas.getY();
    data[6] = m_poseSigmas.getRotation().getRadians();
    data[7] = m_pixelSigma;
    for (int i = 0; i < m_cameraIDs.size(); i++) {
      data[kStaticDataLength + kVisionDataLength * i] = (double) m_cameraIDs.get(i);
      data[kStaticDataLength + kVisionDataLength * i + 1] = (double) m_targetIDs.get(i);
      data[kStaticDataLength + kVisionDataLength * i + 2] = (double) m_fiducialCornerIDs.get(i);
      data[kStaticDataLength + kVisionDataLength * i + 3] = m_corners.get(i).x;
      data[kStaticDataLength + kVisionDataLength * i + 4] = m_corners.get(i).y;
    }
    return data;
  }
}
