package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.struct.StructBuffer;
import frc.quixlib.devices.QuixIO;
import frc.quixlib.devices.QuixIOUtil;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.QuixVisionCamera;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * The NTManager class is responsible for managing network table interactions related to robot
 * localization. It handles publishing and subscribing to various topics such as measurements,
 * targets, and camera information. It also maintains the latest pose estimate and updates inputs
 * accordingly.
 */
public class NTManager implements QuixIO {
  private final NetworkTable m_localizerTable;
  // Combined Robot to DS odometry and vision measurements
  private final DoubleArrayPublisher m_measurementsPub;
  // Robot to DS targets
  private final StructArrayPublisher<Fiducial> m_targetPub;
  // Robot to Camera transforms and intrinsics
  private final StructArrayPublisher<CameraInfo> m_cameraInfosPublisher;

  // Use an AtomicReference to make updating the value thread-safe
  private final AtomicReference<PoseEstimate> m_latestPoseEstimate =
      new AtomicReference<PoseEstimate>();

  private final PoseEstimateInputsAutoLogged m_inputs = new PoseEstimateInputsAutoLogged();

  @AutoLog
  public static class PoseEstimateInputs {
    protected int id = 0;
    protected Pose2d pose = new Pose2d();
    protected boolean hasVision = false;
  }

  /**
   * Manages the NetworkTables (NT) interactions for localization. This class sets up publishers and
   * subscribers for various localization-related topics.
   *
   * <p>It initializes the following publishers:
   *
   * <ul>
   *   <li>Measurements publisher for double array topic "measurements"
   *   <li>Targets publisher for struct array topic "targets" with Fiducial structure
   *   <li>Cameras publisher for struct array topic "cameras" with CameraInfo structure
   * </ul>
   *
   * <p>It also sets up a listener for the "estimates" topic to update the latest pose estimate.
   *
   * <p>Usage:
   *
   * <pre>{@code
   * NTManager ntManager = new NTManager();
   * }</pre>
   */
  public NTManager() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_localizerTable = inst.getTable("localizer");
    m_measurementsPub =
        m_localizerTable.getDoubleArrayTopic("measurements").publish(PubSubOption.sendAll(true));
    m_targetPub =
        m_localizerTable
            .getStructArrayTopic("targets", Fiducial.struct)
            .publish(PubSubOption.sendAll(true));
    m_cameraInfosPublisher =
        m_localizerTable
            .getStructArrayTopic("cameras", CameraInfo.struct)
            .publish(PubSubOption.sendAll(true));

    // Setup listener for when the estimate is updated.
    final var estimatesSub =
        m_localizerTable
            .getStructTopic("estimates", PoseEstimate.struct)
            .subscribe(new PoseEstimate(), PubSubOption.sendAll(true));
    final StructBuffer<PoseEstimate> poseEstimateStructBuffer =
        StructBuffer.create(PoseEstimate.struct);
    inst.addListener(
        estimatesSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          m_latestPoseEstimate.set(poseEstimateStructBuffer.read(event.valueData.value.getRaw()));
        });

    QuixIOUtil.registerDevice(this);
  }

  /**
   * Publishes a measurement to the network table.
   *
   * @param measurement The measurement to be published.
   * @param id The identifier for the measurement.
   */
  public void publishMeasurement(final Measurement measurement, final int id) {
    m_measurementsPub.set(measurement.toArray(id));
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Publishes an array of fiducial targets to the network table.
   *
   * @param targets An array of Fiducial objects representing the targets to be published.
   */
  public void publishTargets(final Fiducial[] targets) {
    m_targetPub.set(targets);
  }

  /**
   * Publishes information about a list of cameras to the network table.
   *
   * @param cameras An ArrayList of QuixVisionCamera objects representing the cameras to be
   *     published. Each camera's transform, camera matrix, and distortion coefficients will be used
   *     to create a CameraInfo object, which will then be published.
   */
  public void publishCameras(final ArrayList<QuixVisionCamera> cameras) {
    final ArrayList<CameraInfo> infos = new ArrayList<>();
    for (var camera : cameras) {
      infos.add(
          new CameraInfo(camera.getTransform(), camera.getCameraMatrix(), camera.getDistCoeffs()));
    }
    CameraInfo[] array = new CameraInfo[infos.size()];
    infos.toArray(array);
    m_cameraInfosPublisher.set(array);
  }

  /**
   * Updates the input values with the latest pose estimate if available.
   *
   * <p>This method retrieves the latest pose estimate and updates the input values with the ID,
   * pose, and vision status from the estimate. If the latest pose estimate is null, the input
   * values are not updated. The updated inputs are then processed by the Logger.
   */
  public void updateInputs() {
    final var latestEstimate = m_latestPoseEstimate.get();
    if (latestEstimate != null) {
      m_inputs.id = latestEstimate.getID();
      m_inputs.pose = latestEstimate.getPose();
      m_inputs.hasVision = latestEstimate.hasVision();
    }
    Logger.processInputs("Inputs/NTManager", m_inputs);
  }

  /** Get the latest estimate over NT. */
  /**
   * Retrieves the latest pose estimate.
   *
   * @return A {@link PoseEstimate} object containing the latest pose information.
   */
  public PoseEstimate getLatestPoseEstimate() {
    return new PoseEstimate(m_inputs.id, m_inputs.pose, m_inputs.hasVision);
  }
}
