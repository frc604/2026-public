package frc.quixlib.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A data class for a pipeline packet. */
public class PipelineVisionPacket {
  private final boolean m_hasTargets;
  private final PhotonTrackedTarget m_bestTarget;
  private final List<PhotonTrackedTarget> m_targets;
  private final Optional<MultiTargetPNPResult> m_multitagResult;
  private final double m_captureTimestamp;

  public PipelineVisionPacket(
      final boolean hasTargets,
      final PhotonTrackedTarget bestTarget,
      final List<PhotonTrackedTarget> targets,
      final Optional<MultiTargetPNPResult> multitagResult,
      final double captureTimestamp) {
    m_hasTargets = hasTargets;
    m_bestTarget = bestTarget;
    m_targets = targets;
    m_multitagResult = multitagResult;
    m_captureTimestamp = captureTimestamp;
  }

  /**
   * If the vision packet has valid targets.
   *
   * @return if targets are found.
   */
  public boolean hasTargets() {
    return m_hasTargets;
  }

  /**
   * Gets best target.
   *
   * @return the best target.
   */
  public PhotonTrackedTarget getBestTarget() {
    return m_bestTarget;
  }

  /**
   * Gets targets.
   *
   * @return the targets.
   */
  public List<PhotonTrackedTarget> getTargets() {
    return m_targets;
  }

  /**
   * Gets the multitag result (if present).
   *
   * @return the multitag result.
   */
  public Optional<MultiTargetPNPResult> getMultitagResult() {
    return m_multitagResult;
  }

  /**
   * Gets the capture timestamp in seconds. -1 if the result has no timestamp set.
   *
   * @return the timestamp in seconds.
   */
  public double getCaptureTimestamp() {
    return m_captureTimestamp;
  }
}
