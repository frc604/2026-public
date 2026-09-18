package frc.quixlib.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.Optional;

/**
 * The CameraInfo class holds information about a camera's transform, camera matrix, and distortion
 * coefficients.
 *
 * <p>This class is used to encapsulate the camera's intrinsic and extrinsic parameters, which are
 * essential for camera calibration and image processing tasks in robotics.
 *
 * <p>It contains the following fields:
 *
 * <ul>
 *   <li>{@code m_transform} - The 3D transform representing the camera's position and orientation.
 *   <li>{@code m_cameraMatrix} - An optional 3x3 matrix representing the camera's intrinsic
 *       parameters.
 *   <li>{@code m_distCoeffs} - An optional 8x1 matrix representing the camera's distortion
 *       coefficients.
 * </ul>
 *
 * <p>The class provides getter methods to access these fields:
 *
 * <ul>
 *   <li>{@link #getTransform()} - Returns the camera's transform.
 *   <li>{@link #getCameraMatrix()} - Returns the camera's intrinsic matrix, if available.
 *   <li>{@link #getDistCoeffs()} - Returns the camera's distortion coefficients, if available.
 * </ul>
 *
 * <p>Additionally, it includes a static nested class {@code CameraInfoStruct} for serialization
 * purposes.
 *
 * @param transform The 3D transform representing the camera's position and orientation.
 * @param cameraMatrix An optional 3x3 matrix representing the camera's intrinsic parameters.
 * @param distCoeffs An optional 8x1 matrix representing the camera's distortion coefficients.
 */
public class CameraInfo {
  final Transform3d m_transform;
  final Optional<Matrix<N3, N3>> m_cameraMatrix;
  final Optional<Matrix<N8, N1>> m_distCoeffs;

  public CameraInfo(
      final Transform3d transform,
      final Optional<Matrix<N3, N3>> cameraMatrix,
      final Optional<Matrix<N8, N1>> distCoeffs) {
    m_transform = transform;
    m_cameraMatrix = cameraMatrix;
    m_distCoeffs = distCoeffs;
  }

  public Transform3d getTransform() {
    return m_transform;
  }

  public Optional<Matrix<N3, N3>> getCameraMatrix() {
    return m_cameraMatrix;
  }

  public Optional<Matrix<N8, N1>> getDistCoeffs() {
    return m_distCoeffs;
  }

  // Struct for serialization.
  public static final CameraInfoStruct struct = new CameraInfoStruct();
}
