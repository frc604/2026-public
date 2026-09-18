package frc.quixlib.localization;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.Optional;

/**
 * CameraInfoStruct is a structure that implements the Struct interface for the CameraInfo class. It
 * provides methods to get the type class, type name, size, schema, nested structures, and to pack
 * and unpack CameraInfo objects from a ByteBuffer.
 *
 * <p>The structure includes the following fields:
 *
 * <ul>
 *   <li>Transform3d transform
 *   <li>int32 hasCameraMatrix
 *   <li>double M00, M01, M02, M10, M11, M12, M20, M21, M22
 *   <li>int32 hasDistCoeffs
 *   <li>double d0, d1, d2, d3, d4, d5, d6, d7
 * </ul>
 *
 * <p>The unpack method reads data from a ByteBuffer and constructs a CameraInfo object. The pack
 * method writes a CameraInfo object to a ByteBuffer.
 *
 * <p>The isImmutable method indicates that the structure is immutable.
 *
 * <p>Used by NetworkTables to efficiently transmit camera calibration data between robot and driver
 * station.
 *
 * @see ByteBuffer
 * @see CameraInfo
 * @see Transform3d
 * @see Struct
 */
public class CameraInfoStruct implements Struct<CameraInfo> {
  @Override
  public Class<CameraInfo> getTypeClass() {
    return CameraInfo.class;
  }

  @Override
  public String getTypeName() {
    return "CameraInfo";
  }

  @Override
  public int getSize() {
    return Transform3d.struct.getSize() + Struct.kSizeInt32 * 2 + Struct.kSizeDouble * 17;
  }

  @Override
  public String getSchema() {
    return "Transform3d transform;"
        + "int32 hasCameraMatrix;"
        + "double M00;"
        + "double M01;"
        + "double M02;"
        + "double M10;"
        + "double M11;"
        + "double M12;"
        + "double M20;"
        + "double M21;"
        + "double M22;"
        + "int32 hasDistCoeffs;"
        + "double d0;"
        + "double d1;"
        + "double d2;"
        + "double d3;"
        + "double d4;"
        + "double d5;"
        + "double d6;"
        + "double d7;";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Transform3d.struct};
  }

  /**
   * Unpacks camera information from a ByteBuffer into a CameraInfo object.
   *
   * @param bb The ByteBuffer containing the packed camera information data
   * @return A new CameraInfo object containing the unpacked camera transformation, optional camera
   *     matrix, and optional distortion coefficients
   *     <p>The ByteBuffer should contain, in order: - A Transform3d struct - A boolean flag for
   *     camera matrix presence - Nine doubles representing a 3x3 camera matrix (if present) - A
   *     boolean flag for distortion coefficients presence - Eight doubles representing distortion
   *     coefficients (if present)
   */
  @Override
  public CameraInfo unpack(ByteBuffer bb) {
    final Transform3d transform = Transform3d.struct.unpack(bb);
    final boolean hasCameraMatrix = bb.getInt() != 0;
    final double M00 = bb.getDouble();
    final double M01 = bb.getDouble();
    final double M02 = bb.getDouble();
    final double M10 = bb.getDouble();
    final double M11 = bb.getDouble();
    final double M12 = bb.getDouble();
    final double M20 = bb.getDouble();
    final double M21 = bb.getDouble();
    final double M22 = bb.getDouble();
    final boolean hasDistCoeffs = bb.getInt() != 0;
    final double d0 = bb.getDouble();
    final double d1 = bb.getDouble();
    final double d2 = bb.getDouble();
    final double d3 = bb.getDouble();
    final double d4 = bb.getDouble();
    final double d5 = bb.getDouble();
    final double d6 = bb.getDouble();
    final double d7 = bb.getDouble();
    return new CameraInfo(
        transform,
        hasCameraMatrix
            ? Optional.of(
                MatBuilder.fill(Nat.N3(), Nat.N3(), M00, M01, M02, M10, M11, M12, M20, M21, M22))
            : Optional.empty(),
        hasDistCoeffs
            ? Optional.of(MatBuilder.fill(Nat.N8(), Nat.N1(), d0, d1, d2, d3, d4, d5, d6, d7))
            : Optional.empty());
  }

  /**
   * Packs camera information into a ByteBuffer. This method serializes a CameraInfo object,
   * including its transformation matrix, camera matrix (if present), and distortion coefficients
   * (if present).
   *
   * @param bb The ByteBuffer to pack the data into
   * @param value The CameraInfo object containing the data to be packed
   */
  @Override
  public void pack(ByteBuffer bb, CameraInfo value) {
    Transform3d.struct.pack(bb, value.getTransform());
    boolean hasCameraMatrix = value.getCameraMatrix().isPresent();
    bb.putInt(hasCameraMatrix ? 1 : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(0, 0) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(0, 1) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(0, 2) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(1, 0) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(1, 1) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(1, 2) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(2, 0) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(2, 1) : 0);
    bb.putDouble(hasCameraMatrix ? value.getCameraMatrix().get().get(2, 2) : 0);
    boolean hasDistCoeffs = value.getDistCoeffs().isPresent();
    bb.putInt(hasDistCoeffs ? 1 : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(0, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(1, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(2, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(3, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(4, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(5, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(6, 0) : 0);
    bb.putDouble(hasDistCoeffs ? value.getDistCoeffs().get().get(7, 0) : 0);
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
