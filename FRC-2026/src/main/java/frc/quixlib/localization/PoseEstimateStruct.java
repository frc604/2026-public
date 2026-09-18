package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

/**
 * PoseEstimateStruct is a class that implements the Struct interface for the PoseEstimate type. It
 * provides methods to get the type class, type name, size, schema, and nested structures. It also
 * provides methods to pack and unpack PoseEstimate objects to and from ByteBuffers.
 *
 * <p>The PoseEstimateStruct class is immutable and ensures that the PoseEstimate objects are
 * correctly serialized and deserialized with the appropriate schema.
 *
 * <p>Methods:
 *
 * <ul>
 *   <li>{@link #getTypeClass()} - Returns the class type of PoseEstimate.
 *   <li>{@link #getTypeName()} - Returns the name of the type as a string.
 *   <li>{@link #getSize()} - Returns the size of the PoseEstimate structure in bytes.
 *   <li>{@link #getSchema()} - Returns the schema of the PoseEstimate structure as a string.
 *   <li>{@link #getNested()} - Returns an array of nested Structs within the PoseEstimate
 *       structure.
 *   <li>{@link #unpack(ByteBuffer)} - Unpacks a ByteBuffer into a PoseEstimate object.
 *   <li>{@link #pack(ByteBuffer, PoseEstimate)} - Packs a PoseEstimate object into a ByteBuffer.
 *   <li>{@link #isImmutable()} - Returns true indicating that the structure is immutable.
 * </ul>
 */
public class PoseEstimateStruct implements Struct<PoseEstimate> {
  @Override
  public Class<PoseEstimate> getTypeClass() {
    return PoseEstimate.class;
  }

  @Override
  public String getTypeName() {
    return "PoseEstimate";
  }

  @Override
  public int getSize() {
    return Pose2d.struct.getSize() + Struct.kSizeInt32 * 2;
  }

  @Override
  public String getSchema() {
    return "int32 id;" + "Pose2d pose;" + "int32 hasVision;";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose2d.struct};
  }

  @Override
  public PoseEstimate unpack(ByteBuffer bb) {
    final int id = bb.getInt();
    final Pose2d pose = Pose2d.struct.unpack(bb);
    final boolean hasVision = bb.getInt() != 0;
    return new PoseEstimate(id, pose, hasVision);
  }

  @Override
  public void pack(ByteBuffer bb, PoseEstimate value) {
    bb.putInt(value.getID());
    Pose2d.struct.pack(bb, value.getPose());
    bb.putInt(value.hasVision() ? 1 : 0);
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
