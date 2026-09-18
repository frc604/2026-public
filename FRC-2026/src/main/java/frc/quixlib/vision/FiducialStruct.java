package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import frc.quixlib.vision.Fiducial.Type;
import java.nio.ByteBuffer;

public class FiducialStruct implements Struct<Fiducial> {
  @Override
  public Class<Fiducial> getTypeClass() {
    return Fiducial.class;
  }

  @Override
  public String getTypeName() {
    return "Fiducial";
  }

  @Override
  public int getSize() {
    return Pose3d.struct.getSize() + Struct.kSizeInt32 * 2 + Struct.kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "int32 type;int32 id;Pose3d pose;double size;";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose3d.struct};
  }

  @Override
  public Fiducial unpack(ByteBuffer bb) {
    final Type type = Type.values()[bb.getInt()];
    final int id = bb.getInt();
    final Pose3d pose = Pose3d.struct.unpack(bb);
    final double size = bb.getDouble();
    return new Fiducial(type, id, pose, size);
  }

  @Override
  public void pack(ByteBuffer bb, Fiducial value) {
    bb.putInt(value.getType().getValue());
    bb.putInt(value.id());
    Pose3d.struct.pack(bb, value.getPose());
    bb.putDouble(value.getSize());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
