package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.quixlib.vision.Fiducial;

public class Fiducials {
  // Page 11: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dwg-complete.pdf
  // FIRST California uses welded perimeter

  private static final double aprilTagSize = Units.inchesToMeters(6.5); // m

  public static final Fiducial[] aprilTagFiducials =
      new Fiducial[] {
        new Fiducial(
            Fiducial.Type.APRILTAG,
            1,
            new Pose3d(
                Units.inchesToMeters(467.64),
                Units.inchesToMeters(292.31),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            2,
            new Pose3d(
                Units.inchesToMeters(469.11),
                Units.inchesToMeters(182.60),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            3,
            new Pose3d(
                Units.inchesToMeters(445.35),
                Units.inchesToMeters(172.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            4,
            new Pose3d(
                Units.inchesToMeters(445.35),
                Units.inchesToMeters(158.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            5,
            new Pose3d(
                Units.inchesToMeters(469.11),
                Units.inchesToMeters(135.09),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            6,
            new Pose3d(
                Units.inchesToMeters(467.64),
                Units.inchesToMeters(25.37),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            7,
            new Pose3d(
                Units.inchesToMeters(470.59),
                Units.inchesToMeters(25.37),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            8,
            new Pose3d(
                Units.inchesToMeters(483.11),
                Units.inchesToMeters(135.09),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            9,
            new Pose3d(
                Units.inchesToMeters(492.88),
                Units.inchesToMeters(144.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            10,
            new Pose3d(
                Units.inchesToMeters(492.88),
                Units.inchesToMeters(158.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            11,
            new Pose3d(
                Units.inchesToMeters(483.11),
                Units.inchesToMeters(182.60),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            12,
            new Pose3d(
                Units.inchesToMeters(470.59),
                Units.inchesToMeters(292.31),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            13,
            new Pose3d(
                Units.inchesToMeters(650.92),
                Units.inchesToMeters(291.47),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            14,
            new Pose3d(
                Units.inchesToMeters(650.92),
                Units.inchesToMeters(274.47),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            15,
            new Pose3d(
                Units.inchesToMeters(650.90),
                Units.inchesToMeters(170.22),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            16,
            new Pose3d(
                Units.inchesToMeters(650.90),
                Units.inchesToMeters(153.22),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            17,
            new Pose3d(
                Units.inchesToMeters(183.59),
                Units.inchesToMeters(25.37),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            18,
            new Pose3d(
                Units.inchesToMeters(182.11),
                Units.inchesToMeters(135.09),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            19,
            new Pose3d(
                Units.inchesToMeters(205.87),
                Units.inchesToMeters(144.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            20,
            new Pose3d(
                Units.inchesToMeters(205.87),
                Units.inchesToMeters(158.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            21,
            new Pose3d(
                Units.inchesToMeters(182.11),
                Units.inchesToMeters(182.60),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            22,
            new Pose3d(
                Units.inchesToMeters(183.59),
                Units.inchesToMeters(292.31),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            23,
            new Pose3d(
                Units.inchesToMeters(180.64),
                Units.inchesToMeters(292.31),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            24,
            new Pose3d(
                Units.inchesToMeters(168.11),
                Units.inchesToMeters(182.60),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            25,
            new Pose3d(
                Units.inchesToMeters(158.34),
                Units.inchesToMeters(172.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            26,
            new Pose3d(
                Units.inchesToMeters(158.34),
                Units.inchesToMeters(158.84),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            27,
            new Pose3d(
                Units.inchesToMeters(168.11),
                Units.inchesToMeters(135.09),
                Units.inchesToMeters(44.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            28,
            new Pose3d(
                Units.inchesToMeters(180.64),
                Units.inchesToMeters(25.37),
                Units.inchesToMeters(35.00),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            29,
            new Pose3d(
                Units.inchesToMeters(0.30),
                Units.inchesToMeters(26.22),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            30,
            new Pose3d(
                Units.inchesToMeters(0.30),
                Units.inchesToMeters(43.22),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            31,
            new Pose3d(
                Units.inchesToMeters(0.32),
                Units.inchesToMeters(147.47),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            32,
            new Pose3d(
                Units.inchesToMeters(0.32),
                Units.inchesToMeters(164.47),
                Units.inchesToMeters(21.75),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
      };
}
