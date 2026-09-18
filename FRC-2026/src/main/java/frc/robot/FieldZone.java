package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Represents a rectangular zone on the field. Handles automatic flipping of boundaries if the robot
 * is on the red alliance.
 */
public class FieldZone {
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(12 * 54 + 2.5975);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(12 * 26 + 5.6875);

  private final double blueMinX;
  private final double blueMaxX;
  private final double blueMinY;
  private final double blueMaxY;

  /**
   * Creates a rectangular field zone relative to the Blue Alliance origin.
   *
   * @param blueMinX Minimum X coordinate (closest to blue alliance wall)
   * @param blueMaxX Maximum X coordinate (closest to center line/red alliance wall)
   * @param blueMinY Minimum Y coordinate
   * @param blueMaxY Maximum Y coordinate
   */
  public FieldZone(double blueMinX, double blueMaxX, double blueMinY, double blueMaxY) {
    this.blueMinX = blueMinX;
    this.blueMaxX = blueMaxX;
    this.blueMinY = blueMinY;
    this.blueMaxY = blueMaxY;
  }

  public double getBlueMinX() {
    return blueMinX;
  }

  public double getBlueMaxX() {
    return blueMaxX;
  }

  public double getBlueMinY() {
    return blueMinY;
  }

  public double getBlueMaxY() {
    return blueMaxY;
  }

  /**
   * Creates a rectangular field zone from two corner translations relative to the Blue Alliance
   * origin.
   */
  public FieldZone(Translation2d blueBottomLeft, Translation2d blueTopRight) {
    this(blueBottomLeft.getX(), blueTopRight.getX(), blueBottomLeft.getY(), blueTopRight.getY());
  }

  /**
   * Queries whether a given pose (in standard Blue-Origin field frame) is within this zone.
   * Automatically adjusts the boundaries if the robot is on the Red alliance based on
   * DriverStation.
   *
   * @param robotPose The robot's current pose
   * @return true if the pose is inside the zone
   */
  public boolean contains(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    return contains(robotPose, isRed);
  }

  /**
   * Explicit override for checking against a specific alliance. Useful for simulation, autonomous
   * trajectory planning, or off-field testing.
   *
   * @param robotPose The robot's current pose
   * @param isRedAlliance true if we want to check against the red alliance flipped zone
   * @return true if the pose is inside the zone for the given alliance
   */
  public boolean contains(Pose2d robotPose, boolean isRedAlliance) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    if (isRedAlliance) {
      // Rotational Symmetry Flip (Standard for modern FRC games like 2024 Crescendo / 2025
      // Reefscape)
      // Flip X
      double redMinX = FIELD_LENGTH_METERS - blueMaxX;
      double redMaxX = FIELD_LENGTH_METERS - blueMinX;

      // Flip Y
      double redMinY = FIELD_WIDTH_METERS - blueMaxY;
      double redMaxY = FIELD_WIDTH_METERS - blueMinY;

      return x >= redMinX && x <= redMaxX && y >= redMinY && y <= redMaxY;

      /*
       * IF 2026 GAME USES AXIAL SYMMETRY (Y-axis never flips, like 2023 Charged Up):
       * return x >= redMinX && x <= redMaxX && y >= blueMinY && y <= blueMaxY;
       */
    } else {
      // Standard Blue check
      return x >= blueMinX && x <= blueMaxX && y >= blueMinY && y <= blueMaxY;
    }
  }

  /**
   * Returns an array of Pose2d representing the corners of the zone (for visual logging). It
   * returns 5 points specifically (BottomLeft, TopLeft, TopRight, BottomRight, BottomLeft) to draw
   * a closed rectangle using WPILib's Line plot.
   *
   * @param isRedAlliance whether to return the flipped coordinates for the Red alliance
   */
  public Pose2d[] getCornerPoses(boolean isRedAlliance) {
    double minX = isRedAlliance ? FIELD_LENGTH_METERS - blueMaxX : blueMinX;
    double maxX = isRedAlliance ? FIELD_LENGTH_METERS - blueMinX : blueMaxX;
    double minY = isRedAlliance ? FIELD_WIDTH_METERS - blueMaxY : blueMinY;
    double maxY = isRedAlliance ? FIELD_WIDTH_METERS - blueMinY : blueMaxY;

    // Ordered to draw a continuous rectangle
    return new Pose2d[] {
      new Pose2d(minX, minY, Rotation2d.kZero),
      new Pose2d(minX, maxY, Rotation2d.kZero),
      new Pose2d(maxX, maxY, Rotation2d.kZero),
      new Pose2d(maxX, minY, Rotation2d.kZero),
      new Pose2d(minX, minY, Rotation2d.kZero)
    };
  }
}
