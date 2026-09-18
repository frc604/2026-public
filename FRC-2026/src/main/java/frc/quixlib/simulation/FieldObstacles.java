package frc.quixlib.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rotation;
import org.dyn4j.world.World;

/** Crescendo Field */
public class FieldObstacles {
  private static final double fieldX = Units.inchesToMeters(690.876);
  private static final double fieldY = Units.inchesToMeters(317.0);
  private static final Translation2d fieldCenter = new Translation2d(fieldX * 0.5, fieldY * 0.5);

  private final List<Body> m_obstacles = new ArrayList<>();

  public FieldObstacles() {
    if (Robot.isReal()) {
      return;
    }

    // Blue/Red alliance station wall
    addFieldWall(new Translation2d(0, 0), new Translation2d(0, fieldY), true);
    // Scoring table wall
    addFieldWall(new Translation2d(0, 0), new Translation2d(fieldX, 0), false);
    // Opposite scoring table wall
    addFieldWall(new Translation2d(0, fieldY), new Translation2d(fieldX, fieldY), false);

    // Blue/Red coral stations
    addFieldWall(
        new Translation2d(0.0, Units.inchesToMeters(268.0)),
        new Translation2d(Units.inchesToMeters(65.6), Units.inchesToMeters(314.9)),
        true);
    addFieldWall(
        new Translation2d(0.0, Units.inchesToMeters(49.0)),
        new Translation2d(Units.inchesToMeters(65.6), Units.inchesToMeters(2.1)),
        true);

    // Blue/Red Reef
    // Blue ID 17
    addFieldWall(
        new Translation2d(Units.inchesToMeters(144.3), Units.inchesToMeters(139.4)),
        new Translation2d(Units.inchesToMeters(176.4), Units.inchesToMeters(120.9)),
        true);
    // Blue ID 18
    addFieldWall(
        new Translation2d(Units.inchesToMeters(144.0), Units.inchesToMeters(177.0)),
        new Translation2d(Units.inchesToMeters(144.0), Units.inchesToMeters(140.0)),
        true);
    // Blue ID 19
    addFieldWall(
        new Translation2d(Units.inchesToMeters(176.4), Units.inchesToMeters(196.1)),
        new Translation2d(Units.inchesToMeters(144.3), Units.inchesToMeters(177.6)),
        true);
    // Blue ID 20
    addFieldWall(
        new Translation2d(Units.inchesToMeters(177.1), Units.inchesToMeters(196.1)),
        new Translation2d(Units.inchesToMeters(209.2), Units.inchesToMeters(177.6)),
        true);
    // Blue ID 21
    addFieldWall(
        new Translation2d(Units.inchesToMeters(209.5), Units.inchesToMeters(140.0)),
        new Translation2d(Units.inchesToMeters(209.5), Units.inchesToMeters(177.0)),
        true);
    // Blue ID 22
    addFieldWall(
        new Translation2d(Units.inchesToMeters(177.1), Units.inchesToMeters(120.9)),
        new Translation2d(Units.inchesToMeters(209.2), Units.inchesToMeters(139.4)),
        true);
  }

  private void addFieldWall(
      Translation2d start, Translation2d end, boolean addRotationallyMirrored) {
    Translation2d center = start.plus(end).div(2.0);
    Translation2d diff = end.minus(start);
    addRectangularObstacle(center, diff.getNorm(), 0.01, diff.getAngle().getRadians());
    if (addRotationallyMirrored) {
      addRectangularObstacle(
          center.rotateAround(fieldCenter, Rotation2d.kPi),
          diff.getNorm(),
          0.01,
          diff.getAngle().getRadians());
    }
  }

  private void addRectangularObstacle(
      Translation2d centerPosition, double width, double height, double rotationRadians) {
    final Body obstacle = new Body();
    obstacle.setMass(MassType.INFINITE);
    final BodyFixture fixture = obstacle.addFixture(Geometry.createRectangle(width, height));
    fixture.setFriction(0.1);
    fixture.setRestitution(0.2);
    obstacle.getTransform().setTranslation(centerPosition.getX(), centerPosition.getY());
    obstacle.getTransform().setRotation(new Rotation(rotationRadians));
    m_obstacles.add(obstacle);
  }

  public void addToField(World<Body> field) {
    for (Body obstacle : m_obstacles) {
      field.addBody(obstacle);
    }
  }
}
