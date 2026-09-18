package frc.quixlib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

public class SwerveDrivePhysicsSim extends Body {
  public SwerveDrivePhysicsSim() {
    if (Robot.isReal()) {
      return;
    }

    final double kRobotWidth = Units.inchesToMeters(36.0);
    final double kRobotLength = Units.inchesToMeters(36.0);
    final double kRobotMass = 60.0; // kg
    final double density = kRobotMass / (kRobotWidth * kRobotLength);

    super.addFixture(Geometry.createRectangle(kRobotWidth, kRobotLength), density, 0.1, 0.2);
    super.setMass(MassType.NORMAL);
    super.setLinearDamping(1.6);
    super.setAngularDamping(1);
  }

  public void setPose(Pose2d pose) {
    super.transform.setTranslation(pose.getX(), pose.getY());
    super.transform.setRotation(pose.getRotation().getRadians());
  }

  public Pose2d getPose() {
    return new Pose2d(
        super.transform.getTranslationX(),
        super.transform.getTranslationY(),
        new Rotation2d(super.transform.getRotation().toRadians()));
  }

  public void setVelocity(Transform2d vel) {
    super.setAtRest(false);
    super.setLinearVelocity(vel.getX(), vel.getY());
    super.setAngularVelocity(vel.getRotation().getRadians());
  }

  public Transform2d getVelocity() {
    return new Transform2d(
        super.linearVelocity.x, super.linearVelocity.y, new Rotation2d(super.angularVelocity));
  }
}
