package frc.quixlib.math;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import org.junit.jupiter.api.*;

public class MathUtilsTest {
  static final double kTol = 1e-6;

  @Test
  public void cart2pol2carTest() {
    var pol = MathUtils.cart2pol(0.0, 0.0);
    var xy = MathUtils.pol2cart(pol.getFirst(), pol.getSecond());
    assertEquals(0.0, xy.getFirst(), kTol);
    assertEquals(0.0, xy.getSecond(), kTol);

    pol = MathUtils.cart2pol(1.23, -4.56);
    xy = MathUtils.pol2cart(pol.getFirst(), pol.getSecond());
    assertEquals(1.23, xy.getFirst(), kTol);
    assertEquals(-4.56, xy.getSecond(), kTol);
  }

  @Test
  public void constrainAngleNegPiToPiTest() {
    // Zero
    assertEquals(0.0, MathUtils.constrainAngleNegPiToPi(0.0), kTol);

    // Bounds
    assertEquals(-Math.PI, MathUtils.constrainAngleNegPiToPi(-Math.PI), kTol);
    assertEquals(-Math.PI, MathUtils.constrainAngleNegPiToPi(Math.PI), kTol);
    assertEquals(Math.PI - 1e-6, MathUtils.constrainAngleNegPiToPi(Math.PI - 1e-6), kTol);

    // > PI
    assertEquals(0.5, MathUtils.constrainAngleNegPiToPi(10.0 * Math.PI + 0.5), kTol);
    assertEquals(0.5, MathUtils.constrainAngleNegPiToPi(-10.0 * Math.PI + 0.5), kTol);
  }

  @Test
  public void unwrapAngleTest() {
    assertEquals(0, MathUtils.placeInScope(0, 0), kTol);
    assertEquals(0.5, MathUtils.placeInScope(0.5, 0), kTol);
    assertEquals(-0.5, MathUtils.placeInScope(-0.5, 0), kTol);

    assertEquals(Math.toRadians(720), MathUtils.placeInScope(0, Math.toRadians(800)), kTol);
    assertEquals(Math.toRadians(720), MathUtils.placeInScope(0, Math.toRadians(700)), kTol);
    assertEquals(Math.toRadians(-720), MathUtils.placeInScope(0, Math.toRadians(-800)), kTol);
    assertEquals(Math.toRadians(-720), MathUtils.placeInScope(0, Math.toRadians(-700)), kTol);

    assertEquals(
        Math.toRadians(765), MathUtils.placeInScope(Math.toRadians(45), Math.toRadians(800)), kTol);
    assertEquals(
        Math.toRadians(765), MathUtils.placeInScope(Math.toRadians(45), Math.toRadians(700)), kTol);

    assertEquals(
        Math.toRadians(675),
        MathUtils.placeInScope(-Math.toRadians(45), Math.toRadians(800)),
        kTol);
    assertEquals(
        Math.toRadians(675),
        MathUtils.placeInScope(-Math.toRadians(45), Math.toRadians(700)),
        kTol);

    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(360), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(-360), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(720), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(-720), Math.toRadians(300)),
        kTol);
  }

  @Test
  public void getVelocityAtOffsetIdentityTest() {
    // Zero offset should return the same velocity.
    final Pose2d basePose = new Pose2d(1, 2, new Rotation2d(0.5));
    final Transform2d offset = new Transform2d();
    final Twist2d velocity = new Twist2d(3, 4, 1.0);

    final Twist2d result = MathUtils.getVelocityAtOffset(basePose, offset, velocity);

    assertEquals(3.0, result.dx, kTol);
    assertEquals(4.0, result.dy, kTol);
    assertEquals(1.0, result.dtheta, kTol);
  }

  @Test
  public void getVelocityAtOffsetTest() {
    // Robot at origin facing +X, spinning at 1 rad/s CCW, with a point 1m in front.
    // The point should have velocity (0, 1) m/s (moving left as robot spins CCW).
    final Pose2d basePose = new Pose2d(0, 0, new Rotation2d(0));
    final Transform2d offset =
        new Transform2d(new Translation2d(1, 0), new Rotation2d()); // 1m in front
    final Twist2d velocity = new Twist2d(0, 0, 1.0); // 1 rad/s CCW

    final Twist2d result = MathUtils.getVelocityAtOffset(basePose, offset, velocity);

    assertEquals(0.0, result.dx, kTol);
    assertEquals(1.0, result.dy, kTol);
    assertEquals(1.0, result.dtheta, kTol);
  }

  @Test
  public void getVelocityAtOffsetRotatedBaseTest() {
    // Robot at origin facing +Y (90 deg CCW), spinning at 1 rad/s CCW, with a point 1m in front.
    // Offset (1, 0) in robot frame becomes (0, 1) in world frame.
    // ω × r = 1 × (0, 1) = (-1, 0), so the point moves in -X direction.
    final Pose2d basePose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
    final Transform2d offset =
        new Transform2d(new Translation2d(1, 0), new Rotation2d()); // 1m in front
    final Twist2d velocity = new Twist2d(0, 0, 1.0); // 1 rad/s CCW

    final Twist2d result = MathUtils.getVelocityAtOffset(basePose, offset, velocity);

    assertEquals(-1.0, result.dx, kTol);
    assertEquals(0.0, result.dy, kTol);
    assertEquals(1.0, result.dtheta, kTol);
  }

  @Test
  public void getVelocityAccelAtOffsetTest() {
    final double step = 1.0;
    final double linearStepMax = 10.0;
    final double angleStep = Math.PI / 4;
    final double angleStepMax = 2 * Math.PI;
    final double x = 1.0;
    final double y = 1.0;
    final double kTol = 1e-6;
    for (double theta = 0; theta < angleStepMax; theta += angleStep) {
      for (double velX = 0; velX < linearStepMax; velX += step) {
        for (double velY = 0; velY < linearStepMax; velY += step) {
          for (double omega = 0; omega < angleStepMax; omega += angleStep) {
            for (double accX = 0; accX < linearStepMax; accX += step) {
              for (double accY = 0; accY < linearStepMax; accY += step) {
                for (double alpha = 0; alpha < angleStepMax; alpha += angleStep) {
                  // Calculate values based on MathUtils methods
                  final Pose2d basePose = new Pose2d(0, 0, new Rotation2d(theta));
                  final Transform2d offset =
                      new Transform2d(new Translation2d(x, y), new Rotation2d());
                  final Twist2d velocity = new Twist2d(velX, velY, omega);
                  final Twist2d acceleration = new Twist2d(accX, accY, alpha);

                  final Twist2d velResult =
                      MathUtils.getVelocityAtOffset(basePose, offset, velocity);
                  final Twist2d accResult =
                      MathUtils.getAccelAtOffset(basePose, offset, velocity, acceleration);

                  // Expected values using alternate math
                  final double cos_t = Math.cos(theta);
                  final double sin_t = Math.sin(theta);

                  final double rx = x * cos_t - y * sin_t;
                  final double ry = x * sin_t + y * cos_t;

                  // Linear Velocity (dxB/dt, dyB/dt)
                  final double vxb = velX - omega * ry;
                  final double vyb = velY + omega * rx;

                  // Linear Acceleration (d2xB/dt^2, d2yB/dt^2)
                  final double axb = accX - alpha * ry - (omega * omega) * rx;
                  final double ayb = accY + alpha * rx - (omega * omega) * ry;

                  // Rotational states (dθB/dt, d2θB/dt^2)
                  final double omega_b = omega;
                  final double alpha_b = alpha;

                  // Assert velocity matches
                  assertEquals(vxb, velResult.dx, kTol);
                  assertEquals(vyb, velResult.dy, kTol);
                  assertEquals(omega_b, velResult.dtheta, kTol);

                  // Assert acceleration matches
                  assertEquals(axb, accResult.dx, kTol);
                  assertEquals(ayb, accResult.dy, kTol);
                  assertEquals(alpha_b, accResult.dtheta, kTol);
                }
              }
            }
          }
        }
      }
    }
  }
}
