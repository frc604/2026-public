package frc.quixlib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class LaunchCalculatorTest {
  private static final double kTol = 1e-9;
  private static final LaunchCalculator kLaunchCalculator =
      new LaunchCalculator("src/test/java/frc/quixlib/math/test_launch_table.csv");

  private void testDistanceM(
      double distanceM, double expectedVelocityMps, double expectedAngleDeg) {
    final LaunchParameter setpoint = kLaunchCalculator.interpolateLaunchEntry(distanceM);

    assertEquals(true, setpoint.inTableRange());
    assertEquals(expectedVelocityMps, setpoint.launchVelocityMps(), kTol);
    assertEquals(expectedAngleDeg, setpoint.launchAngleDeg(), kTol);
  }

  private void testOutOfRangeDistanceM(
      double distanceM, double expectedVelocityMps, double expectedAngleDeg) {
    final LaunchParameter setpoint = kLaunchCalculator.interpolateLaunchEntry(distanceM);

    assertEquals(false, setpoint.inTableRange());
    assertEquals(expectedVelocityMps, setpoint.launchVelocityMps(), kTol);
    assertEquals(expectedAngleDeg, setpoint.launchAngleDeg(), kTol);
  }

  @Test
  public void calculateShotUsesExactDistanceRow() {
    kLaunchCalculator.reloadLaunchTable();

    // Exact table distances should map directly to that row (no interpolation drift).
    testDistanceM(1.0, 5.554, 74.962);
    testDistanceM(6.0, 8.466, 55.832);
    testDistanceM(8.0, 9.634, 56.000);
  }

  @Test
  public void calculateShotInterpolatesBetweenDistanceRows() {
    kLaunchCalculator.reloadLaunchTable();

    // Inexact table distances should interpolate the values between the nearest rows.
    testDistanceM(1.5, (6.513 + 5.554) / 2.0, (73.781 + 74.962) / 2.0);
    testDistanceM(6.5, (9.025 + 8.466) / 2.0, (55.100 + 55.832) / 2.0);
    testDistanceM(7.5, (9.634 + 9.025) / 2.0, (56.000 + 55.100) / 2.0);
  }

  @Test
  public void calculateShotInterpolationOutOfRange() {
    kLaunchCalculator.reloadLaunchTable();

    // Out of range values should result in the nearest values in the table.
    testOutOfRangeDistanceM(0.0, 5.554, 74.962);
    testOutOfRangeDistanceM(0.5, 5.554, 74.962);
    testOutOfRangeDistanceM(10.0, 9.634, 56.000);
    testOutOfRangeDistanceM(11.0, 9.634, 56.000);
  }

  // --- fieldToRobotVelocity tests ---

  @Test
  public void fieldToRobotVelocityIdentityWhenNoTilt() {
    // With zero roll, pitch, and heading the output should equal the input.
    final double[] result = LaunchCalculator.fieldToRobotVelocity(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    assertEquals(1.0, result[0], kTol);
    assertEquals(2.0, result[1], kTol);
    assertEquals(3.0, result[2], kTol);
  }

  @Test
  public void fieldToRobotVelocityPurePitch() {
    // Robot pitched forward (nose down) by 90°. A purely upward field velocity should become a
    // purely backward robot velocity.
    final double pitch = Math.toRadians(90.0);
    final double[] result = LaunchCalculator.fieldToRobotVelocity(0.0, 0.0, 1.0, 0.0, 0.0, pitch);
    // Ry(-90°) maps (0,0,1) → (-1,0,0)
    assertEquals(-1.0, result[0], kTol);
    assertEquals(0.0, result[1], kTol);
    assertEquals(0.0, result[2], kTol);
  }

  @Test
  public void fieldToRobotVelocityPureRoll() {
    // Robot rolled right by 90°. A purely upward field velocity should become a purely leftward
    // robot velocity.
    final double roll = Math.toRadians(90.0);
    final double[] result = LaunchCalculator.fieldToRobotVelocity(0.0, 0.0, 1.0, 0.0, roll, 0.0);
    // Rx(-90°) maps (0,0,1) → (0,1,0) (leftward in robot frame)
    assertEquals(0.0, result[0], kTol);
    assertEquals(1.0, result[1], kTol);
    assertEquals(0.0, result[2], kTol);
  }

  @Test
  public void fieldToRobotVelocityPreservesMagnitude() {
    // Any rigid-body rotation must preserve the velocity magnitude.
    final double vx = 3.0, vy = 4.0, vz = 5.0;
    final double heading = Math.toRadians(37.0);
    final double roll = Math.toRadians(12.0);
    final double pitch = Math.toRadians(-8.0);
    final double[] result = LaunchCalculator.fieldToRobotVelocity(vx, vy, vz, heading, roll, pitch);
    final double magIn = Math.sqrt(vx * vx + vy * vy + vz * vz);
    final double magOut =
        Math.sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2]);
    assertEquals(magIn, magOut, kTol);
  }

  @Test
  public void fieldToRobotVelocityNonZeroHeading() {
    // With heading = 90° (robot facing +Y), zero tilt, a field (1,0,0) vector should rotate to
    // robot frame as (0,-1,0) in the intermediate step, but then gets re-rotated back to field
    // heading. With no tilt, the output should equal the input regardless of heading.
    final double heading = Math.toRadians(90.0);
    final double[] result = LaunchCalculator.fieldToRobotVelocity(1.0, 2.0, 3.0, heading, 0.0, 0.0);
    assertEquals(1.0, result[0], kTol);
    assertEquals(2.0, result[1], kTol);
    assertEquals(3.0, result[2], kTol);
  }

  @Test
  public void fieldToRobotVelocitySmallPitchShiftsVertical() {
    // A small forward pitch should shift some of the vertical velocity into the backward component.
    // With heading = 0, a purely vertical velocity (0, 0, v) after Ry(-pitch) should give:
    //   vx_robot = -sin(pitch) * v, vz_robot = cos(pitch) * v
    final double pitch = Math.toRadians(5.0);
    final double v = 10.0;
    final double[] result = LaunchCalculator.fieldToRobotVelocity(0.0, 0.0, v, 0.0, 0.0, pitch);
    assertEquals(-Math.sin(pitch) * v, result[0], kTol);
    assertEquals(0.0, result[1], kTol);
    assertEquals(Math.cos(pitch) * v, result[2], kTol);
  }

  @Test
  public void fieldToRobotVelocityCombinedRollAndPitch() {
    // With both roll and pitch of 45°, verify the result against a manual matrix computation.
    final double angle = Math.toRadians(45.0);
    final double c = Math.cos(angle);
    final double s = Math.sin(angle);
    // Input: (1, 0, 0) in field frame with heading = 0
    // Ry(-45°): (c*1 - s*0, 0, s*1 + c*0) = (c, 0, s)
    // Rx(-45°): (c, c*0 + s*s, -s*0 + c*s) = (c, s², cs)
    final double[] result = LaunchCalculator.fieldToRobotVelocity(1.0, 0.0, 0.0, 0.0, angle, angle);
    assertEquals(c, result[0], kTol);
    assertEquals(s * s, result[1], kTol);
    assertEquals(c * s, result[2], kTol);
  }
}
