package frc.quixlib.wpilib;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

public class InterpolateableChassisSpeedsTest {
  private final double kTol = 1e-6;

  private final InterpolateableChassisSpeeds start = new InterpolateableChassisSpeeds(1, 2, 3);
  private final InterpolateableChassisSpeeds end = new InterpolateableChassisSpeeds(4, 5, 6);

  @Test
  public void testOutsideBounds() {
    assertEquals(start.interpolate(end, -1), start);
    assertEquals(start.interpolate(end, 0), start);
    assertEquals(start.interpolate(end, 1), end);
    assertEquals(start.interpolate(end, 2), end);
  }

  @Test
  public void testWithinBounds() {
    var result = start.interpolate(end, 0.1);
    assertEquals(result.vxMetersPerSecond, 1.3, kTol);
    assertEquals(result.vyMetersPerSecond, 2.3, kTol);
    assertEquals(result.omegaRadiansPerSecond, 3.3, kTol);

    result = start.interpolate(end, 0.5);
    assertEquals(result.vxMetersPerSecond, 2.5, kTol);
    assertEquals(result.vyMetersPerSecond, 3.5, kTol);
    assertEquals(result.omegaRadiansPerSecond, 4.5, kTol);

    result = start.interpolate(end, 0.9);
    assertEquals(result.vxMetersPerSecond, 3.7, kTol);
    assertEquals(result.vyMetersPerSecond, 4.7, kTol);
    assertEquals(result.omegaRadiansPerSecond, 5.7, kTol);
  }
}
