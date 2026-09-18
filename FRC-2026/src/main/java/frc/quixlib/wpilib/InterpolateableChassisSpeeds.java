package frc.quixlib.wpilib;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class InterpolateableChassisSpeeds extends ChassisSpeeds
    implements Interpolatable<InterpolateableChassisSpeeds> {
  public InterpolateableChassisSpeeds(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }

  @Override
  public InterpolateableChassisSpeeds interpolate(InterpolateableChassisSpeeds endValue, double t) {
    if (t < 0) {
      return this;
    }
    if (t >= 1) {
      return endValue;
    }
    return new InterpolateableChassisSpeeds(
        (1 - t) * vxMetersPerSecond + t * endValue.vxMetersPerSecond,
        (1 - t) * vyMetersPerSecond + t * endValue.vyMetersPerSecond,
        (1 - t) * omegaRadiansPerSecond + t * endValue.omegaRadiansPerSecond);
  }

  public static InterpolateableChassisSpeeds fromChassisSpeeds(ChassisSpeeds cs) {
    return new InterpolateableChassisSpeeds(
        cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond);
  }
}
