package frc.quixlib.planning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.quixlib.math.MathUtils;
import frc.quixlib.planning.QuixTrapezoidProfile.Constraints;
import frc.quixlib.swerve.FieldSpeeds;

public class SimpleSwerveTrajectory {
  private final Pose2d m_startPose;
  private final FieldSpeeds m_startSpeeds;

  private final QuixTrapezoidProfile m_xTrap;
  private final QuixTrapezoidProfile m_yTrap;
  private final QuixTrapezoidProfile m_thetaTrap;

  private final double m_maxTime;
  private final double m_xTimeScalar;
  private final double m_yTimeScalar;
  private final double m_thetaTimeScalar;

  /**
   * Naive acceleration-limited trajectory starting at startPose and startSpeeds and ends at
   * targetPose with zero velocity.
   */
  public SimpleSwerveTrajectory(
      final Pose2d startPose,
      final FieldSpeeds startSpeeds,
      final Pose2d targetPose,
      final Constraints xyConstraints,
      final Constraints thetaConstraints) {
    m_startPose = startPose;
    m_startSpeeds = startSpeeds;
    final Rotation2d startRotation = startPose.getRotation();

    m_xTrap =
        new QuixTrapezoidProfile(
            xyConstraints,
            new State(targetPose.getX(), 0.0),
            new State(startPose.getX(), startSpeeds.vxMetersPerSecond));
    m_yTrap =
        new QuixTrapezoidProfile(
            xyConstraints,
            new State(targetPose.getY(), 0.0),
            new State(startPose.getY(), startSpeeds.vyMetersPerSecond));
    m_thetaTrap =
        new QuixTrapezoidProfile(
            thetaConstraints,
            new State(
                MathUtils.placeInScope(
                    targetPose.getRotation().getRadians(), startRotation.getRadians()),
                0.0),
            new State(startRotation.getRadians(), startSpeeds.omegaRadiansPerSecond));

    // Scale the time of the profiles so that they all end at the same time.
    final double xTime = m_xTrap.totalTime();
    final double yTime = m_yTrap.totalTime();
    final double thetaTime = m_thetaTrap.totalTime();
    m_maxTime = Math.max(Math.max(xTime, yTime), thetaTime);
    m_xTimeScalar = xTime / m_maxTime;
    m_yTimeScalar = yTime / m_maxTime;
    m_thetaTimeScalar = thetaTime / m_maxTime;
  }

  /** Returns the trajectory state at time t. */
  public SwerveTrajectoryState getState(final double t) {
    if (m_maxTime == 0.0) {
      return new SwerveTrajectoryState(
          m_startPose,
          m_startSpeeds.vxMetersPerSecond,
          m_startSpeeds.vyMetersPerSecond,
          m_startSpeeds.omegaRadiansPerSecond);
    }

    final var xState = m_xTrap.calculate(t * m_xTimeScalar);
    final var yState = m_yTrap.calculate(t * m_yTimeScalar);
    final var thetaState = m_thetaTrap.calculate(t * m_thetaTimeScalar);
    return new SwerveTrajectoryState(
        new Pose2d(xState.position, yState.position, new Rotation2d(thetaState.position)),
        xState.velocity * m_xTimeScalar,
        yState.velocity * m_yTimeScalar,
        thetaState.velocity * m_thetaTimeScalar);
  }

  public double totalTime() {
    return m_maxTime;
  }
}
