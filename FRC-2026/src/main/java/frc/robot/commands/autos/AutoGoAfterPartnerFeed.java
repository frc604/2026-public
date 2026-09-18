package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.Constants;
import frc.robot.commands.FollowQuikplan;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoGoAfterPartnerFeed implements AutoCommand {
  private final SwerveSubsystem m_swerve;
  private final IntakeSubsystem m_intake;

  private final QuikPlanSwervePartialTrajectoryReader m_first;
  private final QuikPlanSwervePartialTrajectoryReader m_second;

  public AutoGoAfterPartnerFeed(
      final SwerveSubsystem swerve, final IntakeSubsystem intake, final boolean isLeft) {
    m_swerve = swerve;
    m_intake = intake;
    m_first = new QuikPlanSwervePartialTrajectoryReader("go_after_partner.csv", isLeft);
    m_second = new QuikPlanSwervePartialTrajectoryReader("after_partner_feed.csv", isLeft);
  }

  public Command getCommand() {
    return new WaitCommand(Constants.Auto.startingDelay.get())
        .andThen(new FollowQuikplan(m_first, m_swerve, m_intake))
        .andThen(new FollowQuikplan(m_second, m_swerve, m_intake, 1.0));
  }

  public Pose2d getInitialPose() {
    return m_first.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_first, m_second));
  }
}
