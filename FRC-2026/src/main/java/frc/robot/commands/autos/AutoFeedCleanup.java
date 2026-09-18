package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.commands.FollowQuikplan;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoFeedCleanup implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_traj =
      new QuikPlanSwervePartialTrajectoryReader("feed_cleanup.csv");

  private final Command m_command;

  public AutoFeedCleanup(
      final SwerveSubsystem swerve, final IntakeSubsystem intake, final boolean isLeft) {
    m_command = new FollowQuikplan(m_traj, swerve, intake, 1.0);
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_traj.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_traj));
  }
}
