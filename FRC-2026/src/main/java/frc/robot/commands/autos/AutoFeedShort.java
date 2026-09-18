package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.commands.FollowQuikplan;
import frc.robot.commands.RetractIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoFeedShort implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_first;

  private final Command m_command;

  public AutoFeedShort(
      final SwerveSubsystem swerve, final IntakeSubsystem intake, final boolean isLeft) {
    m_first = new QuikPlanSwervePartialTrajectoryReader("feed_first_cycle_short.csv", isLeft);
    m_command =
        new FollowQuikplan(m_first, swerve, intake)
            .andThen(new WaitCommand(3.0).andThen(new RetractIntake(intake)));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_first.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_first));
  }
}
