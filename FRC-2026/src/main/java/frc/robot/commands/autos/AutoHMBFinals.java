package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.commands.FollowQuikplan;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoHMBFinals implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_first;
  private final QuikPlanSwervePartialTrajectoryReader m_second;

  private final Command m_command;

  public AutoHMBFinals(
      final SwerveSubsystem swerve, final IntakeSubsystem intake, final boolean isLeft) {
    m_first = new QuikPlanSwervePartialTrajectoryReader("vs_XXXX_XXXX_first_sweep.csv", isLeft);
    m_second = new QuikPlanSwervePartialTrajectoryReader("vs_XXXX_XXXX_second_sweep.csv", isLeft);
    m_command =
        new FollowQuikplan(m_first, swerve, intake)
            .andThen(new FollowQuikplan(m_second, swerve, intake));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_first.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_first, m_second));
  }
}
