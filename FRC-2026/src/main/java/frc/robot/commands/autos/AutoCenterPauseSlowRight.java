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

public class AutoCenterPauseSlowRight implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_first;
  private final QuikPlanSwervePartialTrajectoryReader m_second;
  private final QuikPlanSwervePartialTrajectoryReader m_third;

  private final Command m_command;

  public AutoCenterPauseSlowRight(final SwerveSubsystem swerve, final IntakeSubsystem intake) {
    m_first = new QuikPlanSwervePartialTrajectoryReader("center_pause_slow.csv");
    m_second = new QuikPlanSwervePartialTrajectoryReader("center_pause_slow_sweep_hub.csv");
    m_third = new QuikPlanSwervePartialTrajectoryReader("shoot_to_depot.csv");
    m_command =
        new FollowQuikplan(m_first, swerve, intake)
            .andThen(new WaitCommand(2.0))
            .andThen(new FollowQuikplan(m_second, swerve, intake))
            .andThen(new FollowQuikplan(m_third, swerve, intake, 1.0))
            .andThen(new WaitCommand(1.0).andThen(new RetractIntake(intake)));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_first.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_first, m_second, m_third));
  }
}
