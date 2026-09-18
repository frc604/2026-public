package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.Constants;
import frc.robot.commands.FollowQuikplan;
import frc.robot.commands.RetractIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoFastRaceLeft implements AutoCommand {
  private final SwerveSubsystem m_swerve;
  private final IntakeSubsystem m_intake;

  private final QuikPlanSwervePartialTrajectoryReader m_first;
  private final QuikPlanSwervePartialTrajectoryReader m_second;
  private final QuikPlanSwervePartialTrajectoryReader m_third;

  public AutoFastRaceLeft(final SwerveSubsystem swerve, final IntakeSubsystem intake) {
    m_swerve = swerve;
    m_intake = intake;
    m_first = new QuikPlanSwervePartialTrajectoryReader("fast_race.csv", true);
    m_second = new QuikPlanSwervePartialTrajectoryReader("fast_race_return.csv", true);
    m_third = new QuikPlanSwervePartialTrajectoryReader("shoot_to_depot_left.csv");
  }

  public Command getCommand() {
    return new FollowQuikplan(m_first, m_swerve, m_intake)
        .andThen(new WaitCommand(Constants.Auto.centerDelay.get()))
        .andThen(new FollowQuikplan(m_second, m_swerve, m_intake, 1.0))
        .andThen(new FollowQuikplan(m_third, m_swerve, m_intake, 1.0))
        .andThen(new WaitCommand(1.0).andThen(new RetractIntake(m_intake)));
  }

  public Pose2d getInitialPose() {
    return m_first.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_first, m_second, m_third));
  }
}
