package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikPlanAction;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikplanTrajectoryState;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;

public class FollowQuikplan extends Command {
  private final boolean kDebugPrint = false;

  private final QuikPlanSwervePartialTrajectoryReader m_reader;
  private final SwerveSubsystem m_swerve;
  private final IntakeSubsystem m_intake;

  private final Timer m_timer = new Timer();
  private final double m_endTimeExtension;

  private ArrayList<Double> m_times = new ArrayList<>();
  private ArrayList<Double> m_xErrors = new ArrayList<>();
  private ArrayList<Double> m_yErrors = new ArrayList<>();
  private ArrayList<Double> m_thetaErrors = new ArrayList<>();

  private boolean m_shootEnabled = false;
  private boolean m_deployIntake = false;
  private boolean m_rollIntake = false;

  public FollowQuikplan(
      QuikPlanSwervePartialTrajectoryReader reader,
      SwerveSubsystem swerve,
      IntakeSubsystem intake) {
    this(reader, swerve, intake, 0.0);
  }

  public FollowQuikplan(
      QuikPlanSwervePartialTrajectoryReader reader,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      double endTimeExtension) {
    m_reader = reader;
    m_swerve = swerve;
    m_intake = intake;
    m_endTimeExtension = endTimeExtension;

    addRequirements(swerve, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
    m_shootEnabled = false;
    m_deployIntake = false;
    m_rollIntake = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double curTime = m_timer.get();
    final QuikplanTrajectoryState targetState = m_reader.getState(curTime);
    final var actionEntry = m_reader.getAction(curTime);
    final QuikPlanAction action = actionEntry == null ? null : actionEntry.getValue();

    if (actionEntry != null) {
      switch (action.actionType) {
        case 1:
          m_shootEnabled = true;
          break;
        case 2:
          m_shootEnabled = false;
          break;
        case 3:
          m_deployIntake = true;
          m_rollIntake = true;
          break;
        case 4:
          m_rollIntake = false;
          break;
        case 5:
          m_deployIntake = false;
          m_rollIntake = false;
          break;
        case 6:
          m_rollIntake = false;
          m_shootEnabled = true;
        case 7:
          m_deployIntake = true;
          m_rollIntake = true;
          m_shootEnabled = true;
          break;
        case 8:
          m_deployIntake = true;
          m_rollIntake = true;
          m_shootEnabled = false;
        case 9:
          m_deployIntake = false;
          m_rollIntake = false;
          m_shootEnabled = false;
          break;
        case 10:
          m_deployIntake = false;
          m_rollIntake = false;
          m_shootEnabled = false;
        // TODO this also does climb
        default:
          break;
      }
    }

    final Pose2d poseError =
        m_swerve.driveToPose(
            targetState.pose,
            targetState.xVel,
            targetState.yVel,
            targetState.thetaVel,
            Constants.Swerve.autoScrubLimit);
    if (kDebugPrint) {
      m_times.add(m_timer.get());
      m_xErrors.add(poseError.getX());
      m_yErrors.add(poseError.getY());
      m_thetaErrors.add(poseError.getRotation().getRadians());
    }

    if (m_deployIntake) {
      m_intake.setAngle(Constants.Intake.deployAngle, false);
    } else {
      m_intake.setAngle(Constants.Intake.stowAngle, true);
    }

    if (m_rollIntake) {
      m_intake.setRollerPercentOutput(Constants.Intake.rollerPercentOutput);
    } else {
      m_intake.setRollerPercentOutput(0.0);
    }

    if (m_shootEnabled) {
      TrackAndLaunchCommand.enableLaunch();
    } else {
      TrackAndLaunchCommand.disableLaunch();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_swerve.stop();
    if (kDebugPrint) {
      for (int i = 0; i < m_times.size(); i++) {
        System.out.println(
            m_times.get(i)
                + ","
                + m_xErrors.get(i)
                + ","
                + m_yErrors.get(i)
                + ","
                + m_thetaErrors.get(i));
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_reader.getTotalTime() + m_endTimeExtension);
  }
}
