// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.quixlib.devices.QuixIOUtil;
import frc.quixlib.phoenix.PhoenixUtil;
import frc.quixlib.powermonitor.QuixPowerMonitor;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  static {
    // WPIlib OpenCV Version
    // https://github.com/wpilibsuite/GradleRIO/blob/v2026.2.1/src/main/java/edu/wpi/first/gradlerio/wpi/WPIVersionsExtension.java#L11
    System.loadLibrary("opencv_java4100");
  }

  public Robot() {
    RobotController.setBrownoutVoltage(6.25);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (!isReal() && Constants.isReplay) {
      // Replaying a log, set up replay source
      setUseTiming(Constants.resimWithTiming);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
    } else {
      // Log to a USB stick ("/U/logs") on a real robot and "logs" in sim.
      Logger.addDataReceiver(new WPILOGWriter());
      // Publish data to NetworkTables
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger
    Logger.start();

    // Start WPILib logger (for raw NetworkTables logs) only on a real robot.
    // TODO: Evaluate if this is actually needed, or if we can get away with just AdvantageKit.
    if (isReal()) {
      DataLogManager.start();
    }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void disabledPeriodic() {
    ShiftManager.initialize();

    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    double startTimestamp = Timer.getFPGATimestamp();
    PhoenixUtil.refreshAll();
    double endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedRobot/PhoenixRefreshMs", (endTimestamp - startTimestamp) * 1000.0);

    startTimestamp = Timer.getFPGATimestamp();
    QuixIOUtil.updateAll();
    endTimestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedRobot/QuixIOUpdateMs", (endTimestamp - startTimestamp) * 1000.0);

    QuixPowerMonitor.logPowerStats();

    ShiftManager.logShiftInfo();

    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();

    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void loopFunc() {
    // Update programmable simulation scenario if active before any periodic method runs
    if (isSimulation()) {
      frc.robot.simulation.SimScenarioManager.getInstance()
          .update(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
    super.loopFunc();
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }

  @Override
  public void autonomousInit() {
    ShiftManager.initialize();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    ShiftManager.initialize();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
