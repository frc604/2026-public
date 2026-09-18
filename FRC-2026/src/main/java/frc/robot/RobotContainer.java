// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.math.LaunchCalculator;
import frc.quixlib.vision.PhotonVisionCamera;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.robot.Constants.FieldZones;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.TrackAndLaunchCommand;
import frc.robot.commands.UnjamCommand;
import frc.robot.commands.autos.AutoCenterPause;
import frc.robot.commands.autos.AutoCenterPauseHub;
import frc.robot.commands.autos.AutoCenterPauseOpp;
import frc.robot.commands.autos.AutoCenterPauseSlowLeft;
import frc.robot.commands.autos.AutoCenterPauseSlowLeftXXXX;
import frc.robot.commands.autos.AutoCenterPauseSlowRight;
import frc.robot.commands.autos.AutoCenterPauseSlowRightXXXX;
import frc.robot.commands.autos.AutoCommand;
import frc.robot.commands.autos.AutoDoubleCycleRightOutsideIn;
import frc.robot.commands.autos.AutoDoubleCycleRightWithXXXX;
import frc.robot.commands.autos.AutoFastRaceLeft;
import frc.robot.commands.autos.AutoFastRaceNoDepot;
import frc.robot.commands.autos.AutoFastRaceRight;
import frc.robot.commands.autos.AutoGoAfterPartnerFeed;
import frc.robot.commands.autos.AutoGoAfterPartnerPauseReturnSameSideLeft;
import frc.robot.commands.autos.AutoGoAfterPartnerPauseReturnSameSideRight;
import frc.robot.commands.autos.AutoGoAfterPartnerReturnSameSideLeft;
import frc.robot.commands.autos.AutoGoAfterPartnerReturnSameSideLeftNoDepot;
import frc.robot.commands.autos.AutoGoAfterPartnerReturnSameSideRight;
import frc.robot.commands.autos.AutoGoAfterPartnerReturnSameSideRightNoDepot;
import frc.robot.commands.autos.AutoSimpleTest;
import frc.robot.commands.autos.AutoSlowRaceNoDepot;
import frc.robot.commands.autos.AutoSquareTest;
import frc.robot.commands.test.LaunchFuelTest;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Controllers
  public final XboxController driverXbox = new XboxController(0);

  // ABXY Buttons
  private final JoystickButton buttonA =
      new JoystickButton(driverXbox, XboxController.Button.kA.value);
  private final JoystickButton buttonB =
      new JoystickButton(driverXbox, XboxController.Button.kB.value);
  private final JoystickButton buttonX =
      new JoystickButton(driverXbox, XboxController.Button.kX.value);
  private final JoystickButton buttonY =
      new JoystickButton(driverXbox, XboxController.Button.kY.value);

  // Bumpers
  private final JoystickButton bumperLeft =
      new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton bumperRight =
      new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);

  // Triggers
  private final Trigger leftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.2);
  private final Trigger rightTrigger = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.2);

  // D-pad Buttons
  private final Trigger dPadUp = new Trigger(() -> driverXbox.getPOV() == 0);
  private final Trigger dPadRight = new Trigger(() -> driverXbox.getPOV() == 90);
  private final Trigger dPadDown = new Trigger(() -> driverXbox.getPOV() == 180);
  private final Trigger dPadLeft = new Trigger(() -> driverXbox.getPOV() == 270);

  // Menu Buttons
  private final JoystickButton buttonStart =
      new JoystickButton(driverXbox, XboxController.Button.kStart.value);
  private final JoystickButton buttonBack =
      new JoystickButton(driverXbox, XboxController.Button.kBack.value);

  // Paddles
  // Bind sticks to paddles on controller
  private final Trigger leftPaddle =
      new JoystickButton(driverXbox, XboxController.Button.kLeftStick.value);
  private final Trigger rightPaddle =
      new JoystickButton(driverXbox, XboxController.Button.kRightStick.value);

  // Sensors
  private final QuixPigeon2 imu =
      new QuixPigeon2(
          Constants.IMU.pigeonID,
          QuixPigeon2.makeDefaultConfig().setGyroTrimZ(Constants.IMU.gyroTrimZ));
  private final ArrayList<QuixVisionCamera> cameras =
      new ArrayList<>(
          Arrays.asList(
              new PhotonVisionCamera(
                  "left",
                  Constants.Cameras.LeftCam.robotToCameraT,
                  Constants.Cameras.LeftCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "backleft",
                  Constants.Cameras.BackLeftCam.robotToCameraT,
                  Constants.Cameras.BackLeftCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "backright",
                  Constants.Cameras.BackRightCam.robotToCameraT,
                  Constants.Cameras.BackRightCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "right",
                  Constants.Cameras.RightCam.robotToCameraT,
                  Constants.Cameras.RightCam.pipelineConfigs)));

  // Simulation & viz
  private final QuixVisionSim visionSim = new QuixVisionSim(cameras, Fiducials.aprilTagFiducials);
  private final Field2d fieldViz = visionSim.getSimField();

  // Simulation setup (must be before subsystem initialization)
  final Timer m_lastLaunchedFuelSimTimer;

  {
    if (Robot.isSimulation()) {
      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
      SimulatedArena.getInstance().resetFieldForAuto();
      m_lastLaunchedFuelSimTimer = new Timer();
      m_lastLaunchedFuelSimTimer.start();
    } else {
      m_lastLaunchedFuelSimTimer = null;
    }
  }

  // Subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(imu, cameras, visionSim, fieldViz);
  private final IntakeSimulation intakeSim =
      Robot.isSimulation()
          ? IntakeSimulation.OverTheBumperIntake(
              // Specify the type of game pieces that the intake can collect
              "Fuel",
              // Specify the drivetrain to which this intake is attached
              swerve.getDriveSim(),
              // Width of the intake
              Inches.of(25.0),
              // The extension length of the intake beyond the robot's frame (when activated)
              Inches.of(6.0),
              // The intake is mounted on the back side of the chassis
              IntakeSimulation.IntakeSide.FRONT,
              67)
          : null;

  private final IntakeSubsystem intake = new IntakeSubsystem(intakeSim);
  private final SerializerSubsystem serializer = new SerializerSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();

  // Misc.
  private final LaunchCalculator launchCalculator =
      new LaunchCalculator(
          Filesystem.getDeployDirectory().toPath()
              + "/launch/continuousv0theta_accuracy_checked.csv");

  // Autos
  private final LoggedDashboardChooser<AutoCommand> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private AutoCommand lastSelectedAuto = null;

  public RobotContainer() {
    // Autos
    // autoChooser.addOption(
    //     "Double Cycle RIGHT", new AutoDoubleCycleRight(swerve, intake, false));
    // autoChooser.addOption("Double Cycle LEFT", new AutoDoubleCycleRight(swerve, intake, true));
    // autoChooser.addOption(
    //     "Double Cycle (no opp) RIGHT", new AutoDoubleCycleNoOppRight(swerve, intake, false));
    // autoChooser.addOption(
    //     "Double Cycle (no opp) LEFT", new AutoDoubleCycleNoOppRight(swerve, intake, true));
    // autoChooser.addOption(
    //     "Single Cycle No Citrus RIGHT", new AutoSingleCycleNoCitrusRight(swerve, intake, false));
    // autoChooser.addOption(
    //     "Single Cycle No Citrus LEFT", new AutoSingleCycleNoCitrusRight(swerve, intake, true));
    // autoChooser.addOption("vs XXXX RIGHT", new AutoVsXXXX(swerve, intake, false));
    // autoChooser.addOption("vs XXXX LEFT", new AutoVsXXXX(swerve, intake, true));
    // autoChooser.addOption("HMB Finals RIGHT", new AutoHMBFinals(swerve, intake, false));
    // autoChooser.addOption("HMB Finals LEFT", new AutoHMBFinals(swerve, intake, true));

    // autoChooser.addOption("Feed Short RIGHT", new AutoFeedShort(swerve, intake, false));
    // autoChooser.addOption("Feed Short LEFT", new AutoFeedShort(swerve, intake, true));
    // autoChooser.addOption("Feed Long RIGHT", new AutoFeedLong(swerve, intake, false));
    // autoChooser.addOption("Feed Long LEFT", new AutoFeedLong(swerve, intake, true));

    // autoChooser.addOption(
    //     "Double Cycle Outwards RIGHT", new AutoDoubleCycleRightInsideOut(swerve, intake, false));
    // autoChooser.addOption(
    //     "Double Cycle Outwards LEFT", new AutoDoubleCycleRightInsideOut(swerve, intake, true));
    autoChooser.addDefaultOption(
        "Double Cycle Inwards RIGHT", new AutoDoubleCycleRightOutsideIn(swerve, intake, false));
    autoChooser.addOption(
        "Double Cycle Inwards LEFT", new AutoDoubleCycleRightOutsideIn(swerve, intake, true));

    // autoChooser.addOption("Cleanup Feed RIGHT", new AutoFeedCleanup(swerve, intake, false));
    // autoChooser.addOption("Cleanup Feed LEFT", new AutoFeedCleanup(swerve, intake, true));

    autoChooser.addOption("XXXX RIGHT", new AutoDoubleCycleRightWithXXXX(swerve, intake, false));
    autoChooser.addOption("XXXX LEFT", new AutoDoubleCycleRightWithXXXX(swerve, intake, true));

    autoChooser.addOption(
        "Single Cycle Center Pause RIGHT", new AutoCenterPause(swerve, intake, false));
    autoChooser.addOption(
        "Single Cycle Center Pause LEFT", new AutoCenterPause(swerve, intake, true));
    autoChooser.addOption(
        "Single Cycle Center Pause OPP RIGHT", new AutoCenterPauseOpp(swerve, intake, false));
    autoChooser.addOption(
        "Single Cycle Center Pause OPP LEFT", new AutoCenterPauseOpp(swerve, intake, true));
    autoChooser.addOption(
        "Single Cycle Center Pause HUB RIGHT", new AutoCenterPauseHub(swerve, intake, false));
    autoChooser.addOption(
        "Single Cycle Center Pause HUB LEFT", new AutoCenterPauseHub(swerve, intake, true));
    autoChooser.addOption("Center Hub + Depot RIGHT", new AutoCenterPauseSlowRight(swerve, intake));
    autoChooser.addOption("Center Hub + Depot LEFT", new AutoCenterPauseSlowLeft(swerve, intake));

    autoChooser.addOption(
        "Center Hub + Depot RIGHT XXXX", new AutoCenterPauseSlowRightXXXX(swerve, intake));
    autoChooser.addOption(
        "Center Hub + Depot LEFT XXXX", new AutoCenterPauseSlowLeftXXXX(swerve, intake));

    // Champs autos
    autoChooser.addOption("Fast Race RIGHT", new AutoFastRaceRight(swerve, intake));
    autoChooser.addOption("Fast Race LEFT", new AutoFastRaceLeft(swerve, intake));
    autoChooser.addOption(
        "Fast Race no depot RIGHT", new AutoFastRaceNoDepot(swerve, intake, false));
    autoChooser.addOption("Fast Race no depot LEFT", new AutoFastRaceNoDepot(swerve, intake, true));
    autoChooser.addOption(
        "Slow Race no depot RIGHT", new AutoSlowRaceNoDepot(swerve, intake, false));
    autoChooser.addOption("Slow Race no depot LEFT", new AutoSlowRaceNoDepot(swerve, intake, true));
    autoChooser.addOption(
        "Go after partner + return same RIGHT",
        new AutoGoAfterPartnerReturnSameSideRight(swerve, intake));
    autoChooser.addOption(
        "Go after partner + return same LEFT",
        new AutoGoAfterPartnerReturnSameSideLeft(swerve, intake));
    autoChooser.addOption(
        "Go after partner pause + return same RIGHT",
        new AutoGoAfterPartnerPauseReturnSameSideRight(swerve, intake));
    autoChooser.addOption(
        "Go after partner pause + return same LEFT",
        new AutoGoAfterPartnerPauseReturnSameSideLeft(swerve, intake));
    autoChooser.addOption(
        "Go after partner + return same NO DEPOT RIGHT",
        new AutoGoAfterPartnerReturnSameSideRightNoDepot(swerve, intake));
    autoChooser.addOption(
        "Go after partner + return same NO DEPOT LEFT",
        new AutoGoAfterPartnerReturnSameSideLeftNoDepot(swerve, intake));
    autoChooser.addOption(
        "Go after partner + feed RIGHT", new AutoGoAfterPartnerFeed(swerve, intake, false));
    autoChooser.addOption(
        "Go after partner + feed LEFT", new AutoGoAfterPartnerFeed(swerve, intake, true));

    // Test autos
    autoChooser.addOption("Simple Test", new AutoSimpleTest(swerve, intake));
    autoChooser.addOption("Square Test", new AutoSquareTest(swerve, intake));

    // Default commands
    swerve.setDefaultCommand(new TeleopSwerveCommand(swerve, driverXbox));
    turret.setDefaultCommand(
        new TrackAndLaunchCommand(
            turret,
            serializer,
            swerve::setDriveSupplyCurrentLimit,
            swerve::getPose,
            swerve::getVelocity,
            swerve::getAcceleration,
            swerve::getFieldRelativeVelocity,
            swerve::getFieldRelativeAcceleration,
            imu::getRoll,
            imu::getPitch,
            launchCalculator,
            driverXbox::getLeftBumperButton));

    configureBindings();
  }

  private void configureBindings() {
    // Gyro reset
    buttonStart.onTrue(
        new InstantCommand(
            () -> {
              final var alliance = DriverStation.getAlliance();
              swerve.setContinuousYaw(
                  alliance.isPresent() && alliance.get() == Alliance.Blue ? 0.0 : Math.PI);
            }));
    bumperRight.onTrue(new RetractIntake(intake));
    rightTrigger.onTrue(new DeployIntake(intake, driverXbox));

    leftTrigger.whileFalse(
        new InstantCommand(
            () -> {
              TrackAndLaunchCommand.disableLaunch();
            }));
    leftTrigger.onTrue(
        new InstantCommand(
            () -> {
              TrackAndLaunchCommand.enableLaunch();
            }));

    dPadUp.whileTrue(new LaunchFuelTest(serializer, turret));
    dPadDown.whileTrue(
        new ParallelCommandGroup(
            new UnjamCommand(intake, serializer),
            new InstantCommand(
                () -> {
                  if (Robot.isSimulation()) {
                    SimulatedArena.getInstance().resetFieldForAuto();
                  }
                })));

    // Test commands
    // dPadLeft.whileTrue(new TurretTest(turret, Math.toRadians(150), Math.toRadians(55)));
    // dPadRight.whileTrue(new TurretTest(turret, Math.toRadians(210), Math.toRadians(75)));
    // buttonA.whileTrue(
    //     new LaserTurretTest(
    //         turret,
    //         swerve::getPose,
    //         swerve::getFieldRelativeVelocity,
    //         swerve::getFieldRelativeAcceleration));
    // buttonY.whileTrue(new LaunchFuelTest(serializer, turret));
    // buttonB.whileTrue(new LaunchVelocityCalibration(serializer, turret, launchCalculator));
    // bumperRight.whileTrue(
    //     new LaunchFuelHubTest(
    //         turret,
    //         swerve::getPose,
    //         swerve::getFieldRelativeVelocity,
    //         swerve::getFieldRelativeAcceleration,
    //         launchCalculator,
    //         serializer));
  }

  public Command getAutonomousCommand() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      return null;
    }
    return selectedAuto.getCommand();
  }

  public void disabledPeriodic() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      // Clear poses
      fieldViz.getObject("traj").setPoses();
      LoggerHelper.recordPose2dList("AutoTraj", new ArrayList<Pose2d>());
    } else if (lastSelectedAuto != selectedAuto) {
      selectedAuto.loadAndUpdateViz(fieldViz);
      swerve.resetPose(selectedAuto.getInitialPose());
      if (Robot.isSimulation()) {
        swerve.resetSimPose(selectedAuto.getInitialPose());
      }
    }
    lastSelectedAuto = selectedAuto;

    // Handle rotor trim
    if (driverXbox.getAButton()
        && driverXbox.getBButton()
        && driverXbox.getXButton()
        && driverXbox.getYButton()) {
      intake.setRotorTrims();
      turret.setRotorTrims();
    }

    // Log all field zones
    final boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Logger.recordOutput("FieldZones/TOWER_ZONE", FieldZones.TOWER_ZONE.getCornerPoses(isRed));
    Logger.recordOutput("FieldZones/BUMP_ZONE", FieldZones.BUMP_ZONE.getCornerPoses(isRed));
    Logger.recordOutput("FieldZones/LEFT_ZONE", FieldZones.LEFT_ZONE.getCornerPoses(isRed));
    Logger.recordOutput("FieldZones/RIGHT_ZONE", FieldZones.RIGHT_ZONE.getCornerPoses(isRed));
    Logger.recordOutput("FieldZones/BLOCKED_ZONE", FieldZones.BLOCKED_ZONE.getCornerPoses(isRed));

    // Not sure why this is needed to get this to populate on NT.
    Constants.Auto.startingDelay.get();
  }

  public void robotPeriodic() {
    final Transform3d robotToIntakeLeftArmTransform =
        Constants.Viz3d.robotToIntakeLeftArm.plus(
            new Transform3d(0, 0, 0, new Rotation3d(0, -intake.getLeftWristAngle(), 0)));
    final Transform3d robotToIntakeRightArmTransform =
        Constants.Viz3d.robotToIntakeRightArm.plus(
            new Transform3d(0, 0, 0, new Rotation3d(0, -intake.getRightWristAngle(), 0)));
    final Transform3d robotToSerializerHookTransform =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, -serializer.getRotorAngle()));
    final Transform3d robotToTurretBaseTransform =
        Constants.Turret.robotToTurretBaseT.plus(
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, turret.getYaw())));
    final Transform3d robotToTurretHoodTransform =
        robotToTurretBaseTransform
            .plus(Constants.Turret.turretBaseToHoodT)
            .plus(new Transform3d(0, 0, 0, new Rotation3d(0, turret.getHoodAngle(), 0)));

    final Pose3d intakeLeftArm = Pose3d.kZero.transformBy(robotToIntakeLeftArmTransform);
    final Pose3d intakeRightArm = Pose3d.kZero.transformBy(robotToIntakeRightArmTransform);
    final Pose3d serializerHook = Pose3d.kZero.transformBy(robotToSerializerHookTransform);
    final Pose3d turretBase = Pose3d.kZero.transformBy(robotToTurretBaseTransform);
    final Pose3d turretHood = Pose3d.kZero.transformBy(robotToTurretHoodTransform);

    Logger.recordOutput(
        "mechanismPoses",
        new Pose3d[] {intakeLeftArm, intakeRightArm, serializerHook, turretBase, turretHood});

    // Plot turret line
    Translation2d Translation = new Translation2d(10, new Rotation2d(turret.getYaw()));
    // take those xy coords and convert then to a transformation for the robot's current pose
    Transform2d Transformation = new Transform2d(Translation, new Rotation2d(0.0));
    Pose2d TurretPose = swerve.getPose().transformBy(Constants.Turret.robotToTurretBaseT2d);
    Pose2d LineEnd = TurretPose.transformBy(Transformation);
    Logger.recordOutput("Turret/turretLine", TurretPose, LineEnd);
  }

  public void simulationPeriodic() {
    // Run MapleSim tick
    final var simulatedArena = SimulatedArena.getInstance();
    final var driveSim = swerve.getDriveSim();

    if (m_lastLaunchedFuelSimTimer.get() > serializer.getSecondsPerBall()) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  driveSim.getSimulatedDriveTrainPose().getTranslation(),
                  Constants.Turret.robotToTurretBaseT.getTranslation().toTranslation2d(),
                  driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  driveSim
                      .getSimulatedDriveTrainPose()
                      .getRotation()
                      .plus(new Rotation2d(turret.getYaw())),
                  Units.Meters.of(Constants.Turret.robotToTurretBaseT.getTranslation().getZ()),
                  Units.MetersPerSecond.of(turret.getLaunchVelocity()),
                  Units.Radians.of(turret.getLaunchAngle())));
      m_lastLaunchedFuelSimTimer.reset();
    }

    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);

    simulatedArena.simulationPeriodic();
  }
}
