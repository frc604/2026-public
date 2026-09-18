package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixCANBus;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.swerve.QuixSwerveController;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Constants {
  public static final boolean isReplay = false;
  public static final boolean resimWithTiming = false;
  public static final boolean simLocalization = false;

  private static final QuixCANBus rioBus = new QuixCANBus();
  private static final QuixCANBus canivoreBus = new QuixCANBus("canivore");

  public static final class Auto {
    public static final LoggedNetworkNumber startingDelay =
        new LoggedNetworkNumber("Auto/startingDelay", 0.5);
    public static final LoggedNetworkNumber centerDelay =
        new LoggedNetworkNumber("Auto/centerDelay", 0.5);
  }

  public static final class RotorTrim {
    public static final LoggedNetworkNumber leftIntake =
        new LoggedNetworkNumber("RotorTrim/leftIntake", 0);
    public static final LoggedNetworkNumber rightIntake =
        new LoggedNetworkNumber("RotorTrim/rightIntake", 0);
    public static final LoggedNetworkNumber turretYaw =
        new LoggedNetworkNumber("RotorTrim/turretYaw", 0);
    public static final LoggedNetworkNumber turretHood =
        new LoggedNetworkNumber("RotorTrim/turretHood", 0);
  }

  public static final class FieldZones {
    public static final double blueLineX = 182.11; // Inches
    public static final double blockedZoneLength = 72; // Inches
    public static final double zoneBuffer = 1; // Meters
    public static final FieldZone TOWER_ZONE =
        new FieldZone(
            -zoneBuffer,
            Units.inchesToMeters(45.0),
            Units.inchesToMeters(110.0),
            Units.inchesToMeters(180.0));
    public static final FieldZone BUMP_ZONE =
        new FieldZone(
            Units.inchesToMeters(180.0),
            Units.inchesToMeters(210.0),
            Units.inchesToMeters(50.0),
            FieldZone.FIELD_WIDTH_METERS - Units.inchesToMeters(50.0));
    public static final FieldZone LEFT_ZONE =
        new FieldZone(
            Units.inchesToMeters(blueLineX),
            FieldZone.FIELD_LENGTH_METERS + zoneBuffer,
            Units.inchesToMeters(317.0 / 2.0),
            FieldZone.FIELD_WIDTH_METERS + zoneBuffer);
    public static final FieldZone RIGHT_ZONE =
        new FieldZone(
            Units.inchesToMeters(blueLineX),
            FieldZone.FIELD_LENGTH_METERS + zoneBuffer,
            -zoneBuffer,
            Units.inchesToMeters(317.0 / 2.0));
    public static final FieldZone BLOCKED_ZONE =
        new FieldZone(
            Units.inchesToMeters(blueLineX),
            Units.inchesToMeters(blueLineX + blockedZoneLength),
            Units.inchesToMeters(317.0 / 2.0 - 40.0),
            Units.inchesToMeters(317.0 / 2.0 + 40.0));
  }

  public static final class Cameras {
    public static final Pose3d blueCalibrationPose = new Pose3d(1.547, 4.0, 0, Rotation3d.kZero);

    public static final Matrix<N3, N3> simIntrinsics =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            916.965230021908,
            0.0,
            661.6928938560056,
            0.0,
            916.8276406094255,
            435.5533504564346,
            0.0,
            0.0,
            1.0);

    public static final Matrix<N8, N1> simDistCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.05009856981900392,
            -0.07369910749297018,
            -1.0660525417228317E-5,
            1.3422933851637837E-4,
            0.009667561013012865,
            0.0,
            0.0,
            0.0);

    public static final class LeftCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-8.5),
                  Units.inchesToMeters(11.375),
                  Units.inchesToMeters(26)),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(90)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800, simIntrinsics, simDistCoeffs),
          };
    }

    public static final class BackLeftCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-13), Units.inchesToMeters(1), Units.inchesToMeters(25.5)),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(150)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800, simIntrinsics, simDistCoeffs),
          };
    }

    public static final class BackRightCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-13), Units.inchesToMeters(-1), Units.inchesToMeters(25.5)),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(-150)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800, simIntrinsics, simDistCoeffs),
          };
    }

    public static final class RightCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-8.5),
                  Units.inchesToMeters(-11.375),
                  Units.inchesToMeters(26)),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(-90)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800, simIntrinsics, simDistCoeffs),
          };
    }
  }

  public static final class IMU {
    public static final CANDeviceID pigeonID = new CANDeviceID(0, canivoreBus);
    public static final double gyroTrimZ = 0; // TODO(2026): Tune
  }

  public static final class Intake {
    public static final CANDeviceID leftWristMotorID = new CANDeviceID(11, canivoreBus);
    public static final CANDeviceID rightWristMotorID = new CANDeviceID(12, canivoreBus);
    public static final CANDeviceID leftRollerMotorID = new CANDeviceID(13, canivoreBus);
    public static final CANDeviceID rightRollerMotorID = new CANDeviceID(14, canivoreBus);

    public static final double intakeWheelDiameter = Units.inchesToMeters(2.0);
    public static final double intakeWheelCircumfrence = intakeWheelDiameter * Math.PI;

    // 12:24 * 24:24
    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(12 * 24, 24 * 24, intakeWheelCircumfrence);

    // 8:38 * 18:38 * 12:36
    public static final MechanismRatio wristMotorRatio =
        new MechanismRatio(8 * 18 * 12, 38 * 38 * 36);

    public static final boolean leftRollerMotorInvert = false;
    public static final boolean rightRollerMotorInvert = true;

    public static final boolean leftWristMotorInvert = true;
    public static final boolean rightWristMotorInvert = false;

    // TODO: Update values
    public static final double maxWristVelocity = 4.0;
    public static final double slowMaxWristVelocity = 2.0;

    public static final double maxWristAcceleration = 45;

    public static final double maxWristJerk = 0.0;

    public static final double leftWristRotorBootOffset = 0.012;
    public static final double rightWristRotorBootOffset = 0.057;
    public static final double minAngle = Math.toRadians(-5);
    public static final double maxAngle = Math.toRadians(98);
    public static final double startingAngle = maxAngle;
    public static final double stowAngle = Math.toRadians(80);
    public static final double standbyAngle = Math.toRadians(10);
    public static final double deployAngle = Math.toRadians(-5);

    public static final double wristTolerance = Math.toRadians(30.0);

    // Roller position control params
    public static final double maxRollerVelocity = 0.75; // m/s
    public static final double maxRollerAcceleration = 100.0; // m/s/s
    public static final double maxRollerJerk = 0.0; // m/s/s/s

    public static final double rollerPercentOutput = 1.0;
    public static final double rollerUnjamPercentOutput = -0.3;

    // Sim stuff
    public static final double simWristMOI = 0.059; // k * m^2
    public static final double simWristCGLength = Units.inchesToMeters(4.0);

    public static final double simRollerMOI = 0.0005; // k * m^2
  }

  public static final class Swerve {
    public static final double defaultDriveSupplyLimit = 40.0; // amps
    public static final double reducedDriveSupplyLimit = 20.0; // amps

    public static final double maxDriveSpeed = 4.5; // m/s
    public static final double maxDriveAcceleration = 8.0; // m/s/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double maxAngularAcceleration = Math.PI * 4.0; // rad/s/s
    public static final double trackWidth = Units.inchesToMeters(22.00);
    public static final double wheelBase = Units.inchesToMeters(22.00);
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio =
        new MechanismRatio(1.0, (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0), wheelCircumference);
    public static final MechanismRatio steeringRatio =
        new MechanismRatio(1.0, (22.0 / 10.0) * (88.0 / 16.0));
    public static final double steerDriveCouplingRatio = 54.0 / 10.0;
    public static final PIDConfig driveOpenLoopPIDConfig =
        new PIDConfig(0.0, 0.0, 0.0, 0.1, 0.125, 0.0, 0.0);
    public static final PIDConfig driveClosedLoopPIDConfig =
        new PIDConfig(0.3, 0.0, 0.0, 0.1, 0.125, 0.005, 0.0);
    public static final PIDConfig steeringPIDConfig =
        Robot.isSimulation() ? new PIDConfig(1.5, 0.0, 0.0) : new PIDConfig(5.0, 0.0, 0.01);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 60.0; // m/s/s
    public static final double angularSlewRate = 120.0; // rad/s/s
    public static final double stickDeadband = 0.05;

    /* Module Slew Rate Limits */
    public static final double maxModuleAcceleration = 60.0; // m/s/s
    public static final double maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.25; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final QuixSwerveController driveController =
        new QuixSwerveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(4.0, 0.0, 0.0));

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(1, canivoreBus);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(2, canivoreBus);
      public static final CANDeviceID canCoderID = new CANDeviceID(1, canivoreBus);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.565430 * 2.0 * Math.PI;
    }

    /* Rear Left Module - Module 1 */
    public static final class RearLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(3, canivoreBus);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(4, canivoreBus);
      public static final CANDeviceID canCoderID = new CANDeviceID(2, canivoreBus);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.966309 * 2.0 * Math.PI;
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(5, canivoreBus);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(6, canivoreBus);
      public static final CANDeviceID canCoderID = new CANDeviceID(3, canivoreBus);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.009766 * 2.0 * Math.PI;
    }

    /* Front Right Module - Module 3 */
    public static final class FrontRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(7, canivoreBus);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(8, canivoreBus);
      public static final CANDeviceID canCoderID = new CANDeviceID(4, canivoreBus);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.339844 * 2.0 * Math.PI;
    }
  }

  public static final class Turret {
    // TODO: change placeholder yaw motor ratio
    public static final CANDeviceID yawMotorID = new CANDeviceID(31, canivoreBus);
    public static final MechanismRatio yawMotorRatio = new MechanismRatio(10 * 10, 135 * 24);
    public static final boolean yawMotorInvert = true;

    public static final CANDeviceID hoodMotorID = new CANDeviceID(32, canivoreBus);
    public static final MechanismRatio hoodMotorRatio = new MechanismRatio(8 * 10, 50 * 165);
    public static final boolean hoodMotorInvert = true;

    public static final CANDeviceID launchMotor1ID = new CANDeviceID(33, canivoreBus);
    public static final boolean launchMotor1Invert = false;
    public static final CANDeviceID launchMotor2ID = new CANDeviceID(34, canivoreBus);
    public static final boolean launchMotor2Invert = true;
    public static final MechanismRatio launchMotorRatio = new MechanismRatio(18, 15);

    public static final double yawMotorBootOffset = -0.104;
    public static final double minYawAngle = Math.toRadians(50);
    public static final double maxYawAngle = Math.toRadians(310);
    public static final double startingYawAngle = Math.toRadians(180);

    public static final double hoodMotorBootOffset = 0.04;
    // Note: these are hood angles, not launch angles. Launch angle is 90 minus angle.
    public static final double minHoodAngle = Math.toRadians(15);
    public static final double maxHoodAngle = Math.toRadians(35);
    public static final double startingHoodAngle = minHoodAngle;

    public static final double launchVelocityTolerance = 0.8; // m/s
    public static final double launchAngleTolerance = Math.toRadians(3); // radians
    public static final double yawTolerance = Math.toRadians(5.0); // radians

    public static final double feedVelocityTolerance = 2.0; // m/s
    public static final double feedAngleTolerance = Math.toRadians(15); // radians
    public static final double feedYawTolerance = Math.toRadians(40.0); // radians

    public static final Transform3d robotToTurretBaseT =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(-1),
            Units.inchesToMeters(27.5),
            Rotation3d.kZero);
    public static final Transform2d robotToTurretBaseT2d =
        new Transform2d(
            Units.inchesToMeters(0), Units.inchesToMeters(-1), Rotation2d.fromDegrees(0));
    public static final Transform3d turretBaseToHoodT =
        new Transform3d(
            Units.inchesToMeters(2.931),
            Units.inchesToMeters(0),
            Units.inchesToMeters(-3.5),
            Rotation3d.kZero);

    // Turret targets (center of hub)
    // - Red side:  (x, y) = (469.11 in, 158.845 in)
    // - Blue side: (x, y) = (182.11 in, 158.845 in)
    public static final Pose2d blueHubCenterPose =
        new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.845), Rotation2d.kZero);
    public static final Pose2d redHubCenterPose =
        new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.845), Rotation2d.kZero);

    // Feed determining parameters
    public static final double blueZoneXMax = Units.inchesToMeters(182.11); // in
    public static final double redZoneXMin = Units.inchesToMeters(469.11); // in
    public static final double feedYMin = Units.inchesToMeters((317.0 / 2.0) - 40.0); // in
    public static final double feedYMax = Units.inchesToMeters((317.0 / 2.0) + 40.0); // in

    // Feed targeting poses (field corners offset inwards)
    public static final double feedXOffset = Units.inchesToMeters(100.0);
    public static final double feedYOffset = Units.inchesToMeters(60.0);
    public static final Pose2d blueRightFeedPose =
        new Pose2d(feedXOffset, feedYOffset, Rotation2d.kZero);
    public static final Pose2d blueLeftFeedPose =
        new Pose2d(feedXOffset, Units.inchesToMeters(317.0) - feedYOffset, Rotation2d.kZero);
    public static final Pose2d redRightFeedPose =
        new Pose2d(
            Units.inchesToMeters(651.25) - feedXOffset,
            Units.inchesToMeters(317.0) - feedYOffset,
            Rotation2d.kZero);
    public static final Pose2d redLeftFeedPose =
        new Pose2d(Units.inchesToMeters(651.25) - feedXOffset, feedYOffset, Rotation2d.kZero);

    public static final double feedingAngle = Math.toRadians(55);

    // TODO change placeholder numbers for default
    public static final LoggedNetworkNumber launchVelocity =
        new LoggedNetworkNumber("TurretParams/LaunchVelocity", 7.1);
    public static final LoggedNetworkNumber shotAngle =
        new LoggedNetworkNumber("TurretParams/ShotAngle", 69);

    // For trimming
    public static final LoggedNetworkNumber launchVelocityMultiplier =
        new LoggedNetworkNumber("TurretParams/VelocityMultiplier (%)", 0.95);
    public static final LoggedNetworkNumber yawTrim =
        new LoggedNetworkNumber("TurretParams/yawTrim (deg)", 0.0);

    public static final LoggedNetworkBoolean useVisionYaw =
        new LoggedNetworkBoolean("Temp/visionYaw", true);

    private static final double[][] launchVelocityEntries = {
      {0.0, 0.0}, // m/s to rad/s
      {5.54, 220.0},
      {7.88, 360.0},
      {9.0, 480.0},
      {10.0, 510.0},
      {12.0, 600.0},
    };

    public static final double maxLaunchVelocityRadsPerSec =
        launchVelocityEntries[launchVelocityEntries.length - 1][1];

    // Lookup of desired exit velocity in m/s to rad/s
    public static final InterpolatingDoubleTreeMap linearVelocityToRadsPerSecLookup =
        new InterpolatingDoubleTreeMap();

    // Lookup of rad/s to exit velocity in m/s
    public static final InterpolatingDoubleTreeMap radsPerSecToLinearVelocityLookup =
        new InterpolatingDoubleTreeMap();

    static {
      for (double[] point : launchVelocityEntries) {
        linearVelocityToRadsPerSecLookup.put(point[0], point[1]);
        radsPerSecToLinearVelocityLookup.put(point[1], point[0]);
      }
    }

    // Sim stuff
    public static final double simTurretMOI = 0.15; // k * m^2
    public static final double simHoodMOI = 0.015; // k * m^2
    public static final double simLaunchMOI = 0.003; // k * m^2
  }

  public static final class Serializer {
    public static final double effectiveRotorRadius = Units.inchesToMeters(7.8);
    public static final double rollerWheelDiameter = Units.inchesToMeters(1.625);
    public static final double rollerWheelRadius = 0.5 * rollerWheelDiameter;

    public static final CANDeviceID rotorMotorID = new CANDeviceID(21, canivoreBus);
    public static final MechanismRatio rotorMotorRatio = new MechanismRatio(1 * 10, 4 * 53);
    public static final boolean rotorMotorInvert = false;
    public static final CANDeviceID rollerMotorID = new CANDeviceID(22, canivoreBus);
    public static final MechanismRatio rollerMotorRatio = new MechanismRatio(10 * 16, 16 * 16);
    public static final boolean rollerMotorInvert = false;

    // TODO change placeholder numbers for default
    public static final LoggedNetworkNumber rollerVelocity =
        new LoggedNetworkNumber("SerializerParams/SerializerRollerVelocity", 3.6);
    public static final LoggedNetworkNumber feedingRollerVelocity =
        new LoggedNetworkNumber("SerializerParams/SerializerFeedingRollerVelocity", 3.6);
    public static final double rollerUnjamVelocity = -0.5;

    // Sim stuff
    public static final double simRotorMOI = 0.03; // k * m^2
    public static final double simRollerMOI = 0.0005; // k * m^2
  }

  public static final class Viz3d {
    public static final Transform3d robotToIntakeLeftArm =
        new Transform3d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(13.0),
            Units.inchesToMeters(8.125),
            Rotation3d.kZero);
    public static final Transform3d robotToIntakeRightArm =
        new Transform3d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(-13.0),
            Units.inchesToMeters(8.125),
            Rotation3d.kZero);
  }

  public static final class Example {
    public static final CANDeviceID motorID = new CANDeviceID(62, canivoreBus);
    public static final MechanismRatio motorRatio = new MechanismRatio(0, 0, 0);
    public static final boolean motorInvert = false;
  }
}
