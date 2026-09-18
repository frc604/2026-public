package frc.quixlib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.quixlib.phoenix.PhoenixUtil;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * The QuixPigeon2 class is an implementation of the PhoenixIO and QuixIMU interfaces, providing an
 * interface to the Pigeon2 IMU sensor. It includes methods for configuring the sensor, updating
 * input values, and retrieving various sensor readings such as roll, pitch, yaw, and their
 * respective rates.
 *
 * <p>Features:
 *
 * <ul>
 *   <li>Configuration of the Pigeon2 sensor with custom settings.
 *   <li>Automatic logging of sensor inputs.
 *   <li>Methods to retrieve roll, pitch, yaw, and their rates.
 *   <li>Support for continuous yaw with offset adjustment.
 *   <li>Simulation support for continuous yaw.
 * </ul>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // Create a QuixPigeon2 instance with default configuration
 * QuixPigeon2 pigeon = new QuixPigeon2(canID);
 *
 * // Update inputs and retrieve yaw
 * pigeon.updateInputs();
 * double yaw = pigeon.getContinuousYaw();
 * }</pre>
 *
 * <p>Configuration:
 *
 * <pre>{@code
 * // Create a custom configuration
 * QuixPigeon2.QuixPigeon2Configuration config = new QuixPigeon2.QuixPigeon2Configuration();
 * config.setGyroTrimZ(Math.toRadians(5.0));
 *
 * // Create a QuixPigeon2 instance with custom configuration
 * QuixPigeon2 pigeon = new QuixPigeon2(canID, config);
 * }</pre>
 *
 * <p>Methods:
 *
 * <ul>
 *   <li>{@link #setConfiguration()} - Applies the configuration to the Pigeon2 sensor.
 *   <li>{@link #updateInputs()} - Updates the input values from the sensor.
 *   <li>{@link #waitForInputs(double)} - Waits for input values to be updated with a timeout.
 *   <li>{@link #zeroContinuousYaw()} - Resets the continuous yaw to zero.
 *   <li>{@link #setContinuousYaw(double)} - Sets the continuous yaw to a specified value.
 *   <li>{@link #getRoll()} - Retrieves the roll value.
 *   <li>{@link #getPitch()} - Retrieves the pitch value.
 *   <li>{@link #getContinuousYaw()} - Retrieves the continuous yaw value.
 *   <li>{@link #getLatencyCompensatedContinuousYaw()} - Retrieves the latency-compensated
 *       continuous yaw value.
 *   <li>{@link #getRollRate()} - Retrieves the roll rate value.
 *   <li>{@link #getPitchRate()} - Retrieves the pitch rate value.
 *   <li>{@link #getYawRate()} - Retrieves the yaw rate value.
 *   <li>{@link #setSimContinuousYaw(double)} - Sets the simulated continuous yaw value.
 * </ul>
 *
 * <p>Inner Classes:
 *
 * <ul>
 *   <li>{@link QuixPigeon2.Pigeon2Inputs} - Holds the input values from the Pigeon2 sensor.
 *   <li>{@link QuixPigeon2.QuixPigeon2Configuration} - Configuration class for the Pigeon2 sensor.
 * </ul>
 *
 * @see QuixIO
 * @see QuixIMU
 */
public class QuixPigeon2 implements QuixIO, QuixIMU {
  private static final double kCANTimeoutS = 0.1; // s
  private final String m_name;
  private final String m_loggingName;
  private final Pigeon2 m_pigeon;
  private final Pigeon2SimState m_simState;
  private final QuixPigeon2Configuration m_config;

  private final QuixStatusSignal<Integer> m_faultFieldSignal;
  private final QuixStatusSignal<Integer> m_stickyFaultFieldSignal;
  private final QuixStatusSignal<Angle> m_rollSignal;
  private final QuixStatusSignal<Angle> m_pitchSignal;
  private final QuixStatusSignal<Angle> m_yawSignal;
  private final QuixStatusSignal<AngularVelocity> m_rollRateSignal;
  private final QuixStatusSignal<AngularVelocity> m_pitchRateSignal;
  private final QuixStatusSignal<AngularVelocity> m_yawRateSignal;
  private final QuixStatusSignal<LinearAcceleration> m_accelXSignal;
  private final QuixStatusSignal<LinearAcceleration> m_accelYSignal;
  private final QuixStatusSignal<LinearAcceleration> m_accelZSignal;
  private final BaseStatusSignal[] m_allSignals;

  private static final double kGravity = 9.81; // m/s^2

  private double continuousYawOffset = 0.0;

  // Simulation state for acceleration (bypasses Pigeon2SimState which doesn't support accel)
  private double m_simAccelX = 0.0; // m/s^2
  private double m_simAccelY = 0.0; // m/s^2
  private double m_simAccelZ = 0.0; // m/s^2

  private final Pigeon2InputsAutoLogged m_inputs = new Pigeon2InputsAutoLogged();

  /**
   * Data structure to hold Pigeon2 IMU sensor inputs. Contains status information, fault codes, and
   * orientation/rate measurements.
   *
   * @field status Current status code of the Pigeon2
   * @field faultField Active fault codes as a bit field
   * @field stickyFaultField Sticky fault codes as a bit field
   * @field roll Current roll angle in degrees
   * @field pitch Current pitch angle in degrees
   * @field yaw Current yaw angle in degrees
   * @field latencyCompensatedYaw Latency-compensated yaw angle in degrees
   * @field rollRate Angular velocity around roll axis in degrees per second
   * @field pitchRate Angular velocity around pitch axis in degrees per second
   * @field yawRate Angular velocity around yaw axis in degrees per second
   */
  @AutoLog
  public static class Pigeon2Inputs {
    protected int faultField = 0;
    protected int stickyFaultField = 0;
    protected double roll = 0.0;
    protected double pitch = 0.0;
    protected double yaw = 0.0;
    protected double latencyCompensatedYaw = 0.0;
    protected double rollRate = 0.0;
    protected double pitchRate = 0.0;
    protected double yawRate = 0.0;
    protected double accelX = 0.0; // m/s^2
    protected double accelY = 0.0; // m/s^2
    protected double accelZ = 0.0; // m/s^2
  }

  /**
   * Configuration class for QuixPigeon2 that handles mount pose and gyro trim settings. All angle
   * measurements are in radians internally but converted to degrees when creating the
   * Pigeon2Configuration.
   *
   * <p>Mount pose represents the physical orientation of the Pigeon2 sensor relative to the robot,
   * while gyro trim values are used for calibration adjustments.
   *
   * @see com.ctre.phoenix.sensors.Pigeon2Configuration
   */
  public static class QuixPigeon2Configuration {
    private double mountPoseRoll = 0.0; // rads
    private double mountPosePitch = 0.0; // rads
    private double mountPoseYaw = 0.0; // rads
    private double gyroTrimX = 0.0; // rads
    private double gyroTrimY = 0.0; // rads
    private double gyroTrimZ = 0.0; // rads

    public QuixPigeon2Configuration setGyroTrimZ(final double rads) {
      gyroTrimZ = rads;
      return this;
    }

    public Pigeon2Configuration toPigeon2Configuration() {
      Pigeon2Configuration config = new Pigeon2Configuration();

      config.MountPose.MountPoseRoll = Math.toDegrees(mountPoseYaw);
      config.MountPose.MountPosePitch = Math.toDegrees(mountPosePitch);
      config.MountPose.MountPoseYaw = Math.toDegrees(mountPoseRoll);

      config.GyroTrim.GyroScalarX = Math.toDegrees(gyroTrimX);
      config.GyroTrim.GyroScalarY = Math.toDegrees(gyroTrimY);
      config.GyroTrim.GyroScalarZ = Math.toDegrees(gyroTrimZ);

      return config;
    }
  }

  public static QuixPigeon2Configuration makeDefaultConfig() {
    return new QuixPigeon2Configuration();
  }

  /** Default constructor */
  public QuixPigeon2(final CANDeviceID canID) {
    this(canID, makeDefaultConfig());
  }

  /**
   * A wrapper class for CTRE's Pigeon2 IMU sensor with additional functionality.
   *
   * @param canID The CAN ID configuration for the Pigeon2 device
   * @param config The configuration settings for the Pigeon2
   *     <p>The constructor: - Initializes the Pigeon2 device with the given CAN ID - Sets up status
   *     signals for faults, orientation (roll, pitch, yaw), and angular velocities - Converts
   *     angular measurements from degrees to radians - Clears any previous reset flags and sticky
   *     faults - Logs the device configuration
   */
  public QuixPigeon2(final CANDeviceID canID, final QuixPigeon2Configuration config) {
    m_name = "Pigeon2 " + canID.toString();
    m_loggingName = "Inputs/" + m_name;
    m_pigeon = new Pigeon2(canID.deviceNumber, canID.CANBus.rawCANBus());
    m_simState = m_pigeon.getSimState();
    m_config = config;

    m_faultFieldSignal = new QuixStatusSignal<>(m_pigeon.getFaultField());
    m_stickyFaultFieldSignal = new QuixStatusSignal<>(m_pigeon.getStickyFaultField());
    m_rollSignal = new QuixStatusSignal<>(m_pigeon.getRoll(), Math::toRadians);
    m_pitchSignal = new QuixStatusSignal<>(m_pigeon.getPitch(), Math::toRadians);
    m_yawSignal =
        new QuixStatusSignal<>(
            m_pigeon.getYaw(),
            (Double value) -> {
              return Math.toRadians(value) - continuousYawOffset;
            });
    m_rollRateSignal =
        new QuixStatusSignal<>(m_pigeon.getAngularVelocityXDevice(), Math::toRadians);
    m_pitchRateSignal =
        new QuixStatusSignal<>(m_pigeon.getAngularVelocityYDevice(), Math::toRadians);
    m_yawRateSignal = new QuixStatusSignal<>(m_pigeon.getAngularVelocityZDevice(), Math::toRadians);
    // Acceleration signals: convert from g's to m/s^2
    m_accelXSignal =
        new QuixStatusSignal<>(m_pigeon.getAccelerationX(), (Double g) -> g * kGravity);
    m_accelYSignal =
        new QuixStatusSignal<>(m_pigeon.getAccelerationY(), (Double g) -> g * kGravity);
    m_accelZSignal =
        new QuixStatusSignal<>(m_pigeon.getAccelerationZ(), (Double g) -> g * kGravity);
    m_allSignals =
        QuixStatusSignal.toBaseStatusSignals(
            m_faultFieldSignal,
            m_stickyFaultFieldSignal,
            m_rollSignal,
            m_pitchSignal,
            m_yawSignal,
            m_rollRateSignal,
            m_pitchRateSignal,
            m_yawRateSignal,
            m_accelXSignal,
            m_accelYSignal,
            m_accelZSignal);

    // Clear reset flag and sticky faults.
    m_pigeon.hasResetOccurred();
    m_pigeon.clearStickyFaults();

    Logger.recordOutput("Configuration/" + m_name, setConfiguration());

    PhoenixUtil.registerSignals(m_allSignals);
    QuixIOUtil.registerDevice(this);
  }

  /**
   * Configures the Pigeon2 IMU device with specified settings and update frequencies.
   *
   * <p>This method: 1. Applies the Pigeon2 configuration 2. Sets update frequencies for fault
   * signals (4Hz) 3. Sets update frequencies for motion signals (100Hz) 4. Waits for valid input
   * signals 5. Checks for unlicensed feature usage
   *
   * <p>Note: Bus utilization optimization is currently disabled due to potential stale signal
   * issues.
   *
   * @return boolean indicating whether all configuration steps were successful (true) or if any
   *     failed (false)
   */
  public boolean setConfiguration() {
    boolean allSuccess = true;

    // Set configuration.
    Pigeon2Configuration config = m_config.toPigeon2Configuration();
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pigeon.getConfigurator().apply(config, kCANTimeoutS),
            () -> {
              Pigeon2Configuration readConfig = new Pigeon2Configuration();
              m_pigeon.getConfigurator().refresh(readConfig, kCANTimeoutS);
              return PhoenixUtil.Pigeon2ConfigsEqual(config, readConfig);
            },
            m_name + ": applyConfiguration");

    // Set update frequencies.
    final double kFaultUpdateFrequency = 4.0; // Hz
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_faultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> m_faultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            m_name + ": m_faultFieldSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_stickyFaultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> m_stickyFaultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            m_name + ": m_stickyFaultFieldSignal.setUpdateFrequency()");

    final double kUpdateFrequency = 100.0; // Hz
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_rollSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_rollSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_rollSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pitchSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_pitchSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_pitchSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_yawSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_yawSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_yawSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_rollRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_rollRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_rollRateSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_pitchRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_pitchRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_pitchRateSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_yawRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_yawRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_yawRateSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_accelXSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_accelXSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_accelXSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_accelYSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_accelYSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_accelYSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_accelZSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_accelZSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_accelZSignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    // TODO: Figure out why this sometimes causes stale signals.
    // allSuccess &=
    //     PhoenixUtil.retryUntilSuccess(
    //         () -> m_pigeon.optimizeBusUtilization(kCANTimeoutS),
    //         m_name + ": optimizeBusUtilization");

    // Block until we get valid signals.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> BaseStatusSignal.waitForAll(kCANTimeoutS, m_allSignals),
            m_name + ": waitForAll()");

    // Check if unlicensed.
    allSuccess &= !m_pigeon.getStickyFault_UnlicensedFeatureInUse().getValue();

    return allSuccess;
  }

  /**
   * Updates all input values from status signals. This method collects data for roll, pitch, yaw,
   * and their respective rates, as well as fault information from the sensor.
   */
  public void updateInputs() {
    m_inputs.faultField = m_faultFieldSignal.getRawValue();
    m_inputs.stickyFaultField = m_stickyFaultFieldSignal.getRawValue();
    m_inputs.roll = m_rollSignal.getUnitConvertedValue();
    m_inputs.pitch = m_pitchSignal.getUnitConvertedValue();
    m_inputs.yaw = m_yawSignal.getUnitConvertedValue();
    m_inputs.latencyCompensatedYaw =
        QuixStatusSignal.getLatencyCompensatedValue(m_yawSignal, m_yawRateSignal);
    m_inputs.rollRate = m_rollRateSignal.getUnitConvertedValue();
    m_inputs.pitchRate = m_pitchRateSignal.getUnitConvertedValue();
    m_inputs.yawRate = m_yawRateSignal.getUnitConvertedValue();

    // Use simulated acceleration values in simulation mode (Pigeon2SimState doesn't support accel)
    if (Robot.isSimulation()) {
      m_inputs.accelX = m_simAccelX;
      m_inputs.accelY = m_simAccelY;
      m_inputs.accelZ = m_simAccelZ;
    } else {
      m_inputs.accelX = m_accelXSignal.getUnitConvertedValue();
      m_inputs.accelY = m_accelYSignal.getUnitConvertedValue();
      m_inputs.accelZ = m_accelZSignal.getUnitConvertedValue();
    }

    Logger.processInputs(m_loggingName, m_inputs);
  }

  /**
   * Resets the continuous yaw measurement to zero. This is equivalent to calling
   * setContinuousYaw(0.0).
   */
  public void zeroContinuousYaw() {
    setContinuousYaw(0.0);
  }

  /**
   * Sets the continuous yaw (heading) of the Pigeon2 to a specified angle. Adjusts the continuous
   * yaw offset to maintain continuous rotation tracking.
   *
   * @param rad The desired yaw angle in radians to set the Pigeon2 to
   */
  public void setContinuousYaw(final double rad) {
    continuousYawOffset += getContinuousYaw() - rad;
    updateInputs();
  }

  public double getRoll() {
    return m_inputs.roll;
  }

  public double getPitch() {
    return m_inputs.pitch;
  }

  public double getContinuousYaw() {
    return m_inputs.yaw;
  }

  public double getLatencyCompensatedContinuousYaw() {
    return m_inputs.latencyCompensatedYaw;
  }

  public double getRollRate() {
    return m_inputs.rollRate;
  }

  public double getPitchRate() {
    return m_inputs.pitchRate;
  }

  public double getYawRate() {
    return m_inputs.yawRate;
  }

  /** Returns the X-axis linear acceleration in m/s^2. */
  public double getAccelX() {
    return m_inputs.accelX;
  }

  /** Returns the Y-axis linear acceleration in m/s^2. */
  public double getAccelY() {
    return m_inputs.accelY;
  }

  /** Returns the Z-axis linear acceleration in m/s^2. */
  public double getAccelZ() {
    return m_inputs.accelZ;
  }

  public void setSimContinuousYaw(final double rad) {
    m_simState.setRawYaw(Math.toDegrees(rad));
  }

  public void setSimYawRate(final double radsPerSec) {
    m_simState.setAngularVelocityZ(Math.toDegrees(radsPerSec));
  }

  /** Sets the simulated X-axis linear acceleration in m/s^2. */
  public void setSimAccelX(final double metersPerSecondSquared) {
    m_simAccelX = metersPerSecondSquared;
  }

  /** Sets the simulated Y-axis linear acceleration in m/s^2. */
  public void setSimAccelY(final double metersPerSecondSquared) {
    m_simAccelY = metersPerSecondSquared;
  }

  /** Sets the simulated Z-axis linear acceleration in m/s^2. */
  public void setSimAccelZ(final double metersPerSecondSquared) {
    m_simAccelZ = metersPerSecondSquared;
  }
}
