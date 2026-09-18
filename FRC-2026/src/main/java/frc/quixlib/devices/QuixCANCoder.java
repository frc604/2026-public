package frc.quixlib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.phoenix.PhoenixUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * The QuixCANCoder class implements the PhoenixIO and QuixAbsoluteEncoder interfaces to provide
 * functionality for interacting with a CANcoder device.
 *
 * <p>This class handles the configuration, input updates, and simulation state of the CANcoder. It
 * also provides methods to get and set the position and velocity of the encoder.
 *
 * <p>Constructor:
 *
 * <ul>
 *   <li>{@link #QuixCANCoder(CANDeviceID, MechanismRatio)}: Initializes a new instance of the
 *       QuixCANCoder class with the specified CAN device ID and mechanism ratio.
 * </ul>
 *
 * <p>Public Methods:
 *
 * <ul>
 *   <li>{@link #setConfiguration()}: Configures the CANcoder and sets update frequencies for
 *       various signals.
 *   <li>{@link #updateInputs()}: Updates the input signals from the CANcoder.
 *   <li>{@link #waitForInputs(double)}: Waits for input signals to be updated within the specified
 *       timeout period.
 *   <li>{@link #zero()}: Sets the position of the CANcoder to zero.
 *   <li>{@link #setPosition(double)}: Sets the position of the CANcoder to the specified value.
 *   <li>{@link #getPosition()}: Gets the current position of the CANcoder.
 *   <li>{@link #getAbsPosition()}: Gets the absolute position of the CANcoder.
 *   <li>{@link #getVelocity()}: Gets the current velocity of the CANcoder.
 *   <li>{@link #setSimSensorVelocity(double, double)}: Sets the simulated sensor velocity and
 *       updates the simulation state.
 * </ul>
 *
 * <p>Private Methods:
 *
 * <ul>
 *   <li>{@link #toNativeSensorPosition(double)}: Converts a position value to native sensor units.
 *   <li>{@link #fromNativeSensorPosition(double)}: Converts a native sensor position value to
 *       mechanism units.
 *   <li>{@link #toNativeSensorVelocity(double)}: Converts a velocity value to native sensor units.
 *   <li>{@link #fromNativeSensorVelocity(double)}: Converts a native sensor velocity value to
 *       mechanism units.
 * </ul>
 *
 * <p>Inner Classes:
 *
 * <ul>
 *   <li>{@link CANCoderInputs}: A class representing the input signals from the CANcoder.
 * </ul>
 *
 * <p>Fields:
 *
 * <ul>
 *   <li>{@code kCANTimeoutS}: The timeout value for CAN operations, in seconds.
 *   <li>{@code m_name}: The name of the CANcoder instance.
 *   <li>{@code m_loggingName}: The logging name for the CANcoder instance.
 *   <li>{@code m_cancoder}: The CANcoder device instance.
 *   <li>{@code m_simState}: The simulation state of the CANcoder.
 *   <li>{@code m_ratio}: The mechanism ratio for converting between sensor and mechanism units.
 *   <li>{@code m_faultFieldSignal}: The signal representing the fault field of the CANcoder.
 *   <li>{@code m_stickyFaultFieldSignal}: The signal representing the sticky fault field of the
 *       CANcoder.
 *   <li>{@code m_positionSignal}: The signal representing the position of the CANcoder.
 *   <li>{@code m_absolutePositionSignal}: The signal representing the absolute position of the
 *       CANcoder.
 *   <li>{@code m_velocitySignal}: The signal representing the velocity of the CANcoder.
 *   <li>{@code m_allSignals}: An array of all status signals for the CANcoder.
 *   <li>{@code m_inputs}: An instance of the CANCoderInputs class representing the current input
 *       signals.
 * </ul>
 */
public class QuixCANCoder implements QuixIO, QuixAbsoluteEncoder {
  private static final double kCANTimeoutS = 0.1; // s
  private final String m_name;
  private final String m_loggingName;
  private final CANcoder m_cancoder;
  private final CANcoderSimState m_simState;
  private final MechanismRatio m_ratio;

  private final QuixStatusSignal<Integer> m_faultFieldSignal;
  private final QuixStatusSignal<Integer> m_stickyFaultFieldSignal;
  private final QuixStatusSignal<Angle> m_positionSignal;
  private final QuixStatusSignal<Angle> m_absolutePositionSignal;
  private final QuixStatusSignal<AngularVelocity> m_velocitySignal;
  private final BaseStatusSignal[] m_allSignals;

  private final CANCoderInputsAutoLogged m_inputs = new CANCoderInputsAutoLogged();

  @AutoLog
  public static class CANCoderInputs {
    protected int faultField = 0;
    protected int stickyFaultField = 0;
    protected double position = 0.0;
    protected double absolutePosition = 0.0;
    protected double velocity = 0.0;
  }

  public QuixCANCoder(final CANDeviceID canID, final MechanismRatio ratio) {
    m_name = "CANCoder " + canID.toString();
    m_loggingName = "Inputs/" + m_name;
    m_cancoder = new CANcoder(canID.deviceNumber, canID.CANBus.rawCANBus());
    m_simState = m_cancoder.getSimState();
    m_ratio = ratio;

    m_faultFieldSignal = new QuixStatusSignal<>(m_cancoder.getFaultField());
    m_stickyFaultFieldSignal = new QuixStatusSignal<>(m_cancoder.getStickyFaultField());
    m_positionSignal =
        new QuixStatusSignal<>(m_cancoder.getPosition(), this::fromNativeSensorPosition);
    m_absolutePositionSignal =
        new QuixStatusSignal<>(m_cancoder.getAbsolutePosition(), this::fromNativeSensorPosition);
    m_velocitySignal =
        new QuixStatusSignal<>(m_cancoder.getVelocity(), this::fromNativeSensorVelocity);
    m_allSignals =
        QuixStatusSignal.toBaseStatusSignals(
            m_faultFieldSignal,
            m_stickyFaultFieldSignal,
            m_positionSignal,
            m_absolutePositionSignal,
            m_velocitySignal);

    // Clear reset flag and sticky faults.
    m_cancoder.hasResetOccurred();
    m_cancoder.clearStickyFaults();

    Logger.recordOutput("Configuration/" + m_name, setConfiguration());

    PhoenixUtil.registerSignals(m_allSignals);
    QuixIOUtil.registerDevice(this);
  }

  public boolean setConfiguration() {
    boolean allSuccess = true;

    // Set configuration.
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_cancoder.getConfigurator().apply(config, kCANTimeoutS),
            () -> {
              CANcoderConfiguration readConfig = new CANcoderConfiguration();
              m_cancoder.getConfigurator().refresh(readConfig, kCANTimeoutS);
              return PhoenixUtil.CANcoderConfigsEqual(config, readConfig);
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

    final double kUpdateFrequency = 10.0; // Hz
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_positionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_positionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_positionSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_absolutePositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_absolutePositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_absolutePositionSignal.setUpdateFrequency()");
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_velocitySignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> m_velocitySignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            m_name + ": m_velocitySignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> m_cancoder.optimizeBusUtilization(0.0, kCANTimeoutS),
            m_name + ": optimizeBusUtilization");

    // Block until we get valid signals.
    allSuccess &=
        PhoenixUtil.retryUntilSuccess(
            () -> BaseStatusSignal.waitForAll(kCANTimeoutS, m_allSignals),
            m_name + ": waitForAll()");

    // Check if unlicensed.
    allSuccess &= !m_cancoder.getStickyFault_UnlicensedFeatureInUse().getValue();

    return allSuccess;
  }

  public void updateInputs() {
    m_inputs.faultField = m_faultFieldSignal.getRawValue();
    m_inputs.stickyFaultField = m_stickyFaultFieldSignal.getRawValue();
    m_inputs.position = m_positionSignal.getUnitConvertedValue();
    m_inputs.absolutePosition = m_absolutePositionSignal.getUnitConvertedValue();
    m_inputs.velocity = m_velocitySignal.getUnitConvertedValue();

    Logger.processInputs(m_loggingName, m_inputs);
  }

  public void zero() {
    setPosition(0.0);
  }

  public void setPosition(final double pos) {
    m_cancoder.setPosition(toNativeSensorPosition(pos));
  }

  public double getPosition() {
    return m_inputs.position;
  }

  public double getAbsPosition() {
    return m_inputs.absolutePosition;
  }

  public double getVelocity() {
    return m_inputs.velocity;
  }

  private double toNativeSensorPosition(final double pos) {
    // Native units are in rotations.
    return m_ratio.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
  }

  private double fromNativeSensorPosition(final double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  private double toNativeSensorVelocity(final double vel) {
    return toNativeSensorPosition(vel);
  }

  private double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorVelocity(final double vel, final double dt) {
    final double rotationsPerSecond = toNativeSensorVelocity(vel);
    m_simState.setVelocity(rotationsPerSecond);
    m_simState.addPosition(rotationsPerSecond * dt);
  }
}
