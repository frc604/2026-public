package frc.quixlib.motorcontrol;

/** Defines a ratio and distance per rotation between motor and mechanism. */
/**
 * The MechanismRatio class represents the ratio between the driving and driven gears in a
 * mechanism. It provides methods to calculate the reduction ratio, distance per rotation, and
 * conversions between sensor position in radians and mechanism position. Distance per rotation is
 * an optional parameter and defaults to 2 * PI radians.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * MechanismRatio ratio = new MechanismRatio(10, 50);
 * double reduction = ratio.reduction();
 * double distance = ratio.distancePerRotation();
 * double mechanismPos = ratio.sensorRadiansToMechanismPosition(sensorRadians);
 * double sensorPos = ratio.mechanismPositionToSensorRadians(mechanismPos);
 * }</pre>
 *
 * <p>Constructors:
 *
 * <ul>
 *   <li>{@link #MechanismRatio()} - Default constructor with 1:1 ratio and default distance per
 *       rotation.
 *   <li>{@link #MechanismRatio(double, double)} - Constructor with specified driving and driven
 *       teeth.
 *   <li>{@link #MechanismRatio(double, double, double)} - Constructor with specified driving and
 *       driven teeth, and distance per rotation.
 * </ul>
 *
 * <p>Methods:
 *
 * <ul>
 *   <li>{@link #reduction()} - Returns the reduction ratio (driven / driving).
 *   <li>{@link #distancePerRotation()} - Returns the distance per rotation of the driven gear.
 *   <li>{@link #sensorRadiansToMechanismPosition(double)} - Converts sensor position in radians to
 *       mechanism position.
 *   <li>{@link #mechanismPositionToSensorRadians(double)} - Converts mechanism position to sensor
 *       position in radians.
 * </ul>
 */
public class MechanismRatio {
  private static final double kDefaultDistancePerRotation = 2.0 * Math.PI; // Default to radians;

  private final double m_driving;
  private final double m_driven;
  private final double m_distancePerRotation;

  public MechanismRatio() {
    this(1.0, 1.0);
  }

  public MechanismRatio(double drivingTeeth, double drivenTeeth) {
    this(drivingTeeth, drivenTeeth, kDefaultDistancePerRotation);
  }

  public MechanismRatio(double drivingTeeth, double drivenTeeth, double distancePerRotation) {
    m_driving = drivingTeeth;
    m_driven = drivenTeeth;
    m_distancePerRotation = distancePerRotation;
  }

  /**
   * Returns the mechanism ratio defined as: driven / driving. Numbers greater than 1 represent
   * reductions.
   */
  public double reduction() {
    return m_driven / m_driving;
  }

  /** Returns the distance per rotation of the driven gear. */
  public double distancePerRotation() {
    return m_distancePerRotation;
  }

  /** Returns the mechanism position for the given sensor position in radians. */
  public double sensorRadiansToMechanismPosition(double sensorRadians) {
    final double distancePerRadian = m_distancePerRotation / (2.0 * Math.PI);
    return sensorRadians * distancePerRadian / reduction();
  }

  /** Returns the sensor position in radians for the given mechanism position. */
  public double mechanismPositionToSensorRadians(double mechanismPos) {
    return mechanismPos / sensorRadiansToMechanismPosition(1.0);
  }
}
