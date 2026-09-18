package frc.quixlib.devices;

/**
 * Interface representing a Quix Absolute Encoder. Provides methods to interact with and simulate an
 * absolute encoder sensor.
 */
public interface QuixAbsoluteEncoder {
  // ==================== Setters ====================

  /** Sets the sensor zero-point to the current position. */
  public void zero();

  /** Sets the sensor position to the given value. Uses MechanismRatio units. */
  public void setPosition(double pos);

  // ==================== Getters ====================

  /** Returns the sensor position. Uses MechanismRatio units. */
  public double getPosition();

  /** Returns the sensor absolute position. Uses MechanismRatio units. */
  public double getAbsPosition();

  /** Returns the sensor velocity. Uses MechanismRatio units. */
  public double getVelocity();

  // ==================== Simulation ====================

  /**
   * Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated
   * position.
   */
  public void setSimSensorVelocity(double vel, double dt);
}
