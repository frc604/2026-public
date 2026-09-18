package frc.quixlib.devices;

public interface QuixIMU {
  // ==================== Setters ====================

  /** Sets the continuous yaw zero-point to the current position. */
  public void zeroContinuousYaw();

  /** Sets the continuous yaw to the given value in radians. */
  public void setContinuousYaw(double rad);

  // ==================== Getters ====================

  /** Returns the roll (rotation around +X axis) in radians. Roll to the right is positive. */
  public double getRoll();

  /** Returns the pitch (rotation around +Y axis) in radians. Pitch down is positive. */
  public double getPitch();

  /** Returns the continuous yaw in radians. CCW is positive. */
  public double getContinuousYaw();

  /**
   * Returns the roll rate (rotation around +X axis) in radians per sec. Roll to the right is
   * positive.
   */
  public double getRollRate();

  /**
   * Returns the pitch rate (rotation around +Y axis) in radians per sec. Pitch down is positive.
   */
  public double getPitchRate();

  /** Returns the yaw rate in radians per sec. CCW is positive. */
  public double getYawRate();

  /** Returns the X-axis linear acceleration in m/s^2. */
  public double getAccelX();

  /** Returns the Y-axis linear acceleration in m/s^2. */
  public double getAccelY();

  /** Returns the Z-axis linear acceleration in m/s^2. */
  public double getAccelZ();

  // ==================== Simulation ====================

  /** Sets the simulated continuous yaw in radians. Note that updates must be continuous. */
  public void setSimContinuousYaw(double rad);

  /** Sets the simulated X-axis linear acceleration in m/s^2. */
  public void setSimAccelX(double metersPerSecondSquared);

  /** Sets the simulated Y-axis linear acceleration in m/s^2. */
  public void setSimAccelY(double metersPerSecondSquared);

  /** Sets the simulated Z-axis linear acceleration in m/s^2. */
  public void setSimAccelZ(double metersPerSecondSquared);
}
