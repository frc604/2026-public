package frc.quixlib.devices;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */
public class CANDeviceID {
  /** Default CAN bus name for the RoboRIO */
  public static final String kRIOCANbusName = "rio";

  /** The device number on the CAN bus */
  public final int deviceNumber;

  /** The CAN bus this device is on */
  public final QuixCANBus CANBus;

  /**
   * Creates a new CAN device ID with specified device number and bus name.
   *
   * @param deviceNumber The device's CAN ID number
   * @param CANBus The CAN bus
   */
  public CANDeviceID(final int deviceNumber, final QuixCANBus CANBus) {
    this.deviceNumber = deviceNumber;
    this.CANBus = CANBus;
  }

  /**
   * Creates a new CAN device ID on the default RIO CAN bus.
   *
   * @param deviceNumber The device's CAN ID number
   */
  public CANDeviceID(final int deviceNumber) {
    this(deviceNumber, new QuixCANBus(kRIOCANbusName));
  }

  /**
   * @return String representation of the CAN device ID in format [busName deviceNumber]
   */
  @Override
  public String toString() {
    return "[" + CANBus.getName() + " " + deviceNumber + "]";
  }
}
