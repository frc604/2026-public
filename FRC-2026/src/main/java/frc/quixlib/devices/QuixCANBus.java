package frc.quixlib.devices;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * The QuixCANBus class represents a CAN bus interface for the robot. It provides functionality to
 * log and update the status of the CAN bus.
 *
 * <p>It includes an inner static class CANBusInputs which holds the status and various metrics of
 * the CAN bus.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * QuixCANBus canBus = new QuixCANBus();
 * canBus.updateInputs();
 * }</pre>
 *
 * <p>Constructor Summary:
 *
 * <ul>
 *   <li>{@link #QuixCANBus()} - Initializes the CAN bus with the default name "rio".
 *   <li>{@link #QuixCANBus(String canbusName)} - Initializes the CAN bus with the specified name.
 * </ul>
 *
 * <p>Method Summary:
 *
 * <ul>
 *   <li>{@link #updateInputs()} - Updates the CAN bus inputs and logs the current status.
 * </ul>
 *
 * <p>Inner Class:
 *
 * <ul>
 *   <li>{@link QuixCANBus.CANBusInputs} - Represents the inputs for a CAN bus.
 * </ul>
 */
public class QuixCANBus implements QuixIO {
  private final String m_canbusName;
  private final String m_loggingName;
  private final CANBus m_canBus;

  private final CANBusInputsAutoLogged m_inputs = new CANBusInputsAutoLogged();

  /** Represents the inputs for a CAN bus. */
  @AutoLog
  public static class CANBusInputs {
    protected StatusCode status = StatusCode.OK;
    protected double busUtilization = 0.0;
    protected int busOffCount = 0;
    protected int txFullCount = 0;
    protected int REC = 0;
    protected int TEC = 0;
    protected boolean isNetworkFD = false;
  }

  public QuixCANBus() {
    this("rio");
  }

  public QuixCANBus(final String canbusName) {
    m_canbusName = canbusName;
    m_loggingName = "Inputs/CANBus [" + m_canbusName + "]";
    m_canBus = new CANBus(canbusName);

    QuixIOUtil.registerDevice(this);
  }

  public String getName() {
    return m_canbusName;
  }

  public CANBus rawCANBus() {
    return m_canBus;
  }

  public void updateInputs() {
    CANBusStatus status = m_canBus.getStatus();
    m_inputs.status = status.Status;
    m_inputs.busUtilization = status.BusUtilization;
    m_inputs.busOffCount = status.BusOffCount;
    m_inputs.txFullCount = status.TxFullCount;
    m_inputs.REC = status.REC;
    m_inputs.TEC = status.TEC;
    m_inputs.isNetworkFD = m_canBus.isNetworkFD();
    Logger.processInputs(m_loggingName, m_inputs);
  }
}
