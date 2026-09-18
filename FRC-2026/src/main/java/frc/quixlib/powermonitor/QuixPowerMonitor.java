package frc.quixlib.powermonitor;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.quixlib.motorcontrol.QuixMotorControllerWithEncoder;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class QuixPowerMonitor {
  private static QuixMotorControllerWithEncoder[] m_devices = new QuixMotorControllerWithEncoder[0];
  private static double m_totalEnergy = 0.0;

  public static void registerDevice(final QuixMotorControllerWithEncoder ioDevice) {
    QuixMotorControllerWithEncoder[] newDevices =
        new QuixMotorControllerWithEncoder[m_devices.length + 1];
    System.arraycopy(m_devices, 0, newDevices, 0, m_devices.length);
    newDevices[m_devices.length] = ioDevice;
    m_devices = newDevices;
  }

  public static void logPowerStats() {
    double totalSupplyCurrent = 0.0;
    for (var device : m_devices) {
      totalSupplyCurrent += device.getSupplyCurrent();
    }

    if (RobotBase.isSimulation()) {
      // Calculate sagged battery voltage based on default WPILib BatterySim parameters (12V, 20
      // mOhms)
      double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalSupplyCurrent);
      // Clamp to a safe physical minimum to avoid numerical issues/unrealistic drops (e.g. 4.5V)
      batteryVoltage = Math.max(4.5, batteryVoltage);

      // Update RoboRIO simulated battery voltage
      RoboRioSim.setVInVoltage(batteryVoltage);

      // Update motor controller simulation states
      for (var device : m_devices) {
        device.setSimSupplyVoltage(batteryVoltage);
      }
    }

    double totalPower = totalSupplyCurrent * RobotController.getBatteryVoltage();
    m_totalEnergy += totalPower * LoggedRobot.defaultPeriodSecs / 3600.0;

    Logger.recordOutput("PowerMonitor/TotalCurrent", totalSupplyCurrent);
    Logger.recordOutput("PowerMonitor/TotalPower (W)", totalPower);
    Logger.recordOutput("PowerMonitor/TotalEnergy (Wh)", m_totalEnergy);
  }
}
