package frc.quixlib.devices;

public class QuixIOUtil {
  private static QuixIO[] m_devices = new QuixIO[0];

  public static void registerDevice(final QuixIO ioDevice) {
    ioDevice.updateInputs();
    QuixIO[] newDevices = new QuixIO[m_devices.length + 1];
    System.arraycopy(m_devices, 0, newDevices, 0, m_devices.length);
    newDevices[m_devices.length] = ioDevice;
    m_devices = newDevices;
  }

  public static void updateAll() {
    for (var device : m_devices) {
      device.updateInputs();
    }
  }
}
