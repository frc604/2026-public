package frc.quixlib.wpilib;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.quixlib.devices.QuixIO;
import frc.quixlib.devices.QuixIOUtil;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LoggedDigitalInput implements QuixIO {
  private final DigitalInput m_digitalInput;
  private final InputsAutoLogged m_inputs = new InputsAutoLogged();
  private final String m_loggingName;
  private boolean m_simValue = false;

  @AutoLog
  public static class Inputs {
    protected boolean value = false;
  }

  public LoggedDigitalInput(int channel) {
    m_digitalInput = new DigitalInput(channel);
    m_loggingName = "Inputs/DigitalInput [" + channel + "]";

    QuixIOUtil.registerDevice(this);
  }

  public void updateInputs() {
    m_inputs.value = m_digitalInput.get();
    Logger.processInputs(m_loggingName, m_inputs);
  }

  public boolean get() {
    return Robot.isSimulation() ? m_simValue : m_inputs.value;
  }

  public void setSimValue(boolean value) {
    m_simValue = value;
  }
}
