package frc.robot.simulation;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Iterator;
import org.json.JSONArray;
import org.json.JSONObject;

public class SimScenarioManager {
  private static SimScenarioManager m_instance = null;

  private boolean m_active = false;
  private JSONArray m_steps = null;
  private int m_currentStepIndex = 0;
  private double m_startTimeSec = -1.0;

  public static SimScenarioManager getInstance() {
    if (m_instance == null) {
      m_instance = new SimScenarioManager();
    }
    return m_instance;
  }

  private SimScenarioManager() {
    // Check for system property or environment variable designating the scenario file
    String filePath = System.getProperty("sim.scenario");
    if (filePath == null || filePath.isEmpty()) {
      filePath = System.getenv("SIM_SCENARIO");
    }

    if (filePath != null && !filePath.isEmpty()) {
      try {
        System.out.println("SimScenarioManager: Loading scenario file: " + filePath);
        File file = new File(filePath);
        String content = new String(Files.readAllBytes(Paths.get(file.getAbsolutePath())));
        JSONObject scenarioData = new JSONObject(content);

        // Configure Alliance Station
        String alliance = scenarioData.optString("alliance", "blue").toLowerCase();
        if (alliance.equals("red")) {
          DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        } else {
          DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        }

        // Configure Choosers
        JSONObject choosers = scenarioData.optJSONObject("choosers");
        if (choosers != null) {
          Iterator<String> keys = choosers.keys();
          while (keys.hasNext()) {
            String chooserName = keys.next();
            String selectedValue = choosers.getString(chooserName);
            System.out.println(
                "SimScenarioManager: Setting chooser '"
                    + chooserName
                    + "' to '"
                    + selectedValue
                    + "'");
            NetworkTableInstance.getDefault()
                .getEntry("/SmartDashboard/" + chooserName + "/selected")
                .setString(selectedValue);
          }
        }

        m_steps = scenarioData.getJSONArray("steps");
        m_active = true;
        m_currentStepIndex = 0;
        System.out.println(
            "SimScenarioManager: Successfully loaded " + m_steps.length() + " steps.");
      } catch (Exception e) {
        System.err.println("SimScenarioManager: Failed to load scenario file: " + e.getMessage());
        e.printStackTrace();
      }
    }
  }

  public boolean isActive() {
    return m_active;
  }

  public void update(double currentTimeSec) {
    if (!m_active || m_steps == null) {
      return;
    }

    if (m_startTimeSec < 0) {
      m_startTimeSec = currentTimeSec;
    }

    double elapsedSimTime = currentTimeSec - m_startTimeSec;

    // Find the current or next step that matches the elapsed simulation time
    while (m_currentStepIndex < m_steps.length() - 1) {
      JSONObject nextStep = m_steps.getJSONObject(m_currentStepIndex + 1);
      if (elapsedSimTime >= nextStep.getDouble("time_seconds")) {
        m_currentStepIndex++;
      } else {
        break;
      }
    }

    if (m_currentStepIndex >= m_steps.length() - 1
        && elapsedSimTime
            >= m_steps.getJSONObject(m_currentStepIndex).getDouble("time_seconds") + 0.1) {
      System.out.println("SimScenarioManager: Scenario complete. Exiting simulation.");
      System.exit(0);
    }

    JSONObject step = m_steps.getJSONObject(m_currentStepIndex);

    // Apply match status and control mode
    String mode = step.optString("mode", "teleop").toLowerCase();
    boolean enabled = step.optBoolean("enabled", true);

    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(mode.equals("autonomous") || mode.equals("auto"));
    DriverStationSim.setTest(mode.equals("test"));

    // Apply Joysticks
    JSONObject joysticks = step.optJSONObject("joysticks");
    if (joysticks != null) {
      Iterator<String> keys = joysticks.keys();
      while (keys.hasNext()) {
        String key = keys.next();
        int stickIndex = Integer.parseInt(key);
        JSONObject stickData = joysticks.getJSONObject(key);

        // Apply Axes
        JSONArray axes = stickData.optJSONArray("axes");
        if (axes != null) {
          DriverStationSim.setJoystickAxisCount(stickIndex, axes.length());
          for (int i = 0; i < axes.length(); i++) {
            DriverStationSim.setJoystickAxis(stickIndex, i, axes.getDouble(i));
          }
        }

        // Apply Buttons
        JSONArray buttons = stickData.optJSONArray("buttons");
        if (buttons != null) {
          DriverStationSim.setJoystickButtonCount(stickIndex, buttons.length());
          for (int i = 0; i < buttons.length(); i++) {
            DriverStationSim.setJoystickButton(stickIndex, i + 1, buttons.getBoolean(i));
          }
        }

        // Apply POVs
        JSONArray povs = stickData.optJSONArray("povs");
        if (povs != null) {
          DriverStationSim.setJoystickPOVCount(stickIndex, povs.length());
          for (int i = 0; i < povs.length(); i++) {
            DriverStationSim.setJoystickPOV(stickIndex, i, povs.getInt(i));
          }
        }
      }
    }

    // Notify DriverStation of updated packet data
    DriverStationSim.notifyNewData();
  }
}
