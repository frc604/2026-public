// Highly inspired by
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/HubShiftUtil.java

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShiftManager {
  public enum ShiftEnum {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
  }

  public record ShiftInfo(
      ShiftEnum currentShift,
      double elapsedTime,
      double remainingTime,
      boolean isFirstActive,
      boolean isCurrentlyActive) {}

  private static double kStartingActiveAdjustment = 0.0; // s
  private static double kEndingActiveAdjustment = 0.0; // s
  private static Timer m_matchTimer = new Timer();

  // Call at the beginning of auto and teleop
  public static void initialize() {
    m_matchTimer.restart();
  }

  public static Optional<Alliance> getFirstActiveAlliance() {
    final String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Optional.of(Alliance.Blue);
      } else if (character == 'B') {
        return Optional.of(Alliance.Red);
      }
    }
    return Optional.empty();
  }

  public static ShiftEnum getCurrentShift() {
    if (DriverStation.isDisabled()) {
      return ShiftEnum.DISABLED;
    }
    if (DriverStation.isAutonomous()) {
      return ShiftEnum.AUTO;
    }
    double t = m_matchTimer.get();
    if (t < getShiftStart(ShiftEnum.SHIFT1)) return ShiftEnum.TRANSITION;
    if (t < getShiftStart(ShiftEnum.SHIFT2)) return ShiftEnum.SHIFT1;
    if (t < getShiftStart(ShiftEnum.SHIFT3)) return ShiftEnum.SHIFT2;
    if (t < getShiftStart(ShiftEnum.SHIFT4)) return ShiftEnum.SHIFT3;
    if (t < getShiftStart(ShiftEnum.ENDGAME)) return ShiftEnum.SHIFT4;
    return ShiftEnum.ENDGAME;
  }

  private static boolean isFirstActive() {
    Optional<Alliance> firstActiveAlliance = getFirstActiveAlliance();
    Optional<Alliance> ourAlliance = DriverStation.getAlliance();

    boolean isFirstActive = false;
    if (firstActiveAlliance.isPresent() && ourAlliance.isPresent()) {
      isFirstActive = firstActiveAlliance.get() == ourAlliance.get();
    }
    return isFirstActive;
  }

  private static boolean isActive(ShiftEnum shift) {
    if (shift == ShiftEnum.DISABLED) {
      return false;
    }
    if (shift == ShiftEnum.AUTO || shift == ShiftEnum.TRANSITION || shift == ShiftEnum.ENDGAME) {
      return true;
    }

    if (isFirstActive()) {
      return shift == ShiftEnum.SHIFT1 || shift == ShiftEnum.SHIFT3;
    } else {
      return shift == ShiftEnum.SHIFT2 || shift == ShiftEnum.SHIFT4;
    }
  }

  public static double getShiftStart(ShiftEnum shift) {
    switch (shift) {
      case TRANSITION:
        return 0.0;
      case SHIFT1:
        return 10.0;
      case SHIFT2:
        return 35.0
            + (isActive(ShiftEnum.SHIFT2)
                ? kStartingActiveAdjustment
                : (isActive(ShiftEnum.SHIFT1) ? kEndingActiveAdjustment : 0.0));
      case SHIFT3:
        return 60.0
            + (isActive(ShiftEnum.SHIFT3)
                ? kStartingActiveAdjustment
                : (isActive(ShiftEnum.SHIFT2) ? kEndingActiveAdjustment : 0.0));
      case SHIFT4:
        return 85.0
            + (isActive(ShiftEnum.SHIFT4)
                ? kStartingActiveAdjustment
                : (isActive(ShiftEnum.SHIFT3) ? kEndingActiveAdjustment : 0.0));
      case ENDGAME:
        return 110.0
            + (isActive(ShiftEnum.ENDGAME)
                ? kStartingActiveAdjustment
                : (isActive(ShiftEnum.SHIFT4) ? kEndingActiveAdjustment : 0.0));
      default:
        return 0.0;
    }
  }

  public static double getShiftEnd(ShiftEnum shift) {
    switch (shift) {
      case TRANSITION:
        return getShiftStart(ShiftEnum.SHIFT1);
      case SHIFT1:
        return getShiftStart(ShiftEnum.SHIFT2);
      case SHIFT2:
        return getShiftStart(ShiftEnum.SHIFT3);
      case SHIFT3:
        return getShiftStart(ShiftEnum.SHIFT4);
      case SHIFT4:
        return getShiftStart(ShiftEnum.ENDGAME);
      case ENDGAME:
        return 140.0;
      default:
        return 0.0;
    }
  }

  public static ShiftInfo getShiftInfo() {
    ShiftEnum currentShift = getCurrentShift();
    double elapsedTime = 0.0;
    double remainingTime = 0.0;
    boolean active = isActive(currentShift);

    final double t = m_matchTimer.get();
    if (currentShift == ShiftEnum.AUTO) {
      elapsedTime = t;
      remainingTime = 20.0 - t;
    } else if (currentShift != ShiftEnum.DISABLED) {
      ShiftEnum[] shifts = ShiftEnum.values();
      int ordinal = currentShift.ordinal();

      int startOrdinal = ordinal;
      while (startOrdinal > ShiftEnum.TRANSITION.ordinal()
          && isActive(shifts[startOrdinal - 1]) == active) {
        startOrdinal--;
      }

      int endOrdinal = ordinal;
      while (endOrdinal < ShiftEnum.ENDGAME.ordinal()
          && isActive(shifts[endOrdinal + 1]) == active) {
        endOrdinal++;
      }

      double start = getShiftStart(shifts[startOrdinal]);
      double end = getShiftEnd(shifts[endOrdinal]);
      elapsedTime = t - start;
      remainingTime = end - t;
    }

    return new ShiftInfo(
        currentShift,
        Math.max(0.0, elapsedTime),
        Math.max(0.0, remainingTime),
        isFirstActive(),
        active);
  }

  public static void logShiftInfo() {
    ShiftInfo info = getShiftInfo();
    Logger.recordOutput("ShiftInfo/Current Shift", info.currentShift());
    Logger.recordOutput("ShiftInfo/Elapsed Time (s)", info.elapsedTime());
    Logger.recordOutput("ShiftInfo/Remaining Time (s)", info.remainingTime());
    Logger.recordOutput("ShiftInfo/First Active", info.isFirstActive());
    Logger.recordOutput("ShiftInfo/Currently Active", info.isCurrentlyActive());
  }
}
