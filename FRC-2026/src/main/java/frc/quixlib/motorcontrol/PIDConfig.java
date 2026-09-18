package frc.quixlib.motorcontrol;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

/**
 * PIDConfig is a configuration class for PID (Proportional, Integral, Derivative) control
 * parameters. It also includes additional parameters for feedforward control.
 *
 * <p>The class provides constructors to initialize the PID and feedforward parameters, and methods
 * to fill CTRE (Cross The Road Electronics) configuration objects with these parameters.
 *
 * <p>Parameters: - kP: Proportional gain - kI: Integral gain - kD: Derivative gain - kS: Static
 * gain (feedforward) - kV: Velocity gain (feedforward) - kA: Acceleration gain (feedforward) - kG:
 * Gravity gain (feedforward)
 *
 * <p>Constructors: - PIDConfig(): Initializes all parameters to 0.0. - PIDConfig(double kP, double
 * kI, double kD): Initializes PID parameters, sets feedforward parameters to 0.0. -
 * PIDConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG):
 * Initializes all parameters.
 *
 * <p>Methods: - Slot0Configs fillCTRE(Slot0Configs conf): Fills a Slot0Configs object with the PID
 * and feedforward parameters. - Slot1Configs fillCTRE(Slot1Configs conf): Fills a Slot1Configs
 * object with the PID and feedforward parameters. - Slot2Configs fillCTRE(Slot2Configs conf): Fills
 * a Slot2Configs object with the PID and feedforward parameters.
 */
public class PIDConfig {
  // This matches CTRE configs
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kS;
  public final double kV;
  public final double kA;
  public final double kG;

  public PIDConfig() {
    this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public PIDConfig(final double kP, final double kI, final double kD) {
    this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
  }

  public PIDConfig(
      final double kP,
      final double kI,
      final double kD,
      final double kS,
      final double kV,
      final double kA,
      final double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
  }

  public Slot0Configs fillCTRE(Slot0Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }

  public Slot1Configs fillCTRE(Slot1Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }

  public Slot2Configs fillCTRE(Slot2Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }
}
