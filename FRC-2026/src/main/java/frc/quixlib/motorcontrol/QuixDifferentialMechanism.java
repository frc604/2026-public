package frc.quixlib.motorcontrol;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;

public class QuixDifferentialMechanism {
  private final QuixTalonFX m_leader;
  private final QuixTalonFX m_follower;

  private final DifferentialMechanism<TalonFX> m_diffMech;
  private final MotionMagicVoltage m_avgRequest = new MotionMagicVoltage(0);
  private final PositionVoltage m_diffRequest = new PositionVoltage(0);

  public QuixDifferentialMechanism(final QuixTalonFX leader, final QuixTalonFX follower) {
    m_leader = leader;
    m_follower = follower;

    final DifferentialMotorConstants<TalonFXConfiguration> differentialConstants =
        new DifferentialMotorConstants<TalonFXConfiguration>()
            .withCANBusName(m_leader.getCANDeviceID().CANBus.getName())
            .withLeaderId(m_leader.getCANDeviceID().deviceNumber)
            .withFollowerId(m_follower.getCANDeviceID().deviceNumber)
            // .withAlignment(MotorAlignmentValue.Aligned)
            .withClosedLoopRate(200.0)
            .withLeaderInitialConfigs(leader.getConfiguration())
            .withFollowerInitialConfigs(follower.getConfiguration())
            .withFollowerUsesCommonLeaderConfigs(true);

    m_diffMech = new DifferentialMechanism<TalonFX>(TalonFX::new, differentialConstants);
  }

  public void setMotionMagicPositionSetpoint(
      final int avgSlot, final double avgSetpoint, final int diffSlot, final double diffSetpoint) {
    m_avgRequest.Slot = avgSlot;
    m_avgRequest.Position = m_leader.toNativeSensorPosition(avgSetpoint);
    m_diffRequest.Slot = diffSlot;
    m_diffRequest.Position = m_leader.toNativeSensorPosition(diffSetpoint);
    m_diffMech.setControl(m_avgRequest, m_diffRequest);
  }
}
