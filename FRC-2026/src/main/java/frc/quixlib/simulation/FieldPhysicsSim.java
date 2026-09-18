package frc.quixlib.simulation;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

public class FieldPhysicsSim {
  private final World<Body> m_field = new World<>();
  private final FieldObstacles m_fieldObstacles = new FieldObstacles();
  private final SwerveDrivePhysicsSim m_robot = new SwerveDrivePhysicsSim();

  private final Timer m_timer = new Timer();

  public FieldPhysicsSim() {
    if (Robot.isReal()) {
      return;
    }

    m_field.setGravity(PhysicsWorld.ZERO_GRAVITY);

    // Add field obstacles
    m_fieldObstacles.addToField(m_field);

    // Add robot
    m_field.addBody(m_robot);
  }

  /** Must be called every simulation periodic loop. */
  public void update() {
    m_timer.start();
    final double dt = m_timer.get();
    m_timer.reset();
    m_field.step(1, dt);
  }

  public SwerveDrivePhysicsSim getRobot() {
    return m_robot;
  }
}
