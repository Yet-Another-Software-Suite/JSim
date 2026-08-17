package frc.robot.jsim.physics.layers;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

/**
 * Strategy interface for adding static field geometry and obstacles into a dyn4j physics world.
 */
@FunctionalInterface
public interface FieldLayout {

  /**
   * Populates the physics world with static collision boundaries and field structures.
   *
   * @param world The dyn4j simulation world instance.
   */
  void populateWorld(World<Body> world);
}