package jsim.physics.layers;

import static edu.wpi.first.units.Units.Kilogram;

import java.util.List;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;

/**
 * Strategy interface for adding static field geometry and obstacles into a dyn4j physics world.
 */
public class FieldLayout {

  class Element {
    Translation2d[] verticies;
    Mass weight;

    public Element(Mass weight, Translation2d... verticies)
    {
      this.verticies = verticies;
      this.weight = weight;
    }
  }
  Element[] elements;

  public FieldLayout(Element... elements)
  {
    this.elements = elements;
  }

  static FieldLayout LOAD_2026_FIELD = new FieldLayout(new Element(Kilogram.of(1), new Translation2d(0,0), new Translation2d(), new Translation2d()));

/**
   * Populates the physics world with static collision boundaries and field structures.
   *
   * @param world The dyn4j simulation world instance.
   */
  void populateWorld(World<Body> world) {};
}