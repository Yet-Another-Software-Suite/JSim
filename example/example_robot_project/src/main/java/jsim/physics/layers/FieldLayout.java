package jsim.physics.layers;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;
import jsim.physics.layers.fields.Field2026;

/**
 * Strategy interface for a season's static field geometry.
 *
 * <p>Implement one class per season (see {@link Field2026} for the 2026 REBUILT field) and pass
 * an instance to {@link Dyn4jCollisionLayer}. A field is just a list of {@link Element}s, each
 * built from polygon vertices in field-relative meters; {@link #populateWorld(World)} turns them
 * into dyn4j {@link Body} instances and drops them into the simulation world.
 *
 * <p>Use {@link #rectangle(double, double, double, double)} to build axis-aligned box elements
 * (the majority of field structures) without hand-listing four vertices, and {@link #of(Element...)}
 * for simple, one-off layouts that don't need their own class.
 */
public interface FieldLayout {

  /**
   * Populates the physics world with static collision boundaries and field structures.
   *
   * @param world The dyn4j simulation world instance.
   */
  void populateWorld(World<Body> world);

  /**
   * Builds an axis-aligned rectangular {@link Element}, wound counter-clockwise, from its center
   * and full width/depth. Covers the vast majority of field structures (hubs, bumps, trenches,
   * walls, ...) without having to hand-list four vertices every time.
   *
   * @param centerX Field-relative X coordinate of the rectangle's center, in meters.
   * @param centerY Field-relative Y coordinate of the rectangle's center, in meters.
   * @param width Full size of the rectangle along the X axis, in meters.
   * @param depth Full size of the rectangle along the Y axis, in meters.
   */
  static Element rectangle(double centerX, double centerY, double width, double depth) {
    double hw = width / 2.0;
    double hd = depth / 2.0;
    return new Element(
        new Translation2d(centerX - hw, centerY - hd),
        new Translation2d(centerX + hw, centerY - hd),
        new Translation2d(centerX + hw, centerY + hd),
        new Translation2d(centerX - hw, centerY + hd));
  }

  /**
   * A single field obstacle, defined by a convex polygon.
   *
   * <p>Vertices must be wound counter-clockwise and describe a convex polygon, per dyn4j's
   * {@link Geometry#createPolygon(Vector2...)} requirements. Non-convex field structures can be
   * approximated with multiple {@link Element}s.
   *
   * <p>Elements are immovable field structures (walls, posts, reef, etc.) by default. Passing a
   * {@code weight} instead creates a dynamic body, e.g. for a game piece the robot can push around.
   */
  class Element {
    private final Translation2d[] vertices;
    private final Mass weight;

    /** Creates an immovable field obstacle from the given polygon vertices. */
    public Element(Translation2d... vertices) {
      this(null, vertices);
    }

    /**
     * Creates a field element from the given polygon vertices.
     *
     * @param weight Mass of the element, or {@code null} to make it a static, immovable obstacle.
     * @param vertices Convex polygon vertices, wound counter-clockwise, in field-relative meters.
     */
    public Element(Mass weight, Translation2d... vertices) {
      if (vertices.length < 3) {
        throw new IllegalArgumentException(
            "A field element needs at least 3 vertices, got " + vertices.length);
      }
      this.vertices = vertices;
      this.weight = weight;
    }

    /**
     * Returns this element's polygon vertices, in field-relative meters, wound counter-clockwise.
     *
     * @return A copy of the vertex array.
     */
    public Translation2d[] getVertices() {
      return vertices.clone();
    }

    public Body toBody() {
      Vector2[] points = new Vector2[vertices.length];
      for (int i = 0; i < vertices.length; i++) {
        points[i] = new Vector2(vertices[i].getX(), vertices[i].getY());
      }

      Body body = new Body();
      BodyFixture fixture = body.addFixture(Geometry.createPolygon(points));
      fixture.setFriction(0.4);
      fixture.setRestitution(0.1);

      if (weight == null || weight.in(Kilograms) <= 0) {
        body.setMass(MassType.INFINITE);
      } else {
        body.setMass(MassType.NORMAL);
        body.setMass(new org.dyn4j.geometry.Mass(body.getWorldCenter(), weight.in(Kilograms), 0));
      }
      return body;
    }

    public List<Pose2d> getPoses()
    {
      var arrayList = new ArrayList<Pose2d>();
      for(var vert : vertices)
      {
        arrayList.add(new Pose2d(vert.getX(), vert.getY(), Rotation2d.kZero));
      }
      return arrayList;
    }
  }
}
