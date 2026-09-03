package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import jsim.physics.layers.utils.Cuboid3d;
import jsim.physics.layers.Dyn4jCollisionLayer;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;

/**
 * Strategy interface for a season's static field geometry.
 *
 * <p>Implement one class per season (see {@link Field2026} for the 2026 REBUILT field) and pass
 * an instance to {@link Dyn4jCollisionLayer}. A field is just a list of {@link Element}s, each
 * built from polygon vertices in field-relative meters; {@link #populateWorld(World)} turns them
 * into dyn4j {@link Body} instances and drops them into the simulation world.
 *
 * <p>Use {@link #rectangle(double, double, double, double)} to build axis-aligned box elements
 * (the majority of field structures) without hand-listing four vertices.
 *
 * <h3>Vertical extent</h3>
 *
 * <p>Every {@link Element} also carries a vertical extent -- a bottom height plus a top height
 * that may differ at the element's low-X and high-X edges, so a single element can describe
 * either an upright box ({@link #box}) or a sloped ramp such as a BUMP face ({@link #ramp}). The
 * 2D dyn4j drivetrain collision path ignores those heights entirely; they exist for 3D game-piece
 * layers (see {@code FuelLayer}), which have to know whether a ball clears a structure, lands on
 * top of it, or rolls up it.
 */
public interface FieldLayout {

  /**
   * Top height, in meters, assumed for elements built without an explicit vertical extent (i.e.
   * via {@link #rectangle}). Tall enough that a 3D game-piece layer treats such an element as a
   * solid wall rather than something a game piece hops over; seasons that care about pieces
   * clearing a structure should state its real height with {@link #box} or {@link #ramp}.
   */
  double DEFAULT_TOP_HEIGHT_METERS = 1.0;

  /**
   * Populates the physics world with static collision boundaries and field structures.
   *
   * @param world The dyn4j simulation world instance.
   */
  void populateWorld(World<Body> world);

  /**
   * Builds an axis-aligned rectangular {@link Element}, wound counter-clockwise, from its center
   * and full width/depth, spanning from the carpet up to {@link #DEFAULT_TOP_HEIGHT_METERS}.
   *
   * @param centerX Field-relative X coordinate of the rectangle's center, in meters.
   * @param centerY Field-relative Y coordinate of the rectangle's center, in meters.
   * @param width Full size of the rectangle along the X axis, in meters.
   * @param depth Full size of the rectangle along the Y axis, in meters.
   */
  static Element rectangle(double centerX, double centerY, double width, double depth) {
    return box(centerX, centerY, width, depth, 0.0, DEFAULT_TOP_HEIGHT_METERS);
  }

  /**
   * Builds an axis-aligned box {@link Element} with an explicit vertical extent -- the shape of
   * most field structures (HUB walls, TRENCH gates, guardrails, ...).
   *
   * @param centerX Field-relative X coordinate of the box's center, in meters.
   * @param centerY Field-relative Y coordinate of the box's center, in meters.
   * @param width Full size of the box along the X axis, in meters.
   * @param depth Full size of the box along the Y axis, in meters.
   * @param bottomHeight Height of the box's underside above the carpet, in meters.
   * @param topHeight Height of the box's top face above the carpet, in meters.
   */
  static Element box(
      double centerX,
      double centerY,
      double width,
      double depth,
      double bottomHeight,
      double topHeight) {
    return ramp(centerX, centerY, width, depth, bottomHeight, topHeight, topHeight);
  }

  /**
   * Builds an {@link Element} whose top face slopes linearly along the X axis, for structures a
   * game piece rolls up rather than bounces off -- e.g. one face of a BUMP.
   *
   * @param centerX Field-relative X coordinate of the footprint's center, in meters.
   * @param centerY Field-relative Y coordinate of the footprint's center, in meters.
   * @param width Full size of the footprint along the X axis, in meters.
   * @param depth Full size of the footprint along the Y axis, in meters.
   * @param bottomHeight Height of the element's underside above the carpet, in meters.
   * @param topHeightAtMinX Height of the sloped top face at the footprint's low-X edge, in meters.
   * @param topHeightAtMaxX Height of the sloped top face at the footprint's high-X edge, in meters.
   */
  static Element ramp(
      double centerX,
      double centerY,
      double width,
      double depth,
      double bottomHeight,
      double topHeightAtMinX,
      double topHeightAtMaxX) {
    double hw = width / 2.0;
    double hd = depth / 2.0;
    return new Element(
        null,
        bottomHeight,
        topHeightAtMinX,
        topHeightAtMaxX,
        new Translation2d(centerX - hw, centerY - hd),
        new Translation2d(centerX + hw, centerY - hd),
        new Translation2d(centerX + hw, centerY + hd),
        new Translation2d(centerX - hw, centerY + hd));
  }

  /**
   * A single field obstacle, defined by a convex polygon footprint plus a vertical extent.
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
    private final double bottomHeight;
    private final double topHeightAtMinX;
    private final double topHeightAtMaxX;
    private final double minX;
    private final double maxX;
    private final double minY;
    private final double   maxY;
    private final Cuboid3d cuboid;

    /** Creates an immovable field obstacle from the given polygon vertices. */
    public Element(Translation2d... vertices) {
      this(null, vertices);
    }

    /**
     * Creates a field element from the given polygon vertices, spanning from the carpet up to
     * {@link FieldLayout#DEFAULT_TOP_HEIGHT_METERS}.
     *
     * @param weight Mass of the element, or {@code null} to make it a static, immovable obstacle.
     * @param vertices Convex polygon vertices, wound counter-clockwise, in field-relative meters.
     */
    public Element(Mass weight, Translation2d... vertices) {
      this(weight, 0.0, DEFAULT_TOP_HEIGHT_METERS, DEFAULT_TOP_HEIGHT_METERS, vertices);
    }

    /**
     * Creates a field element from the given polygon vertices and vertical extent.
     *
     * @param weight Mass of the element, or {@code null} to make it a static, immovable obstacle.
     * @param bottomHeight Height of the element's underside above the carpet, in meters.
     * @param topHeightAtMinX Height of the top face at the footprint's low-X edge, in meters.
     * @param topHeightAtMaxX Height of the top face at the footprint's high-X edge, in meters.
     *     Equal to {@code topHeightAtMinX} for a flat-topped box.
     * @param vertices Convex polygon vertices, wound counter-clockwise, in field-relative meters.
     */
    public Element(
        Mass weight,
        double bottomHeight,
        double topHeightAtMinX,
        double topHeightAtMaxX,
        Translation2d... vertices) {
      if (vertices.length < 3) {
        throw new IllegalArgumentException(
            "A field element needs at least 3 vertices, got " + vertices.length);
      }
      this.vertices = vertices;
      this.weight = weight;
      this.bottomHeight = bottomHeight;
      this.topHeightAtMinX = topHeightAtMinX;
      this.topHeightAtMaxX = topHeightAtMaxX;

      double lowX = Double.POSITIVE_INFINITY;
      double highX = Double.NEGATIVE_INFINITY;
      double lowY = Double.POSITIVE_INFINITY;
      double highY = Double.NEGATIVE_INFINITY;
      for (Translation2d vertex : vertices) {
        lowX = Math.min(lowX, vertex.getX());
        highX = Math.max(highX, vertex.getX());
        lowY = Math.min(lowY, vertex.getY());
        highY = Math.max(highY, vertex.getY());
      }
      this.minX = lowX;
      this.maxX = highX;
      this.minY = lowY;
      this.maxY = highY;

      double top = Math.max(topHeightAtMinX, topHeightAtMaxX);
      this.cuboid = new Cuboid3d(
          new Pose3d(
              new Translation3d(
                  (lowX + highX) / 2.0, (lowY + highY) / 2.0, (bottomHeight + top) / 2.0),
              Rotation3d.kZero),
          highX - lowX,
          highY - lowY,
          top - bottomHeight);
    }

    /**
     * Returns this element's polygon vertices, in field-relative meters, wound counter-clockwise.
     *
     * @return A copy of the vertex array.
     */
    public Translation2d[] getVertices() {
      return vertices.clone();
    }

    /** Height of this element's underside above the carpet, in meters. */
    public double getBottomHeight() {
      return bottomHeight;
    }

    /** Height of this element's top face at its low-X edge, in meters. */
    public double getTopHeightAtMinX() {
      return topHeightAtMinX;
    }

    /** Height of this element's top face at its high-X edge, in meters. */
    public double getTopHeightAtMaxX() {
      return topHeightAtMaxX;
    }

    /** Highest point of this element's top face above the carpet, in meters. */
    public double getTopHeight() {
      return Math.max(topHeightAtMinX, topHeightAtMaxX);
    }

    /**
     * Whether this element's top face slopes along X (a ramp, e.g. a BUMP face) rather than being
     * flat (a box). Game-piece layers roll pieces along a sloped face instead of bouncing them
     * off a vertical side.
     */
    public boolean isSloped() {
      return topHeightAtMinX != topHeightAtMaxX;
    }

    /**
     * Height of this element's top face directly above {@code x}, in meters, linearly
     * interpolated between {@link #getTopHeightAtMinX()} and {@link #getTopHeightAtMaxX()}.
     * {@code x} values outside the footprint are clamped to its edges.
     */
    public double topHeightAt(double x) {
      if (maxX <= minX) {
        return topHeightAtMinX;
      }
      double t = Math.min(1.0, Math.max(0.0, (x - minX) / (maxX - minX)));
      return topHeightAtMinX + t * (topHeightAtMaxX - topHeightAtMinX);
    }

    /** Lowest X coordinate of this element's footprint, in meters. */
    public double getMinX() {
      return minX;
    }

    /** Highest X coordinate of this element's footprint, in meters. */
    public double getMaxX() {
      return maxX;
    }

    /** Lowest Y coordinate of this element's footprint, in meters. */
    public double getMinY() {
      return minY;
    }

    /** Highest Y coordinate of this element's footprint, in meters. */
    public double getMaxY() {
      return maxY;
    }

    /** Center of this element's footprint bounding box, in field-relative meters. */
    public Translation2d getCenter() {
      return new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
    }

    /**
     * This element's footprint and vertical extent as an axis-aligned {@link Cuboid3d}, for sphere
     * collision (see {@link Cuboid3d#overlapWithSphere}). For a flat-topped element this is exact;
     * for a sloped one ({@link #isSloped()}) it is only the element's bounding box -- collide
     * against the sloped face itself instead, using {@link #topHeightAt(double)}.
     */
    public Cuboid3d getCuboid() {
      return cuboid;
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
