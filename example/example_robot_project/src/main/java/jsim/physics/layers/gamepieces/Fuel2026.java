package jsim.physics.layers.gamepieces;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import jsim.physics.layers.fields.FieldLayout.Element;
import jsim.physics.layers.utils.Contact;
import jsim.physics.layers.utils.Cuboid3d;
import jsim.physics.layers.utils.Sphere3d;

/**
 * A single FUEL game piece: a 6in sphere tracked as a point mass in three dimensions.
 *
 * <p>FUEL is simulated as a free particle rather than a rigid body -- it has a position and a
 * velocity but no orientation, since a smooth sphere's spin doesn't affect anything else in the
 * model. {@code jsim.physics.layers.FuelLayer} owns the integration and most collision response
 * (walls, the carpet, the HUB, the robot); this class holds the ball's own state plus the pieces
 * of collision math -- its {@link #sphere() sphere} and ball-to-ball {@link #collide}
 * response -- that depend only on physical properties of the ball itself.
 */
public class Fuel2026 extends Gamepiece {

  /** Diameter of one FUEL ball -- a fixed physical property of the game piece. */
  public static final Distance FUEL_DIAMETER = Meters.of(0.15);

  /** Radius of one FUEL ball, in meters. */
  public static final double FUEL_RADIUS = FUEL_DIAMETER.in(Meters) / 2.0;

  /**
   * Coefficient of restitution between FUEL and a field structure -- a property of the ball's own
   * material (how much energy a bounce keeps), not of any one structure it hits.
   */
  public static final double FIELD_COR = Math.sqrt(22.0 / 51.5);

  /** Coefficient of restitution between two FUEL balls. */
  public static final double FUEL_COR = 0.5;

  /** Coefficient of restitution between FUEL and a HUB net, which absorbs most of the impact. */
  public static final double NET_COR = 0.2;

  /** Coefficient of restitution between FUEL and a robot bumper. */
  public static final double ROBOT_COR = 0.1;

  /** Mass of one FUEL ball -- a fixed physical property of the game piece. */
  public static final Mass FUEL_MASS = Pounds.of(0.448);

  /** Mass of one FUEL ball, in kilograms. */
  private static final double FUEL_MASS_KG = FUEL_MASS.in(Kilograms);

  /** Density of dry air at room temperature, in kg/m^3. */
  private static final double AIR_DENSITY = 1.2041;

  /**
   * Drag coefficient of a smooth sphere, dimensionless -- a property of the ball's own shape, not
   * of the air it flies through.
   */
  private static final double DRAG_COEFFICIENT = 0.47;

  /** Constant part of the drag force this ball feels in flight, i.e. {@code 0.5 * rho * Cd * A}. */
  private static final double DRAG_FORCE_FACTOR =
      0.5 * AIR_DENSITY * DRAG_COEFFICIENT * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  /**
   * Proportion of horizontal velocity a resting ball loses to rolling friction per second -- a
   * property of the ball's own surface, not of the carpet or structure it is rolling on.
   */
  public static final double ROLLING_FRICTION = 0.1;

  /**
   * Creates a FUEL piece at rest.
   *
   * @param position Field-relative position of the ball's center, in meters.
   */
  public Fuel2026(Translation3d position) {
    this(position, new Translation3d());
  }

  /**
   * Creates a moving FUEL piece.
   *
   * @param position Field-relative position of the ball's center, in meters.
   * @param velocity Field-relative velocity, in meters per second.
   */
  public Fuel2026(Translation3d position, Translation3d velocity) {
    super(position, velocity, FUEL_RADIUS);
  }

  /**
   * Moves this ball by one substep and, unless it is resting on something, accelerates it under
   * gravity and (optionally) aerodynamic drag.
   *
   * @param dtSeconds Substep duration, in seconds.
   * @param gravity Downwards acceleration due to gravity, in m/s^2.
   * @param simulateAirResistance Whether this ball should feel aerodynamic drag while airborne.
   */
  public void integrate(double dtSeconds, double gravity, boolean simulateAirResistance) {
    translate(velocity.times(dtSeconds));

    if (!supported) {
      Translation3d force = new Translation3d(0, 0, gravity * FUEL_MASS_KG);
      if (simulateAirResistance) {
        double speed = velocity.getNorm();
        if (speed > 1e-6) {
          force = force.plus(velocity.times(-DRAG_FORCE_FACTOR * speed));
        }
      }
      addImpulse(force.div(FUEL_MASS_KG).times(dtSeconds));
    }

    // Cleared here so this substep's collisions decide afresh whether this ball is still supported.
    setSupported(false);
  }

  /**
   * Drops this ball onto the carpet if it is touching or nearly touching it, bouncing whatever
   * downwards speed it had left off {@link #FIELD_COR}. A ball hovering within {@code contactSkin}
   * of the carpet with almost no vertical speed left settles onto it instead of bouncing.
   *
   * @param contactSkin Extra reach, in meters, beyond this ball's radius that still counts as
   *     touching a nearly-stopped ball settling onto the carpet.
   * @param restSpeed Vertical speed below which this ball is considered to have stopped.
   */
  public void collideCarpet(double contactSkin, double restSpeed) {
    boolean overlapping = position.getZ() < FUEL_RADIUS;
    if (!overlapping
        && (position.getZ() > FUEL_RADIUS + contactSkin || Math.abs(velocity.getZ()) >= restSpeed)) {
      return;
    }

    setPosition(new Translation3d(position.getX(), position.getY(), FUEL_RADIUS));
    if (overlapping && velocity.getZ() < 0) {
      setVelocity(new Translation3d(velocity.getX(), velocity.getY(), -velocity.getZ() * FIELD_COR));
    }
    setSupported(true);
  }

  /**
   * Settles this ball if it is touching a surface flat enough to hold it and has nearly stopped
   * moving vertically: its remaining vertical speed is dropped so it stops jittering, and its
   * horizontal speed bleeds off to {@link #ROLLING_FRICTION}. A ball still moving vertically is
   * left unsupported so gravity keeps acting on it next substep.
   *
   * @param dtSeconds Substep duration, in seconds.
   * @param restSpeed Vertical speed below which this ball is considered to have stopped.
   */
  public void settle(double dtSeconds, double restSpeed) {
    if (!supported) {
      return;
    }
    if (Math.abs(velocity.getZ()) >= restSpeed) {
      setSupported(false);
      return;
    }

    setVelocity(new Translation3d(velocity.getX(), velocity.getY(), 0.0)
        .times(1.0 - ROLLING_FRICTION * dtSeconds));
  }

  /**
   * Returns this ball's current position and size as a sphere, for collision math.
   *
   * @return A {@link Sphere3d} centered on this ball with radius {@link #FUEL_RADIUS}.
   */
  public Sphere3d sphere() {
    return super.sphere();
  }

  /**
   * Resolves a collision between this ball and {@code other}, separating them if they overlap and
   * exchanging an equal and opposite impulse along the contact normal.
   *
   * @param other The other ball to collide against.
   * @param cor Coefficient of restitution between two FUEL balls.
   */
  public void collide(Fuel2026 other, double cor) {
    super.collide(other, cor);
  }

  /**
   * Reflects this ball's velocity off a surface with the given outward {@code normal}, and marks
   * it supported if that surface is flat enough to rest on. A ball already moving away from the
   * surface keeps its velocity -- it has been pushed clear and shouldn't be pulled back.
   *
   * @param normal Outward surface normal to bounce off of.
   * @param cor Coefficient of restitution for this bounce.
   * @param supportNormalZ Minimum Z component of {@code normal} that still counts as flat enough
   *     to rest on.
   */
  public void bounce(Translation3d normal, double cor, double supportNormalZ) {
    double approachSpeed = velocity.dot(normal);
    if (approachSpeed < 0) {
      addImpulse(normal.times(-(1.0 + cor) * approachSpeed));
    }
    if (normal.getZ() > supportNormalZ) {
      setSupported(true);
    }
  }

  /**
   * Keeps this ball inside {@code bounds} at every height, bouncing it off {@link #FIELD_COR}
   * along whichever axis it crossed.
   *
   * <p>{@link Rectangle2d#nearest} does the X/Y geometry -- the caller is expected to have already
   * inset {@code bounds} by this ball's own radius, so whichever coordinate it clamps is exactly
   * the one that crossed the boundary. {@link #bounce} then decides, per axis, whether this ball
   * was actually heading further out (and so needs a wall bounce) or just grazed the boundary
   * while already heading back in.
   *
   * @param bounds The playable footprint to keep this ball inside, already inset by its radius.
   * @param supportNormalZ Minimum Z component of a contact normal that still counts as flat enough
   *     to rest on.
   */
  public void clampToBounds(Rectangle2d bounds, double supportNormalZ) {
    Translation2d point = getTranslation2d();
    Translation2d nearest = bounds.nearest(point);
    if (nearest.equals(point)) {
      return;
    }

    translate(new Translation3d(nearest.minus(point)));
    if (nearest.getX() != point.getX()) {
      bounce(new Translation3d(Math.signum(nearest.getX() - point.getX()), 0, 0), FIELD_COR, supportNormalZ);
    }
    if (nearest.getY() != point.getY()) {
      bounce(new Translation3d(0, Math.signum(nearest.getY() - point.getY()), 0), FIELD_COR, supportNormalZ);
    }
  }

  /**
   * Whether a contact against this ball that isn't quite touching should still be treated as
   * resting: the surface has to be flat enough to hold the ball up, and the ball has to have
   * nearly stopped moving vertically.
   *
   * @param normal Outward contact normal to check.
   * @param supportNormalZ Minimum Z component of {@code normal} that counts as flat enough to rest on.
   * @param restSpeed Vertical speed below which this ball is considered to have stopped.
   * @return Whether this contact should be treated as resting rather than in-flight.
   */
  public boolean isRestingContact(Translation3d normal, double supportNormalZ, double restSpeed) {
    return normal.getZ() > supportNormalZ && Math.abs(velocity.getZ()) < restSpeed;
  }

  /**
   * Resolves this ball's overlap with an upright box, using {@link Cuboid3d#overlapWithSphere} to
   * push it out along the direction from the box's nearest point to this ball's center and
   * {@link #bounce} to reflect its velocity -- the same routine that handles being shoved sideways
   * off a HUB wall and coming to rest on top of a TRENCH.
   *
   * @param cuboid The box to resolve against.
   * @param cor Coefficient of restitution to bounce off it with.
   * @param margin Extra reach, in meters, beyond this ball's radius that still counts as touching.
   * @param supportNormalZ Minimum Z component of a contact normal that still counts as flat enough
   *     to rest on.
   * @param restSpeed Vertical speed below which this ball is considered to have stopped.
   * @return The resolved contact, or {@code null} if this ball doesn't overlap the box.
   */
  public Contact collideBox(
      Cuboid3d cuboid, double cor, double margin, double supportNormalZ, double restSpeed) {
    Contact contact = cuboid.overlapWithSphere(sphere(), margin);
    if (contact == null) {
      return null;
    }
    if (contact.depth() < 0 && !isRestingContact(contact.normal(), supportNormalZ, restSpeed)) {
      return null; // Inside the margin but not settling onto a top face: not touching yet.
    }

    translate(contact.pushOut());
    bounce(contact.normal(), cor, supportNormalZ);
    return contact;
  }

  /**
   * Resolves this ball rolling on a field structure whose top face slopes along X, such as a BUMP
   * face. The sloped face is treated as a line in the XZ plane spanning the element's footprint;
   * this ball, if within the element's Y band and closer to that line than its own radius, gets
   * pushed out perpendicular to the slope, so it rolls up and over rather than stopping against a
   * wall.
   *
   * @param element The sloped field structure to collide against.
   * @param contactSkin Extra reach, in meters, beyond this ball's radius that still counts as
   *     touching a nearly-stopped ball settling onto the slope.
   * @param supportNormalZ Minimum Z component of a contact normal that still counts as flat enough
   *     to rest on.
   * @param restSpeed Vertical speed below which this ball is considered to have stopped.
   */
  public void collideSlopedTop(
      Element element, double contactSkin, double supportNormalZ, double restSpeed) {
    if (position.getY() < element.getMinY() || position.getY() > element.getMaxY()) {
      return;
    }

    // Collapse to the XZ plane, where the sloped face is a single line segment.
    Translation2d start = new Translation2d(element.getMinX(), element.getTopHeightAtMinX());
    Translation2d end = new Translation2d(element.getMaxX(), element.getTopHeightAtMaxX());
    Translation2d ballXz = new Translation2d(position.getX(), position.getZ());
    Translation2d segment = end.minus(start);

    double segmentLength = segment.getNorm();
    if (segmentLength < 1e-9) {
      return;
    }

    Translation2d closest = start.plus(
        segment.times(ballXz.minus(start).dot(segment) / (segmentLength * segmentLength)));
    if (closest.getDistance(start) + closest.getDistance(end) > segmentLength) {
      return; // Nearest point on the infinite line falls outside the face itself.
    }

    double distance = ballXz.getDistance(closest);
    if (distance > FUEL_RADIUS + contactSkin) {
      return;
    }

    Translation3d normal = new Translation3d(
        -segment.getY() / segmentLength, 0, segment.getX() / segmentLength);
    if (distance > FUEL_RADIUS && !isRestingContact(normal, supportNormalZ, restSpeed)) {
      return; // Inside the contact skin but still moving: not touching the face yet.
    }

    translate(normal.times(FUEL_RADIUS - distance));
    bounce(normal, FIELD_COR, supportNormalZ);
  }

  /**
   * Column of the broadphase collision grid this ball's center falls into, for a grid of square
   * cells {@code cellSize} meters wide with its origin at the FIELD origin.
   *
   * @param cellSize Edge length of one grid cell, in meters.
   * @return Column index of the grid cell this ball's center falls into.
   */
  public int columnOf(double cellSize) {
    return (int) Math.floor(position.getX() / cellSize);
  }

  /**
   * Row of the broadphase collision grid this ball's center falls into. See {@link #columnOf}.
   *
   * @param cellSize Edge length of one grid cell, in meters.
   * @return Row index of the grid cell this ball's center falls into.
   */
  public int rowOf(double cellSize) {
    return (int) Math.floor(position.getY() / cellSize);
  }

  /**
   * Flat index of the broadphase collision grid cell this ball's center falls into, or -1 if it
   * falls outside the {@code gridColumns} x {@code gridRows} grid entirely (a ball somehow off the
   * FIELD). See {@link #columnOf}.
   *
   * @param cellSize Edge length of one grid cell, in meters.
   * @param gridColumns Number of columns in the grid.
   * @param gridRows Number of rows in the grid.
   * @return Flat index of this ball's grid cell, or -1 if it falls outside the grid.
   */
  public int cellIndexOf(double cellSize, int gridColumns, int gridRows) {
    int column = columnOf(cellSize);
    int row = rowOf(cellSize);
    if (column < 0 || column >= gridColumns || row < 0 || row >= gridRows) {
      return -1;
    }
    return column * gridRows + row;
  }
}
