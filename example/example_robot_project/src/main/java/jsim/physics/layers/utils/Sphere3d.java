package jsim.physics.layers.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

/**
 * A 3D ball: a center point plus a radius -- the 3D counterpart of
 * {@link edu.wpi.first.math.geometry.Ellipse2d Ellipse2d}'s circle constructor, and the shape
 * {@code FuelLayer} uses to model a FUEL ball for collision.
 *
 * <p>Unlike {@link Cuboid3d}, a sphere looks the same from every direction, so it carries no
 * orientation -- just a {@link Translation3d} center and a radius.
 *
 * @see Cuboid3d
 */
public class Sphere3d {

  private final Translation3d center;
  private final double radius;

  /**
   * Constructs a sphere at the specified position with the specified radius.
   *
   * @param center The center of the sphere.
   * @param radius The radius of the sphere.
   */
  public Sphere3d(Translation3d center, double radius) {
    if (radius < 0) {
      throw new IllegalArgumentException("Sphere3d radius cannot be less than 0!");
    }
    this.center = center;
    this.radius = radius;
  }

  /**
   * Constructs a sphere at the specified position with the specified radius, in a measure.
   *
   * @param center The center of the sphere.
   * @param radius The radius of the sphere.
   */
  public Sphere3d(Translation3d center, Distance radius) {
    this(center, radius.in(Meters));
  }

  /** The center of the sphere. */
  public Translation3d getCenter() {
    return center;
  }

  /** The radius of the sphere. */
  public double getRadius() {
    return radius;
  }

  /** The radius of the sphere, in a measure. */
  public Distance getMeasureRadius() {
    return Meters.of(radius);
  }

  /**
   * Returns a sphere with the same radius, re-centered at {@code newCenter}. FUEL balls move every
   * physics step but never change size, so callers re-derive a fresh {@link Sphere3d} from a
   * ball's latest position through this rather than constructing one from scratch.
   *
   * @param newCenter The center of the returned sphere.
   * @return A sphere of the same radius, centered at {@code newCenter}.
   */
  public Sphere3d withCenter(Translation3d newCenter) {
    return new Sphere3d(newCenter, radius);
  }

  /**
   * Checks if a point is contained within the sphere. This is inclusive: a point lying exactly on
   * the surface returns {@code true}.
   *
   * @param point The point to check.
   * @return Whether the sphere contains the point.
   */
  public boolean contains(Translation3d point) {
    return center.getDistance(point) <= radius;
  }

  /**
   * Returns the nearest point contained within the sphere.
   *
   * @param point The point to find the nearest point to.
   * @return A new point nearest to {@code point} and contained in the sphere.
   */
  public Translation3d nearest(Translation3d point) {
    double distance = center.getDistance(point);
    if (distance <= radius) {
      return point;
    }
    return center.plus(point.minus(center).times(radius / distance));
  }

  /**
   * Returns the distance between the surface of the sphere and the point.
   *
   * @param point The point to check.
   * @return The distance (0 if the point is contained by the sphere).
   */
  public double getDistance(Translation3d point) {
    return Math.max(0.0, center.getDistance(point) - radius);
  }

  /**
   * Returns the distance between the surface of the sphere and the point, in a measure.
   *
   * @param point The point to check.
   * @return The distance (0 if the point is contained by the sphere), in a measure.
   */
  public Distance getMeasureDistance(Translation3d point) {
    return Meters.of(getDistance(point));
  }

  /**
   * Resolves another sphere against this one, for rigid-body-style collision response (e.g. two
   * FUEL balls colliding).
   *
   * @param other The other sphere.
   * @return The contact normal (pointing from this sphere's center towards {@code other}'s) and
   *     penetration depth, or {@code null} if the two don't overlap.
   * @see #overlapWithSphere(Sphere3d, double)
   */
  public Contact overlapWithSphere(Sphere3d other) {
    return overlapWithSphere(other, 0.0);
  }

  /**
   * Resolves another sphere against this one, as {@link #overlapWithSphere(Sphere3d)}, but
   * widening detection by {@code margin} beyond the two spheres' combined radius.
   *
   * @param other The other sphere.
   * @param margin Extra reach, in meters, beyond the two radii to still report a contact for.
   * @return The contact normal and penetration depth (possibly negative, within {@code margin}),
   *     or {@code null} if nothing is within reach.
   */
  public Contact overlapWithSphere(Sphere3d other, double margin) {
    Translation3d delta = other.center.minus(center);
    double distance = delta.getNorm();
    double reach = radius + other.radius + margin;
    if (distance > reach) {
      return null;
    }

    Translation3d normal = distance > 1e-9 ? delta.div(distance) : new Translation3d(1, 0, 0);
    double depth = radius + other.radius - distance;
    return new Contact(normal, depth);
  }

  @Override
  public String toString() {
    return String.format("Sphere3d(center: %s, radius: %.2f)", center, radius);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Sphere3d other) {
      return other.center.equals(center) && other.radius == radius;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(center, radius);
  }
}
