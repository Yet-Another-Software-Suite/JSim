package jsim.physics.layers.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

/**
 * A 3D rectangular prism with translational, rotational, and scaling components -- the
 * {@link edu.wpi.first.math.geometry.Rectangle2d Rectangle2d} of one more dimension.
 *
 * <p>Like {@code Rectangle2d}, a {@code Cuboid3d} is defined by a {@link Pose3d} center (giving it
 * both a position and an orientation) plus a full size along each of its own local X, Y and Z
 * axes. That orientation is what lets one shape describe both an axis-aligned field structure
 * (built with {@link Rotation3d#kZero}) and a box that has to track a moving, turning robot (see
 * {@code FuelLayer}'s intake pickup zones) without the caller hand-rolling the rotation math
 * itself every step.
 *
 * @see edu.wpi.first.math.geometry.Rectangle2d
 */
public class Cuboid3d {

  private final Pose3d center;
  private final double xWidth;
  private final double yWidth;
  private final double zWidth;

  /**
   * Constructs a cuboid at the specified position with the specified size.
   *
   * @param center The position (translation and rotation) of the cuboid.
   * @param xWidth The x size component of the cuboid, in its own unrotated frame.
   * @param yWidth The y size component of the cuboid, in its own unrotated frame.
   * @param zWidth The z size component of the cuboid, in its own unrotated frame.
   */
  public Cuboid3d(Pose3d center, double xWidth, double yWidth, double zWidth) {
    if (xWidth < 0 || yWidth < 0 || zWidth < 0) {
      throw new IllegalArgumentException("Cuboid3d dimensions cannot be less than 0!");
    }
    this.center = center;
    this.xWidth = xWidth;
    this.yWidth = yWidth;
    this.zWidth = zWidth;
  }

  /**
   * Constructs a cuboid at the specified position with the specified size, in measured units.
   *
   * @param center The position (translation and rotation) of the cuboid.
   * @param xWidth The x size component of the cuboid, in its own unrotated frame.
   * @param yWidth The y size component of the cuboid, in its own unrotated frame.
   * @param zWidth The z size component of the cuboid, in its own unrotated frame.
   */
  public Cuboid3d(Pose3d center, Distance xWidth, Distance yWidth, Distance zWidth) {
    this(center, xWidth.in(Meters), yWidth.in(Meters), zWidth.in(Meters));
  }

  /**
   * Creates an unrotated cuboid from two diagonally opposite corners.
   *
   * @param cornerA The first corner of the cuboid.
   * @param cornerB The corner diagonally opposite {@code cornerA}.
   */
  public Cuboid3d(Translation3d cornerA, Translation3d cornerB) {
    this(
        new Pose3d(cornerA.plus(cornerB).div(2.0), Rotation3d.kZero),
        Math.abs(cornerA.getX() - cornerB.getX()),
        Math.abs(cornerA.getY() - cornerB.getY()),
        Math.abs(cornerA.getZ() - cornerB.getZ()));
  }

  /** The center of the cuboid. */
  public Pose3d getCenter() {
    return center;
  }

  /** The rotational component of the cuboid. */
  public Rotation3d getRotation() {
    return center.getRotation();
  }

  /** The x size component of the cuboid, in its own unrotated frame. */
  public double getXWidth() {
    return xWidth;
  }

  /** The y size component of the cuboid, in its own unrotated frame. */
  public double getYWidth() {
    return yWidth;
  }

  /** The z size component of the cuboid, in its own unrotated frame. */
  public double getZWidth() {
    return zWidth;
  }

  /** The x size component of the cuboid, in a measure. */
  public Distance getMeasureXWidth() {
    return Meters.of(xWidth);
  }

  /** The y size component of the cuboid, in a measure. */
  public Distance getMeasureYWidth() {
    return Meters.of(yWidth);
  }

  /** The z size component of the cuboid, in a measure. */
  public Distance getMeasureZWidth() {
    return Meters.of(zWidth);
  }

  /**
   * Transforms the center of the cuboid and returns the new cuboid.
   *
   * @param other The transform to transform by.
   * @return The transformed cuboid.
   */
  public Cuboid3d transformBy(Transform3d other) {
    return new Cuboid3d(center.transformBy(other), xWidth, yWidth, zWidth);
  }

  /**
   * Rotates the center of the cuboid and returns the new cuboid.
   *
   * @param other The rotation to transform by.
   * @return The rotated cuboid.
   */
  public Cuboid3d rotateBy(Rotation3d other) {
    return new Cuboid3d(center.rotateBy(other), xWidth, yWidth, zWidth);
  }

  /**
   * Checks if a point is contained within the cuboid. This is inclusive: a point lying exactly on
   * a face returns {@code true}.
   *
   * @param point The point to check.
   * @return Whether the cuboid contains the point.
   */
  public boolean contains(Translation3d point) {
    Translation3d local = toLocal(point);
    return Math.abs(local.getX()) <= xWidth / 2.0
        && Math.abs(local.getY()) <= yWidth / 2.0
        && Math.abs(local.getZ()) <= zWidth / 2.0;
  }

  /**
   * Returns the nearest point contained within the cuboid.
   *
   * @param point The point to find the nearest point to.
   * @return A new point nearest to {@code point} and contained in the cuboid.
   */
  public Translation3d nearest(Translation3d point) {
    Translation3d local = toLocal(point);
    Translation3d clamped = new Translation3d(
        MathUtil.clamp(local.getX(), -xWidth / 2.0, xWidth / 2.0),
        MathUtil.clamp(local.getY(), -yWidth / 2.0, yWidth / 2.0),
        MathUtil.clamp(local.getZ(), -zWidth / 2.0, zWidth / 2.0));
    return fromLocal(clamped);
  }

  /**
   * Returns the distance between the surface of the cuboid and the point.
   *
   * @param point The point to check.
   * @return The distance (0 if the point is contained by the cuboid).
   */
  public double getDistance(Translation3d point) {
    return nearest(point).getDistance(point);
  }

  /**
   * Returns the distance between the surface of the cuboid and the point, in a measure.
   *
   * @param point The point to check.
   * @return The distance (0 if the point is contained by the cuboid), in a measure.
   */
  public Distance getMeasureDistance(Translation3d point) {
    return Meters.of(getDistance(point));
  }

  /**
   * Resolves a sphere against this cuboid, for rigid-body-style collision response.
   *
   * <p>Returns {@code null} if the sphere is further than {@code radius} from the cuboid's
   * surface. Otherwise returns the outward {@link Contact#normal()} the sphere should bounce off
   * of and how far it has penetrated the surface along that normal -- {@link Contact#pushOut()} is
   * the vector that, added to the sphere's center, would just clear the overlap.
   *
   * <p>A sphere whose center has already crossed into the cuboid resolves through whichever face
   * it is closest to, so a ball that tunnels fully inside a thin structure in one step still pops
   * back out the nearest side rather than getting stuck.
   *
   * @param sphereCenter Field-relative center of the sphere.
   * @param radius Radius of the sphere.
   * @return The contact normal and penetration depth, or {@code null} if the two don't overlap.
   */
  public Contact overlapWithSphere(Translation3d sphereCenter, double radius) {
    return overlapWithSphere(sphereCenter, radius, 0.0);
  }

  /**
   * Resolves a sphere against this cuboid, as {@link #overlapWithSphere(Translation3d, double)},
   * but widening detection by {@code margin} beyond the sphere's surface.
   *
   * <p>A positive margin reports contacts the sphere hasn't quite reached yet -- {@link
   * Contact#depth()} comes back negative for those, meaning that much separation remains. Callers
   * that only want to react to genuine overlap should check {@code depth() >= 0} (or use the
   * zero-margin overload); the margin exists for callers that want to decide for themselves
   * whether a near-miss should still count as resting contact (see {@code FuelLayer}'s handling of
   * a ball settling onto a surface).
   *
   * @param sphereCenter Field-relative center of the sphere.
   * @param radius Radius of the sphere.
   * @param margin Extra reach, in meters, beyond the sphere's surface to still report a contact for.
   * @return The contact normal and penetration depth (possibly negative, within {@code margin}),
   *     or {@code null} if nothing is within reach.
   */
  public Contact overlapWithSphere(Translation3d sphereCenter, double radius, double margin) {
    double hx = xWidth / 2.0;
    double hy = yWidth / 2.0;
    double hz = zWidth / 2.0;
    double reach = radius + margin;

    Translation3d local = toLocal(sphereCenter);
    double lx = local.getX();
    double ly = local.getY();
    double lz = local.getZ();

    if (lx < -hx - reach || lx > hx + reach
        || ly < -hy - reach || ly > hy + reach
        || lz < -hz - reach || lz > hz + reach) {
      return null;
    }

    double closestX = MathUtil.clamp(lx, -hx, hx);
    double closestY = MathUtil.clamp(ly, -hy, hy);
    double closestZ = MathUtil.clamp(lz, -hz, hz);
    double dx = lx - closestX;
    double dy = ly - closestY;
    double dz = lz - closestZ;
    double distanceSquared = dx * dx + dy * dy + dz * dz;
    if (distanceSquared > reach * reach) {
      return null;
    }

    Translation3d localNormal;
    double depth;
    if (distanceSquared > 1e-12) {
      double distance = Math.sqrt(distanceSquared);
      localNormal = new Translation3d(dx / distance, dy / distance, dz / distance);
      depth = radius - distance;
    } else {
      // The sphere's center is inside the cuboid: escape through whichever face is closest.
      double[] faceDistances = {hx - lx, hx + lx, hy - ly, hy + ly, hz - lz, hz + lz};
      int closestFace = 0;
      for (int face = 1; face < faceDistances.length; face++) {
        if (faceDistances[face] < faceDistances[closestFace]) {
          closestFace = face;
        }
      }
      localNormal = switch (closestFace) {
        case 0 -> new Translation3d(-1, 0, 0);
        case 1 -> new Translation3d(1, 0, 0);
        case 2 -> new Translation3d(0, -1, 0);
        case 3 -> new Translation3d(0, 1, 0);
        case 4 -> new Translation3d(0, 0, -1);
        default -> new Translation3d(0, 0, 1);
      };
      depth = radius + faceDistances[closestFace];
    }

    return new Contact(localNormal.rotateBy(center.getRotation()), depth);
  }

  /** {@code point}, expressed in this cuboid's own rotated, centered frame. */
  private Translation3d toLocal(Translation3d point) {
    return point.minus(center.getTranslation()).rotateBy(center.getRotation().unaryMinus());
  }

  /** The inverse of {@link #toLocal(Translation3d)}: a local-frame point, back in field frame. */
  private Translation3d fromLocal(Translation3d local) {
    return local.rotateBy(center.getRotation()).plus(center.getTranslation());
  }

  @Override
  public String toString() {
    return String.format(
        "Cuboid3d(center: %s, x: %.2f, y: %.2f, z: %.2f)", center, xWidth, yWidth, zWidth);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Cuboid3d other) {
      return other.center.equals(center)
          && other.xWidth == xWidth
          && other.yWidth == yWidth
          && other.zWidth == zWidth;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(center, xWidth, yWidth, zWidth);
  }

  /**
   * The result of resolving a sphere against a {@link Cuboid3d}: the field-relative outward
   * surface normal at the contact point, and how far the sphere has penetrated along it.
   *
   * @param normal Unit vector pointing away from the cuboid's surface, in field-relative meters.
   * @param depth How far past the surface the sphere has penetrated along {@code normal}. Positive
   *     means genuine overlap; see {@link #overlapWithSphere(Translation3d, double, double)} for
   *     when this can be negative.
   */
  public record Contact(Translation3d normal, double depth) {

    /** The vector to add to the sphere's center to just clear this contact. */
    public Translation3d pushOut() {
      return normal.times(depth);
    }
  }
}
