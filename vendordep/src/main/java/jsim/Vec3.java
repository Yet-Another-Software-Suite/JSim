// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

/**
 * Immutable 3D vector used for positions, velocities, and accelerations.
 *
 * @param x the x component
 * @param y the y component
 * @param z the z component
 *
 * <p>Usage example:
 * <pre>{@code
 * Vec3 origin = Vec3.ZERO;
 * Vec3 offset = new Vec3(0.15, 0.0, 0.30); // 15 cm forward, 30 cm up
 * // Vec3 is an immutable record:
 * double x = offset.x();
 * double y = offset.y();
 * double z = offset.z();
 * }</pre>
 */
public record Vec3(double x, double y, double z) {
  /** Zero vector with all components set to 0.0. */
  public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);
}
