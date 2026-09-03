package jsim.physics.layers.utils;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * The result of resolving one 3D shape against another for rigid-body-style collision response --
 * shared by {@link Cuboid3d#overlapWithSphere} and {@link Sphere3d#overlapWithSphere} so callers
 * that bounce off either kind of surface (see {@code FuelLayer#bounce}) don't need to care which.
 *
 * @param normal Unit vector pointing away from the surface being resolved against, in
 *     field-relative meters.
 * @param depth How far past that surface the other shape has penetrated along {@code normal}.
 *     Positive means genuine overlap; some callers (see the {@code margin} parameter on
 *     {@link Cuboid3d#overlapWithSphere(Sphere3d, double)}) permit this to be negative, meaning
 *     that much separation still remains.
 */
public record Contact(Translation3d normal, double depth) {

  /** The vector to add to the overlapping shape's center to just clear this contact. */
  public Translation3d pushOut() {
    return normal.times(depth);
  }
}
