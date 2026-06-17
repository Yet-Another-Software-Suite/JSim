package frc.jsim.collision;

/**
 * A collision shape attached to a rigid body.
 *
 * <p>Shapes are defined in body-local space. The physics world transforms them to
 * world space when building AABBs and running narrowphase tests.
 */
public abstract class ColliderShape {
    /** Shape type tag for fast dispatch in narrowphase. */
    public enum Type {
        /** Sphere defined by a radius. */
        SPHERE,
        /** Axis-aligned box defined by half-extents. */
        BOX,
        /** Infinite half-space defined by a normal and offset. */
        PLANE
    }

    /**
     * The shape type of this collider.
     *
     * @return this shape's type tag
     */
    public abstract Type getType();

    /**
     * Compute the world-space AABB of this shape given the body's centre position
     * and orientation quaternion (w,x,y,z).
     *
     * @param posX world-space X position (metres)
     * @param posY world-space Y position (metres)
     * @param posZ world-space Z position (metres)
     * @param qW   orientation quaternion W component (unit quaternion)
     * @param qX   orientation quaternion X component (unit quaternion)
     * @param qY   orientation quaternion Y component (unit quaternion)
     * @param qZ   orientation quaternion Z component (unit quaternion)
     * @param out  receives [minX, minY, minZ, maxX, maxY, maxZ] in metres
     */
    public abstract void computeAABB(
        double posX, double posY, double posZ,
        double qW,   double qX,   double qY, double qZ,
        double[] out);
}
