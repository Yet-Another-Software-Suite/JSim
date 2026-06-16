package frc.jsim.collision;

/**
 * A collision shape attached to a rigid body.
 *
 * <p>Shapes are defined in body-local space. The physics world transforms them to
 * world space when building AABBs and running narrowphase tests.
 */
public abstract class ColliderShape {
    /** Shape type tag for fast dispatch in narrowphase. */
    public enum Type { SPHERE, BOX, PLANE }

    public abstract Type getType();

    /**
     * Compute the world-space AABB of this shape given the body's centre position
     * and orientation quaternion (w,x,y,z).
     *
     * @param out receives [minX, minY, minZ, maxX, maxY, maxZ]
     */
    public abstract void computeAABB(
        double posX, double posY, double posZ,
        double qW,   double qX,   double qY, double qZ,
        double[] out);
}
