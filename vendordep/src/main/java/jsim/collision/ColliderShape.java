package jsim.collision;

import edu.wpi.first.math.geometry.Pose3d;

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
     * Compute the world-space AABB of this shape given the body's pose.
     *
     * @param pose world-space pose of the body (position in metres, orientation as quaternion)
     * @param out  receives [minX, minY, minZ, maxX, maxY, maxZ] in metres
     */
    public abstract void computeAABB(Pose3d pose, double[] out);
}
