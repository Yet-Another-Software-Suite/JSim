package frc.jsim.collision;

import edu.wpi.first.math.geometry.Pose3d;

/** Sphere shape centred at the body's local origin. */
public final class SphereCollider extends ColliderShape {
    /** Radius of the sphere in metres. */
    public final double radius;

    /**
     * Creates a sphere collider with the given radius.
     *
     * @param radius the sphere radius in metres; must be positive
     */
    public SphereCollider(double radius) {
        if (radius <= 0) throw new IllegalArgumentException("radius must be > 0");
        this.radius = radius;
    }

    @Override
    public Type getType() { return Type.SPHERE; }

    @Override
    public void computeAABB(Pose3d pose, double[] out) {
        double posX = pose.getX(), posY = pose.getY(), posZ = pose.getZ();
        out[0] = posX - radius; out[1] = posY - radius; out[2] = posZ - radius;
        out[3] = posX + radius; out[4] = posY + radius; out[5] = posZ + radius;
    }
}
