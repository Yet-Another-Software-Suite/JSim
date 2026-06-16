package frc.jsim.collision;

/** Sphere shape centred at the body's local origin. */
public final class SphereCollider extends ColliderShape {
    public final double radius;

    public SphereCollider(double radius) {
        if (radius <= 0) throw new IllegalArgumentException("radius must be > 0");
        this.radius = radius;
    }

    @Override
    public Type getType() { return Type.SPHERE; }

    @Override
    public void computeAABB(double posX, double posY, double posZ,
                             double qW, double qX, double qY, double qZ,
                             double[] out) {
        out[0] = posX - radius; out[1] = posY - radius; out[2] = posZ - radius;
        out[3] = posX + radius; out[4] = posY + radius; out[5] = posZ + radius;
    }
}
