package frc.jsim.collision;

/**
 * Infinite half-space collider.
 *
 * <p>The plane is defined in the body's local frame as: n·p = d, where n is the outward
 * normal and d is the signed distance from the body's local origin. For a floor at Z=0
 * with normal pointing up, use normal (0,0,1) and offset 0.
 *
 * <p>PlaneColliders must be attached to {@link frc.jsim.dynamics.RigidBodyFlags#STATIC} bodies.
 * They produce an AABB that covers the entire simulation volume.
 */
public final class PlaneCollider extends ColliderShape {
    /** Outward normal in body-local space (should be unit length). */
    public final double nx, ny, nz;
    /** Signed distance from the body origin to the plane surface along the normal. */
    public final double offset;

    public PlaneCollider(double nx, double ny, double nz, double offset) {
        double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        this.nx = nx / len; this.ny = ny / len; this.nz = nz / len;
        this.offset = offset;
    }

    @Override
    public Type getType() { return Type.PLANE; }

    @Override
    public void computeAABB(double posX, double posY, double posZ,
                             double qW, double qX, double qY, double qZ,
                             double[] out) {
        double big = 1e9;
        out[0] = -big; out[1] = -big; out[2] = -big;
        out[3] =  big; out[4] =  big; out[5] =  big;
    }
}
