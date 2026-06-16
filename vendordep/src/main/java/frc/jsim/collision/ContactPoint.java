package frc.jsim.collision;

import frc.jsim.dynamics.RigidBody;

/**
 * A single contact between two rigid bodies.
 *
 * <p>Convention: the contact normal points from body B toward body A (i.e. the direction
 * body A is pushed to resolve the overlap). Penetration depth is positive when overlapping.
 */
public final class ContactPoint {
    public RigidBody bodyA;
    public RigidBody bodyB;

    /** World-space contact point (midpoint of overlap region). */
    public double contactX, contactY, contactZ;

    /** World-space contact normal (unit vector, points A←B). */
    public double normalX, normalY, normalZ;

    /** Penetration depth (positive = overlapping). */
    public double penetration;

    /** Combined friction and restitution, filled in by the constraint builder. */
    public double combinedFriction;
    public double combinedRestitution;

    /** Accumulated normal impulse for warm-starting (reset each tick). */
    public double accumulatedNormalImpulse;
    /** Accumulated tangent impulse magnitude. */
    public double accumulatedTangentImpulse;

    public ContactPoint() {}

    public void set(RigidBody a, RigidBody b,
                    double cx, double cy, double cz,
                    double nx, double ny, double nz,
                    double penetration) {
        this.bodyA = a; this.bodyB = b;
        this.contactX = cx; this.contactY = cy; this.contactZ = cz;
        this.normalX = nx; this.normalY = ny; this.normalZ = nz;
        this.penetration = penetration;
        this.accumulatedNormalImpulse = 0;
        this.accumulatedTangentImpulse = 0;
    }
}
