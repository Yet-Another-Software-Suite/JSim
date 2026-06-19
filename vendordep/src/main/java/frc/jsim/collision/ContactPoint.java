package frc.jsim.collision;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import frc.jsim.dynamics.RigidBody;

/**
 * A single contact between two rigid bodies.
 *
 * <p>Convention: the contact normal points from body B toward body A (i.e. the direction
 * body A is pushed to resolve the overlap). Penetration depth is positive when overlapping.
 */
public final class ContactPoint {
    /** First body (the one pushed in the normal direction). */
    public RigidBody bodyA;
    /** Second body. */
    public RigidBody bodyB;

    /** World-space contact point (midpoint of overlap region), in metres. */
    public double contactX, contactY, contactZ;

    /** World-space contact normal (unit vector, dimensionless, points A←B). */
    public double normalX, normalY, normalZ;

    /** Penetration depth (positive = overlapping), in metres. */
    public double penetration;

    /** Combined friction coefficient (dimensionless), filled in by the constraint builder. */
    public double combinedFriction;
    /** Combined restitution coefficient (dimensionless), filled in by the constraint builder. */
    public double combinedRestitution;

    /** Accumulated normal impulse for warm-starting (N·s), reset each tick. */
    public double accumulatedNormalImpulse;
    /** Accumulated tangent impulse magnitude (N·s). */
    public double accumulatedTangentImpulse;

    /** Creates an unpopulated contact point; call {@link #set} to fill it in. */
    public ContactPoint() {}

    /**
     * Populate this contact with the given collision data.
     *
     * @param a           first body
     * @param b           second body
     * @param cx          contact point X
     * @param cy          contact point Y
     * @param cz          contact point Z
     * @param nx          contact normal X (unit vector)
     * @param ny          contact normal Y (unit vector)
     * @param nz          contact normal Z (unit vector)
     * @param penetration overlap depth (positive = overlapping, metres)
     */
    public void set(RigidBody a, RigidBody b,
                    Distance cx, Distance cy, Distance cz,
                    double nx, double ny, double nz,
                    double penetration) {
        this.bodyA = a; this.bodyB = b;
        this.contactX = cx.in(Meters); this.contactY = cy.in(Meters); this.contactZ = cz.in(Meters);
        this.normalX = nx; this.normalY = ny; this.normalZ = nz;
        this.penetration = penetration;
        this.accumulatedNormalImpulse = 0;
        this.accumulatedTangentImpulse = 0;
    }
}
