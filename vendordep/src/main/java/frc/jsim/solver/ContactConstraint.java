package frc.jsim.solver;

import frc.jsim.collision.ContactPoint;

/**
 * Pre-computed data for a single contact constraint, cached before the solver
 * iterates so that the inner loop contains only arithmetic.
 *
 * <p>A constraint represents: "prevent relative penetration along the contact normal,
 * and resist sliding via Coulomb friction."
 */
public final class ContactConstraint {
    /** The contact point this constraint was built for. */
    public final ContactPoint contact;

    /** Vector from body A's CoM to the contact point, world frame (metres). */
    public double rAx, rAy, rAz;
    /** Vector from body B's CoM to the contact point, world frame (metres). */
    public double rBx, rBy, rBz;

    /** Effective inverse mass along the contact normal (kg⁻¹). */
    public double normalEffMass;

    /** First friction tangent direction, world frame (unit vector). */
    public double tangX, tangY, tangZ;
    /** Effective inverse mass along the tangent direction (kg⁻¹). */
    public double tangEffMass;

    /** Baumgarte bias velocity for penetration correction (m/s). */
    public double normalBias;

    /** Restitution target velocity used on the first solver iteration (m/s). */
    public double restitutionBias;

    /**
     * @param contact the contact point to constrain
     */
    public ContactConstraint(ContactPoint contact) {
        this.contact = contact;
    }
}
