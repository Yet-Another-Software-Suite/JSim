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
    public final ContactPoint contact;

    // Vectors from each body's CoM to the contact point (world frame)
    public double rAx, rAy, rAz;
    public double rBx, rBy, rBz;

    // Effective mass along the contact normal
    public double normalEffMass;

    // Tangent direction (first friction direction)
    public double tangX, tangY, tangZ;
    public double tangEffMass;

    // Bias velocity for Baumgarte penetration correction
    public double normalBias;

    // Restitution target velocity (used only on first iteration)
    public double restitutionBias;

    public ContactConstraint(ContactPoint contact) {
        this.contact = contact;
    }
}
