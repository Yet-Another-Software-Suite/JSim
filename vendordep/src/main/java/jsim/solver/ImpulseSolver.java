package jsim.solver;

import java.util.ArrayList;
import java.util.List;
import jsim.collision.ContactPoint;
import jsim.core.Mat3;
import jsim.core.SimConstants;
import jsim.core.Vec3;
import jsim.dynamics.RigidBody;

/**
 * Sequential impulse (Projected Gauss-Seidel) constraint solver.
 *
 * <p>For each contact the solver iterates:
 * <ol>
 *   <li>Compute relative velocity at the contact point.
 *   <li>Project a normal impulse j_n ≥ 0 that opposes penetration (with restitution).
 *   <li>Project a tangential impulse j_t clamped by Coulomb: |j_t| ≤ μ · j_n.
 *   <li>Apply accumulated impulses directly to each body's velocity state.
 * </ol>
 *
 * <p>Both linear and angular response are included: the angular term uses the
 * pre-computed world-frame inverse inertia tensor {@link RigidBody#invIWorld}.
 */
public final class ImpulseSolver {
    private final int iterations;

    /** Create a solver with the default iteration count. */
    public ImpulseSolver() {
        this(SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    /**
     * Create a solver with a custom iteration count.
     *
     * @param iterations number of PGS passes per tick (higher = more accurate, slower)
     */
    public ImpulseSolver(int iterations) {
        this.iterations = iterations;
    }

    /**
     * Build constraint data from contacts, then run {@code iterations} solve passes.
     *
     * @param contacts the contact points to resolve
     * @param dt       simulation timestep (seconds)
     */
    public void solve(List<ContactPoint> contacts, double dt) {
        if (contacts.isEmpty()) return;
        List<ContactConstraint> constraints = buildConstraints(contacts, dt);
        for (int iter = 0; iter < iterations; iter++) {
            for (ContactConstraint c : constraints) {
                solveNormal(c);
                solveTangent(c);
            }
        }
    }

    // Pre-computation

    private List<ContactConstraint> buildConstraints(List<ContactPoint> contacts, double dt) {
        List<ContactConstraint> out = new ArrayList<>(contacts.size());
        for (ContactPoint cp : contacts) {
            ContactConstraint c = new ContactConstraint(cp);
            RigidBody a = cp.bodyA, b = cp.bodyB;
            double[] n = Vec3.of(cp.normalX, cp.normalY, cp.normalZ);

            // Vectors from CoM to contact
            c.rAx = cp.contactX - a.posX;
            c.rAy = cp.contactY - a.posY;
            c.rAz = cp.contactZ - a.posZ;
            c.rBx = cp.contactX - b.posX;
            c.rBy = cp.contactY - b.posY;
            c.rBz = cp.contactZ - b.posZ;

            double[] rA = Vec3.of(c.rAx, c.rAy, c.rAz);
            double[] rB = Vec3.of(c.rBx, c.rBy, c.rBz);

            // Effective mass for normal impulse
            double[] rAxN = Vec3.cross(rA, n);
            double[] rBxN = Vec3.cross(rB, n);
            c.normalEffMass = effectiveMass(a, b, rAxN, rBxN);

            // Relative velocity at contact
            double[] vRel = relativeVelocity(a, rA, b, rB);
            double vRelN = Vec3.dot(vRel, n);

            // Baumgarte bias
            double slop = SimConstants.PENETRATION_SLOP;
            c.normalBias = -(SimConstants.BAUMGARTE_BETA / dt)
                * Math.max(0, cp.penetration - slop);

            // Restitution bias — stored as a constant target; PGS converges vRelN → -restitutionBias.
            // Sign: vRelN < 0 when approaching, so e*vRelN < 0, and lambda = -(vRelN + bias)/K
            // yields -(1+e)*vRelN/K > 0 (separating impulse) on the first iteration.
            double e = cp.combinedRestitution;
            if (-vRelN > SimConstants.RESTITUTION_VELOCITY_THRESHOLD) {
                c.restitutionBias = e * vRelN;
            } else {
                c.restitutionBias = 0;
            }

            // Tangent direction (remove normal component from relative velocity)
            double[] tangent = Vec3.sub(vRel, Vec3.scale(n, vRelN));
            double tangLen = Vec3.length(tangent);
            if (tangLen > 1e-6) {
                c.tangX = tangent[0] / tangLen;
                c.tangY = tangent[1] / tangLen;
                c.tangZ = tangent[2] / tangLen;
            } else {
                // Arbitrary orthogonal direction
                double[] perp = (Math.abs(n[0]) < 0.9) ? Vec3.of(1, 0, 0) : Vec3.of(0, 1, 0);
                double[] t = Vec3.normalize(Vec3.cross(n, perp));
                c.tangX = t[0]; c.tangY = t[1]; c.tangZ = t[2];
            }

            double[] t = Vec3.of(c.tangX, c.tangY, c.tangZ);
            double[] rAxT = Vec3.cross(rA, t);
            double[] rBxT = Vec3.cross(rB, t);
            c.tangEffMass = effectiveMass(a, b, rAxT, rBxT);

            out.add(c);
        }
        return out;
    }

    // Normal impulse

    private void solveNormal(ContactConstraint c) {
        ContactPoint cp = c.contact;
        RigidBody a = cp.bodyA, b = cp.bodyB;
        double[] n = Vec3.of(cp.normalX, cp.normalY, cp.normalZ);
        double[] rA = Vec3.of(c.rAx, c.rAy, c.rAz);
        double[] rB = Vec3.of(c.rBx, c.rBy, c.rBz);

        double[] vRel = relativeVelocity(a, rA, b, rB);
        double vRelN = Vec3.dot(vRel, n);

        double bias = c.normalBias + c.restitutionBias;

        double lambda = -(vRelN + bias) / c.normalEffMass;

        // Accumulate and clamp: normal impulse must be ≥ 0 (can't pull bodies together)
        double oldAcc = cp.accumulatedNormalImpulse;
        cp.accumulatedNormalImpulse = Math.max(0, oldAcc + lambda);
        double dLambda = cp.accumulatedNormalImpulse - oldAcc;

        applyImpulse(a, b, rA, rB, n, dLambda);
    }

    // Tangential (friction) impulse

    private void solveTangent(ContactConstraint c) {
        ContactPoint cp = c.contact;
        RigidBody a = cp.bodyA, b = cp.bodyB;
        double[] t = Vec3.of(c.tangX, c.tangY, c.tangZ);
        double[] rA = Vec3.of(c.rAx, c.rAy, c.rAz);
        double[] rB = Vec3.of(c.rBx, c.rBy, c.rBz);

        double[] vRel = relativeVelocity(a, rA, b, rB);
        double vRelT = Vec3.dot(vRel, t);

        double lambda = -vRelT / c.tangEffMass;

        // Coulomb friction cone
        double maxFriction = cp.combinedFriction * cp.accumulatedNormalImpulse;
        double oldAcc = cp.accumulatedTangentImpulse;
        cp.accumulatedTangentImpulse = Math.max(-maxFriction, Math.min(maxFriction, oldAcc + lambda));
        double dLambda = cp.accumulatedTangentImpulse - oldAcc;

        applyImpulse(a, b, rA, rB, t, dLambda);
    }

    // Helpers

    /** Relative velocity of body A's contact point with respect to body B's. */
    private static double[] relativeVelocity(RigidBody a, double[] rA, RigidBody b, double[] rB) {
        // v_contact_A = v_A + ω_A × r_A
        double[] omA = Vec3.of(a.omX, a.omY, a.omZ);
        double[] omB = Vec3.of(b.omX, b.omY, b.omZ);
        double[] vA = Vec3.add(Vec3.of(a.velX, a.velY, a.velZ), Vec3.cross(omA, rA));
        double[] vB = Vec3.add(Vec3.of(b.velX, b.velY, b.velZ), Vec3.cross(omB, rB));
        return Vec3.sub(vA, vB);
    }

    /**
     * Effective mass along axis {@code ax}: 1/m_A + 1/m_B + (rAxAx)^T I_A^-1 (rAxAx)
     * + (rBxAx)^T I_B^-1 (rBxAx).
     */
    private static double effectiveMass(RigidBody a, RigidBody b,
                                         double[] rAxAxis, double[] rBxAxis) {
        double[] invIA_r = Mat3.mulVec(a.invIWorld, rAxAxis);
        double[] invIB_r = Mat3.mulVec(b.invIWorld, rBxAxis);
        return a.invMass + b.invMass
             + Vec3.dot(rAxAxis, invIA_r)
             + Vec3.dot(rBxAxis, invIB_r);
    }

    /** Apply a scalar impulse along {@code axis} to both bodies (A positive, B negative). */
    private static void applyImpulse(RigidBody a, RigidBody b,
                                      double[] rA, double[] rB,
                                      double[] axis, double lambda) {
        if (lambda == 0) return;

        if (!a.isStatic()) {
            a.velX += a.invMass * lambda * axis[0];
            a.velY += a.invMass * lambda * axis[1];
            a.velZ += a.invMass * lambda * axis[2];
            double[] dOmA = Mat3.mulVec(a.invIWorld, Vec3.cross(rA, Vec3.scale(axis, lambda)));
            a.omX += dOmA[0]; a.omY += dOmA[1]; a.omZ += dOmA[2];
        }

        if (!b.isStatic()) {
            b.velX -= b.invMass * lambda * axis[0];
            b.velY -= b.invMass * lambda * axis[1];
            b.velZ -= b.invMass * lambda * axis[2];
            double[] dOmB = Mat3.mulVec(b.invIWorld, Vec3.cross(rB, Vec3.scale(axis, -lambda)));
            b.omX += dOmB[0]; b.omY += dOmB[1]; b.omZ += dOmB[2];
        }
    }
}
