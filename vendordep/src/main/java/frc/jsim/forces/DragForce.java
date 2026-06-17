package frc.jsim.forces;

import frc.jsim.dynamics.RigidBody;

/**
 * Velocity-proportional linear drag applied to a single body.
 *
 * <p>Models aerodynamic drag as F_drag = −b · v (linear model). For a more accurate
 * quadratic model (F ∝ v²), extend this class or write a custom {@link ForceGenerator}.
 *
 * <p>Angular drag is applied similarly: τ_drag = −b_rot · ω.
 */
public final class DragForce implements ForceGenerator {
    private final RigidBody body;
    /** Linear drag coefficient (N·s/m). */
    private final double linearDamping;
    /** Rotational drag coefficient (N·m·s/rad). */
    private final double angularDamping;

    /**
     * @param body            the body to damp
     * @param linearDamping   linear drag coefficient (N·s/m)
     * @param angularDamping  rotational drag coefficient (N·m·s/rad)
     */
    public DragForce(RigidBody body, double linearDamping, double angularDamping) {
        this.body = body;
        this.linearDamping = linearDamping;
        this.angularDamping = angularDamping;
    }

    @Override
    public void apply(double dt) {
        if (body.isStatic()) return;
        body.applyForce(
            -linearDamping * body.velX,
            -linearDamping * body.velY,
            -linearDamping * body.velZ);
        body.applyTorque(
            -angularDamping * body.omX,
            -angularDamping * body.omY,
            -angularDamping * body.omZ);
    }
}
